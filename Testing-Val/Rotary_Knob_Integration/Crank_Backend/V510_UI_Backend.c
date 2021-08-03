/*
 * Copyright 2017, Crank Software Inc. All Rights Reserved.
 * 
 * For more information email info@cranksoftware.com.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
#include <fcntl.h>
#ifdef WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <unistd.h> // for usleep
#endif

#include <gre/greio.h>
#include "ThermostatIO_events.h"

#define THERMOSTAT_SEND_CHANNEL "application"
#define THERMOSTAT_RECEIVE_CHANNEL "thermostat_backend"

#define SIMULATION_MAX_TEMP 35
#define SIMULATION_MIN_TEMP 8
#define SNOOZE_TIME 80
#define SNOOZE_TIME1 1000

static int							dataChanged = 1; //Default to 1 so we send data to the ui once it connects
static knob_data_event_t			knob_state;
#ifdef WIN32
static CRITICAL_SECTION lock;
static HANDLE thread1;
#else 
static pthread_mutex_t lock;
static pthread_t 	thread1;
#endif

//***********************************************************************************************
FILE *fp1, *fp2; 
char *command1 = "cat /sys/bus/counter/devices/counter0/count0/count";
char *command2 = "gpioget gpiochip5 10";
//***********************************************************************************************

/**
 * cross-platform function to create threads
 * @param start_routine This is the function pointer for the thread to run
 * @return 0 on success, otherwise an integer above 1
 */ 
int
create_task(void *start_routine) {
#ifdef WIN32
	thread1 = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) start_routine, NULL, 0, NULL);
	if( thread1 == NULL ) {
		return 1;
	}
	return 0;
#else
	return pthread_create( &thread1, NULL, start_routine, NULL);
#endif
}

/**
 * cross platform mutex initialization
 * @return 0 on success, otherwise an integer above 1
 */ 
int
init_mutex() {
#ifdef WIN32
	InitializeCriticalSection(&lock);
	return 0;
#else
	return pthread_mutex_init(&lock, NULL);
#endif
}

/**
 * cross platform mutex lock
 */ 
void
lock_mutex() {
#ifdef WIN32
	EnterCriticalSection(&lock);
#else
	pthread_mutex_lock(&lock);
#endif
}

/**
 * cross platform mutex unlock
 */ 
void
unlock_mutex() {
#ifdef WIN32
	LeaveCriticalSection(&lock);
#else
	pthread_mutex_unlock(&lock);
#endif
}

/**
 * cross-platform sleep
 */ 
void
sleep_ms(int milliseconds) {
#ifdef WIN32
	Sleep(milliseconds);
#else
	usleep(milliseconds * 1000);
#endif
}

/**
 * Definition for the receive thread
 */
void *
receive_thread(void *arg) {
	gre_io_t					*handle;
	gre_io_serialized_data_t	*nbuffer = NULL;
	char *event_addr;
	char *event_name;
	char *event_format;
	void *event_data;
	int						 ret;
	int nbytes;

	printf("Opening a channel for receive\n");
	// Connect to a channel to receive messages
	handle = gre_io_open(THERMOSTAT_RECEIVE_CHANNEL, GRE_IO_TYPE_RDONLY);
	if (handle == NULL) {
		fprintf(stderr, "Can't open receive channel\n");
		return 0;
	}

	nbuffer = gre_io_size_buffer(NULL, 100);

	while (1) {
		
		ret = gre_io_receive(handle, &nbuffer);
		if (ret < 0) {
			return 0;
		}

		event_name = NULL;
		nbytes = gre_io_unserialize(nbuffer, &event_addr, &event_name, &event_format, &event_data);
		if (!event_name) {
			printf("Missing event name\n");
			return 0;
		}

		printf("Received Event %s nbytes: %d format: %s\n", event_name, nbytes, event_format);

		lock_mutex();
		if (strcmp(event_name,START_VENTI) == 0) {
			Start_event_t *uidata = ( Start_event_t *)event_data;
			printf("Uidata\n");
			printf("%d \n",uidata->Pinsp);
			
		}

		unlock_mutex();
	}

	//Release the buffer memory, close the send handle
	gre_io_free_buffer(nbuffer);
	gre_io_close(handle);
}


//*****************************************************************************************
// Scan the counter value from the linux file system
// and return the latest value to caller.
int scan_counter() {
	
	int count=0;
	fp1=popen(command1,"r");
	fscanf(fp1, "%d" , &count);
	fclose(fp1);
	return count;
}

// Scan the gpio value from the linux file system
// and return the latest value to caller.
int scan_gpio() {
	
	int gpio=0;
	fp2=popen(command2,"r");
	fscanf(fp2, "%d", &gpio);
	fclose(fp2);
	return gpio;
}

//****************************************************************************************

int
main(int argc, char **argv) {
	gre_io_t					*send_handle;
	gre_io_serialized_data_t	*nbuffer = NULL;
	knob_data_event_t			knob_event_data;
	int 						ret;
	time_t						timer = time(NULL);
	double						seconds;
	
	//**************************************************************************************//
	int prev=0,new=0,gpio=0,dir=0,gpio_flag=0;
	prev=new=scan_counter();
	
	memset(&knob_state, 0, sizeof(knob_state));
	knob_state.inc_value=0;
	knob_state.dec_value=0;
	
	//**************************************************************************************//

	if (init_mutex() != 0) {
		fprintf(stderr,"Mutex init failed\n");
		return 0;
	}

	printf("Trying to open the connection to the frontend\n");
	
	while(1) {
		
	 // Connect to a channel to send messages (write)
		sleep_ms(SNOOZE_TIME);
		send_handle = gre_io_open(THERMOSTAT_SEND_CHANNEL, GRE_IO_TYPE_WRONLY);
		if(send_handle != NULL) {
			printf("Send channel: %s successfully opened\n", THERMOSTAT_SEND_CHANNEL);
			break;
		}
	}
	
	//**************************************************************************************
	memset(&knob_event_data, 0, sizeof(knob_event_data));
	//**************************************************************************************

	while(1) {
		
		lock_mutex();
		//**********************************************************************************
		gpio=scan_gpio();
		
		if(!gpio) {

			printf("Button,Pressed..!!\n");
			prev=new;
			new=scan_counter();	
			gpio_flag=1;
			

		} else {
			
			dir=new-prev;
			knob_state.dec_value=0;
			
			if(dir==0) {
			
				knob_state.inc_value=0;
				prev=new;
				new=scan_counter();
				dataChanged=1;
				printf("Direction: At Rest: %d\n",knob_state.inc_value);
				
			
			} else if(dir<0) {
			
				knob_state.inc_value=-1;
				prev=new;
				new=scan_counter();
				dataChanged=1;
				printf("Direction: Backward: %d\n",knob_state.inc_value);
			
			} else {
			
				knob_state.inc_value=1;
				prev=new;
				new=scan_counter();
				dataChanged=1;
				printf("Direction: Forward: %d\n",knob_state.inc_value);
			
			}
		
		}
		//**********************************************************************************
		unlock_mutex();
		
		if (dataChanged) {
			
			lock_mutex();

			//knob_event_data = knob_state;
			dataChanged = 0;
			
			unlock_mutex();
		
			
			if(gpio_flag==1){
				
				knob_state.dec_value='a';
				gpio_flag=0;
				
			}
			
			// Serialize the data to a buffer
			
			if(knob_state.inc_value!=0 || knob_state.dec_value =='a') {
				
				nbuffer = gre_io_serialize(nbuffer, NULL, KNOB_DATA_EVENT, KNOB_DATA_FMT, &knob_state, sizeof(knob_state));
				
				if (!nbuffer) {
					fprintf(stderr, "Can't serialized data to buffer, exiting\n");
					break;
				}
				
			// Send the serialized event buffer
				ret = gre_io_send(send_handle, nbuffer);
				if (ret < 0) {
					fprintf(stderr, "Send failed, exiting\n");
					break;
				}

			}
			
		}
	}

	//Release the buffer memory, close the send handle
	gre_io_free_buffer(nbuffer);
	gre_io_close(send_handle);
	return 0;
}
