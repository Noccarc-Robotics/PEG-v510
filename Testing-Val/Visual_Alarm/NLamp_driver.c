#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>

/*****************************************************/
/*****************************************************
Setup details :
VisionSoM-STM32MP157A-carrier board + SoM board.
2 x Leds
4 x Jumper Wires
No DTS changes required , driven by libgpiod interface.

Platform:
OpenSTLinux Package : 3.0.0.
Kernel Version      : 5.10.10

Hardware connections:

Led   |    Port    | Pin Number

Red        PB6       38
Green      PC3       35

*****************************************************/
/*****************************************************/

// Enable / Disable Debug Prints
#define DEBUG 0


// Global variables for synchronization
int             g_break     = 0;
int             g_hp        = 0;
int             g_lp        = 0;
int             g_mp        = 0;

// Mutex locks for various variables
pthread_mutex_t data_lock;
pthread_mutex_t break_lock;

// Notification Lamp thread Callback
void *lamp(void *vargp)
{
    while(1) {

        if(g_hp && !g_break) {

            pthread_mutex_lock(&data_lock);g_hp=0;pthread_mutex_unlock(&data_lock);
            disable_all_channels();
            enable_HP();
        }


        else if(g_mp && !g_break) {

            pthread_mutex_lock(&data_lock); g_mp=0;  pthread_mutex_unlock(&data_lock);
            disable_all_channels();
            enable_MP();
        }


        else if(g_lp && !g_break) {

            pthread_mutex_lock(&data_lock); g_lp=0;  pthread_mutex_unlock(&data_lock);
            disable_all_channels();
            enable_LP();
        }

        else if (g_break) {

            stop();
        }
    }

    return NULL;
}

void disable_all_channels() {

    // Disable all the channel , by pulling down gpio low.

    system("gpioset gpiochip1 6=0 "); // PB6 - Pin 38
    system("gpioset gpiochip2 3=0 "); // PC3 - Pin 35

}

// Enable High Priority Alarm
void enable_HP() {

#if DEBUG
    printf("Enable HP Notification..!!\n");
#endif

    while(!g_break) {

#if DEBUG
        printf("gbreak from HP= %d\n", g_break);
#endif
        system("gpioset gpiochip1 6=1 "); //PB6 - High - Red
        usleep(500000);
        system("gpioset gpiochip1 6=0 "); //PB6 - Low -Red
        usleep(500000);

    }

}

// Enable Medium Priority Alarm
void enable_MP() {

#if DEBUG
    printf("Enable MP Notification..!!\n");
#endif

    while(!g_break) {

#if DEBUG
        printf("gbreak from MP= %d\n", g_break);
#endif
        system("gpioset gpiochip1 6=1 && gpioset gpiochip2 3=1 "); //PB6 && PC3 - High
        sleep(2);
        system("gpioset gpiochip1 6=0 && gpioset gpiochip2 3=0 "); //PB6 && PC3 - Low
        sleep(3);

    }
}

// Enable Low Priority Alarm
void enable_LP() {

#if DEBUG
        printf("Enable LP Notification..!!\n");
#endif

    while(!g_break) {

#if DEBUG
        printf("gbreak from LP= %d\n", g_break);
#endif
        system("gpioset gpiochip1 6=1 && gpioset gpiochip2 3=1 "); //PB6 && PC3 - High
        sleep(1);
        system("gpioset gpiochip1 6=0 && gpioset gpiochip2 3=0 "); //PB6 && PC3 - Low
        sleep(7);

    }

}

// Disable all notifications
void stop() {

#if DEBUG
        printf("Disabling the lamp...!!!\n");
        printf("gbreak from stop= %d\n", g_break);
#endif
        disable_all_channels();
        pthread_mutex_lock(&break_lock); g_break=0;  pthread_mutex_unlock(&break_lock);

}

int main() {

        int       data = 0 ;
        int       rc   = 0 ;
        pthread_t N_Lamp;

        // Create the N_lamp thread
        rc = pthread_create(&N_Lamp, NULL, lamp, NULL);

        /* could not create thread */
        if(rc) {
            printf("\n ERROR: return code from pthread_create is %d \n", rc);
            return 1;
        }

        /* could not init mutex */
        if (pthread_mutex_init(&data_lock, NULL) != 0) {

            printf("\n Data mutex init has failed\n");
            return 1;
        }

        if (pthread_mutex_init(&break_lock, NULL) != 0) {

            printf("\n Break mutex init has failed\n");
            return 1;
        }


        while (1) {

            printf("Enter value to see the alarm in main thread : 1- HP , 2-MP, 3-LP, 4-STP, 5-Exit: ");
            scanf("%d",&data);

            switch(data) {

                case 1: // Setting flag for firing HP alarm by N_Lamp thread
                        pthread_mutex_lock(&data_lock); g_hp=1;  pthread_mutex_unlock(&data_lock);
                        pthread_mutex_lock(&break_lock); g_break=0;  pthread_mutex_unlock(&break_lock);
                        break;
                case 2: // Setting flag for firing MP alarm by N_Lamp thread
                        pthread_mutex_lock(&data_lock); g_mp=1;  pthread_mutex_unlock(&data_lock);
                        pthread_mutex_lock(&break_lock); g_break=0;  pthread_mutex_unlock(&break_lock);
                        break;
                case 3: // Setting flag for firing LP alarm by N_Lamp thread
                        pthread_mutex_lock(&data_lock); g_lp=1;  pthread_mutex_unlock(&data_lock);
                        pthread_mutex_lock(&break_lock); g_break=0;  pthread_mutex_unlock(&break_lock);
                        break;
                case 4: // Setting flag for stopping alarm by N_Lamp thread
                        pthread_mutex_lock(&break_lock); g_break=1;  pthread_mutex_unlock(&break_lock);
                        break;
                case 5: // Exit and Cleanup
                        pthread_mutex_destroy(&break_lock);
                        pthread_mutex_destroy(&data_lock);
                        pthread_join(N_Lamp, NULL);
                        exit(0);
                        break;
                default: break;

            }

        }

        return 0;

}