/********************************************************************
** @file     HDC2080_us.c
** @brief    User Space Driver for HDC2080
** @author   Copyright (C) 2021  Noccarc Robotics
** @author   Nipun Pal <nipun.k@noccarc.com>
** @version  v1
** @modified 28/09/21  Module Creation
**
********************************************************************/

// Note : Compile command : ${CC} Temp_Hum_US.c -o hdc -lm

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#define BUFFSIZE 4

// Routine to convert hex values to decimal
int HexadecimalToDecimal(char *hex) {
    int p = 0;
    int decimal = 0;
    int r, i;

    for(i = BUFFSIZE - 1 ; i >= 0 ; --i){

        // converting c[i] to appropriate decimal form
        if(hex[i]>='0'&&hex[i]<='9'){
            r = hex[i] - '0';
        }
        else{
            r = hex[i] - 'a' + 10;
        }

        decimal = decimal + r * pow(16 , p);
        ++p;
    }

    return decimal;
}

/* Routine to perform humidity measurement */
void measure_hum() {

    // Declaring variables required for internal ops
    FILE *fp;
    char databuf[10];
    char packet[4];
    float n_hum = 0;
    int c = 0;

    // Creating a pipe to execute the system command.
    fp = popen("i2ctransfer -f -y 0 w1@0x40 0x02 r2", "r");

    if (fp == NULL) {
        printf("Failed to run command\n" );
        exit(1);
    }

    // Reading the system response from the pipe
    while (fgets(databuf, sizeof(databuf), fp) != NULL) {

        /* Data here comes MSB first and when read 2 bytes of data we get upper nibble first
           then lower nibble. So we need to swap the nibbles and then send for conversion */
        packet[0] = databuf[7];
        packet[1] = databuf[8];
        packet[2] = databuf[2];
        packet[3] = databuf[3];

        // Converting response in string to decimal and
        // normalizing the measurement.
        n_hum = (float)(HexadecimalToDecimal(packet));
        n_hum = ((n_hum/65535) * 100);

        printf("Humidity(%): %f\n", n_hum);

    }

    // Close the pipe for clean exit and de-allocation of OS resources.
    pclose(fp);

}

/* Routine to perform temperature measurement */
void measure_temp() {

    // Declaring variables required for internal ops
    FILE *fp;
    char databuf[10];
    char packet[4];
    float n_temp = 0;
    int c = 0;

    // Creating a pipe to execute the system command.
    fp = popen("i2ctransfer -f -y 0 w1@0x40 0x00 r2", "r");

    if (fp == NULL) {
        printf("Failed to run command\n" );
        exit(1);
    }

    // Reading the system response from the pipe
    while (fgets(databuf, sizeof(databuf), fp) != NULL) {

        packet[0] = databuf[7];
        packet[1] = databuf[8];
        packet[2] = databuf[2];
        packet[3] = databuf[3];

        // Converting response in string to decimal and
        // normalizing the measurement.

        n_temp = (float)(HexadecimalToDecimal(packet));
        n_temp = (((n_temp/65535) * 160) - 40.5);

        printf("Temp(in deg C): %f\n", n_temp);

    }

    // Close the pipe for clean exit and de-allocation of OS resources.
    pclose(fp);

}


int main(){


    printf("Start Measurement ....\n");

    // Enable measurement by writing into Measurement Configuration Register
    system("i2ctransfer -f -y 0 w2@0x40 0x0F 0x01");

    while(1) {

        measure_temp();
        sleep(1);
        measure_hum();
        sleep(1);

    }

    return 0;
}