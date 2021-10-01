/********************************************************************
** @file     ALS_BH1750_us.c
** @brief    User Space Driver for BH1750
** @author   Copyright (C) 2021  Noccarc Robotics
** @author   Nipun Pal <nipun.k@noccarc.com>
** @version  v1.01
** @modified 1/10/21  Module Creation
**
********************************************************************/

/* Note : Compile command : ${CC} .c -o als4 -lm.
   Note : Data is received as @ index :2,3 - HB & @index:7,8 - LB.
   Note : PWM Channel : TIM1_CH4 , Frequency = 1Khz.
   Note : Brightness step value : 5%.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

// Globals
#define BUFFSIZE 4
#define SAMPLESIZE 10
unsigned int g_prev_sample = 0;

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
            r = hex[i] - 'A' + 10;
        }

        decimal = decimal + r * pow(16 , p);
        ++p;
    }

    return decimal;
}

// Take 10 samples and normalize the value
unsigned int normalize_sample(unsigned int *buff) {

    unsigned int res = 0;

    for(int i=0;i<SAMPLESIZE;i++) {

            res = res + buff[i];
    }

    return ((res/SAMPLESIZE));

}

int main() {

    // Declaring variables required for internal ops
    FILE *fp;
    char databuf[10];
    char packet[4];
    unsigned int sample[SAMPLESIZE];
    unsigned int n_data = 0;

    // Initialize the ALS to start the measurement
    system ("i2ctransfer -f -y 0 w1@0x23 0x10");

    // Setting up the PWM channel
    system ("echo 3 > /sys/class/pwm/pwmchip4/export");
    system ("echo 1000000 > /sys/class/pwm/pwmchip4/pwm3/period");
    system ("echo 500000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle");
    system ("echo 1 > /sys/class/pwm/pwmchip4/pwm3/enable");

    // Run forever
    while(1) {

        // Creating a pipe to execute the system command.
        fp = popen("i2ctransfer -f -y 0 r2@0x23", "r");

        if (fp == NULL) {
            printf("Failed to run command\n" );
            exit(1);
        }

        for(int i=0;i<SAMPLESIZE;i++) {

            // Reading the system response from the pipe
            while (fgets(databuf, sizeof(databuf), fp) != NULL) {

                packet[0] = databuf[2];
                packet[1] = databuf[3];
                packet[2] = databuf[7];
                packet[3] = databuf[8];

            }

            sample[i] = HexadecimalToDecimal(packet);

        }

        // Close the pipe for clean exit and de-allocation of OS resources.
        pclose(fp);
        // Get normalized value
        n_data = normalize_sample(sample);
        n_data = (n_data/100);

        /*  Switch case to vary the brightness depending on the normalized ALS data.
            Using global flag g_prev_sample to keep a track of previous state. If the normalized
            value is consistent then execution of the system call is only required once and
            subsequent calls can be skipped.
        */
        switch(n_data) {

            case 1: if(g_prev_sample != n_data) {

                        system("echo 100000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 2: if(g_prev_sample != n_data) {

                        system("echo 150000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 3: if(g_prev_sample != n_data) {

                        system("echo 200000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 4: if(g_prev_sample != n_data) {

                        system("echo 250000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 5: if(g_prev_sample != n_data) {

                        system("echo 300000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 6: if(g_prev_sample != n_data) {

                        system("echo 350000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 7: if(g_prev_sample != n_data) {

                        system("echo 400000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 8: if(g_prev_sample != n_data) {

                        system("echo 450000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 9:  if(g_prev_sample != n_data) {

                        system("echo 500000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 10:if(g_prev_sample != n_data) {

                        system("echo 550000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 11:if(g_prev_sample != n_data) {

                        system("echo 600000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 12:if(g_prev_sample != n_data) {

                        system("echo 650000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 13:if(g_prev_sample != n_data) {

                        system("echo 700000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 14: if(g_prev_sample != n_data) {

                        system("echo 750000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 15:if(g_prev_sample != n_data) {

                        system("echo 800000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 16:if(g_prev_sample != n_data) {

                        system("echo 850000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 17:if(g_prev_sample != n_data) {

                        system("echo 900000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            case 18:if(g_prev_sample != n_data) {

                        system("echo 950000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;

                    } break;

            default: system("echo 1000000 > /sys/class/pwm/pwmchip4/pwm3/duty_cycle"); n_data = 0; g_prev_sample = n_data;
                    break;

        }

    }

    return 0;
}