/********************************************************************
** @file     ALS_BH1750_us.c
** @brief    User Space Driver for BH1750
** @author   Copyright (C) 2021  Noccarc Robotics
** @author   Nipun Pal <nipun.k@noccarc.com>
** @version  v1
** @modified 21/09/21  Module Creation
**
********************************************************************/

/* Note :Compile command : ${CC} als4.c -o als4 -lm
   Note : Data is received as @ index :2,3 - HB & @index:7,8 - LB
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// PA0 , PD15 -- pwm channels

#define BUFFSIZE 4

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


int main(){


    FILE *fp;
    char databuf[10];
    char packet[4];

    system("i2ctransfer -f -y 0 w1@0x23 0x10");


    while(1) {

        fp = popen("i2ctransfer -f -y 0 r2@0x23", "r");

        if (fp == NULL) {
            printf("Failed to run command\n" );
            exit(1);
        }

        while (fgets(databuf, sizeof(databuf), fp) != NULL) {

            packet[0] = databuf[2];
            packet[1] = databuf[3];
            packet[2] = databuf[7];
            packet[3] = databuf[8];

            printf("Light: %d\n", HexadecimalToDecimal(packet));

        }

        pclose(fp);

    }


    return 0;
}