/*
 * Uart0.c
 *
 *  Created on: 30-Aug-2021
 *      Author: Nikhil Pachkor
 */

#include <msp430.h>
#include "string.h"


void UART_Init(void)
{
    P3SEL|=BIT3;        //Pin Alternate Function Select UART TX UCA0 Pin3.3 set as UART TX
    P3SEL|=BIT4;        //Pin Alternate Function Select UART RX UCA0 Pin3.4 set as UART RX
    UCA0CTL1|=UCSWRST;  //Resetting USCI for Configuring Uart Module as Indicated in User Guide at page 940
    UCA0CTL0=0x00;      //Choosing all Default Conditions
    UCA0CTL1|=UCSSEL_2; //Selecting SMCLK all other set to default

//    //Setting for 115200 Baud Rate When SMCLK is 1MHz
//    UCA0BR0=9;          //Low Byte   9 for 115200    109 for 9600
//    UCA0BR1=0;          //High Byte
//    UCA0MCTL|=UCBRS1;   //UCBRS2|UCBRF3 for 9600 UCBRS1 for 115200

    //Setting for Baud Rate 115200 when SMCLK is 20MHZ
    UCA0BR0=173;          //Low Byte   9 for 115200    109 for 9600
    UCA0BR1=0;          //High Byte
    UCA0MCTL=UCBRS_5;   //UCBRS2|UCBRF3 for 9600 UCBRS1 for 115200

    UCA0CTL1&=~UCSWRST; //Reset Release of USCI
}

void UART_TX_String(char *String)
{
    int loop=0;
    for(loop=0;loop<strlen(String);loop++)
    {
        while(!(UCA0IFG&UCTXIFG));
        UCA0TXBUF=String[loop];
        while(UCA0CTL1 & UCBUSY);
    }
}

void Newline(void)
{
    while(!(UCA0IFG&UCTXIFG));
    UCA0TXBUF='\n';
    while(UCA0CTL1 & UCBUSY);
}


