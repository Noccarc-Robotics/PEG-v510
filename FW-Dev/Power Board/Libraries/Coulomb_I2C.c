/*
 * Coulomb_I2C.c
 *
 *  Created on: 30-Aug-2021
 *      Author: Nikhil Pachkor
 */

///////////////////////////Coulomb Counter Function Starts here///////////////////////
#include "msp430.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
////////////////////Coulomb Counter Address//////////

#define Coulomb_Counter_Address 0x64;
#define Coulomb_Control_Address 0x01;
#define Coulomb_Voltage_MSB_Address 0x08;
#define Coulomb_Voltage_LSB_Address 0x09;
#define Coulomb_Current_MSB_Address 0x0E;
#define Coulomb_Current_LSB_Address 0x0F;
#define Coulomb_Charge_MSB_Address 0x02;
#define Coulomb_Charge_LSB_Address 0x03;


//////////////////Coulomb Counter Variables/////////////


void I2C0_Init(void)
{

    P3SEL|=BIT0;                //Pin 3.0 Alternate Function Select UCB0SDA

    P3SEL|=BIT1;                //Pin 3.1 Alternate Function Select UCB0SCL

    UCB0CTL1|=(UCSWRST);        //Setting the Software Reset Bit to High for Configuring I2C0

    UCB0CTL0=0b00001111;        //For Setting own,slave Address bit, Master slave mode I2C0 or SPI mode synchronous or asynchronous mode

    UCB0CTL1=0b10010000;        //Setting I2C0 Input Clock to SMCLK and Transmitter Select

    UCB0BR0=50;                 //Baud Rate Setting Register Low Byte for 100Kbps

    UCB0BR1=0;                  //Baud Rate Setting Register High Byte for 100Kbps

    _delay_cycles(1000) ;        //Software Delay

    UCB0CTL1&=~(UCSWRST);       //Setting the Software Reset Bit to low for Enabling I2C0 Module

}

void I2C0_Reset(void)
{
    UCB0CTL1|=(UCSWRST);        //Setting the Software Reset Bit to High for Configuring I2C0
    UCB0CTL1&=~(UCSWRST);
    I2C0_Init();
}

_Bool I2C0_Bus_Status_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(UCB0STAT&UCBBUSY)            //Checking if the Bus is Busy or Not
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C0_Reset();
            return 0;
        }
    }
    return 1;
}

_Bool I2C0_Transmit_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(!(UCB0IFG&UCTXIFG))          //The Flag UCTXIFG is set when Start Condition from previous Statement is Generated so that the Address can be Written.
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C0_Reset();
            return 0;
        }
    }
    return 1;
}

_Bool I2C0_Acknowledge_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(UCB0CTL1&UCTXSTT)            //Waiting for the Start Flag to be cleared,which is cleared when slave Acknowledges the Address
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C0_Reset();
            return 0;
        }
    }
    return 1;
}

_Bool I2C0_Receive_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(!(UCB0IFG&UCRXIFG))
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C0_Reset();
            return 0;
        }
    }
    return 1;
}

_Bool Connected_I2C0_Device(void)
{
    int Read_Temp;
    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    if(!I2C0_Bus_Status_Timeout())
            return 0;
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    if(!I2C0_Transmit_Timeout())
           return 0;
    UCB0TXBUF=Coulomb_Voltage_MSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Acknowledge_Timeout())
            return 0;
    Read_Temp=UCB0RXBUF;

    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    if(UCB0IFG&UCNACKIFG)
    {
        UCB0IFG&=~UCNACKIFG;
        return 0;
    }
    return 1;
}

float Read_Coulomb_Voltage(void)
{
    int Read_Temp;
    volatile unsigned int Voltage_MSB=0;
    volatile unsigned int Voltage_LSB=0;
    volatile int Temporary_Voltage=0;
    volatile float Result_Voltage=0;
    /////////////////////////Byte 1 Reading Start////////////////////////////
    //Master in Transmitting Mode
    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    if(!I2C0_Bus_Status_Timeout())
        return 0;
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    if(!I2C0_Transmit_Timeout())
        return 0;
    UCB0TXBUF=Coulomb_Voltage_MSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Acknowledge_Timeout())
        return 0;
    Read_Temp=UCB0RXBUF;
    //Master In Receiving Mode
    UCB0CTL1&=~UCTR;                    //Turning on I2C in Receive Mode
    UCB0CTL1|=UCTXSTT;                  //Sending Start Condition         //Waiting for the I2C0 to Generate Interrupt Flag that Receive is Complete
    if(!I2C0_Receive_Timeout())
        return 0;
    Voltage_MSB=UCB0RXBUF;                 //Transfering Data From RX buffer to Low Byte
    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    _delay_cycles(10);
    /////////////////////////Byte 1 Reading End////////////////////////////


    /////////////////////////Byte 2 Reading Start////////////////////////////
    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    if(!I2C0_Bus_Status_Timeout())
        return 0;
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    if(!I2C0_Transmit_Timeout())
        return 0;
    UCB0TXBUF=Coulomb_Voltage_LSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Acknowledge_Timeout())
        return 0;
    Read_Temp=UCB0RXBUF;
    //Master In Receiving Mode
    UCB0CTL1&=~UCTR;                    //Turning on I2C0 in Receive Mode
    UCB0CTL1|=UCTXSTT;                  //Sending Start Condition         //Waiting for the I2C0 to Generate Interrupt Flag that Receive is Complete
    if(!I2C0_Receive_Timeout())
        return 0;
    Voltage_LSB=UCB0RXBUF;                 //Transfering Data From RX buffer to Low Byte
    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    _delay_cycles(10);

    /////////////////////////Byte 2 Reading End////////////////////////////
    Voltage_MSB=Voltage_MSB<<8;
    Temporary_Voltage=Voltage_MSB|Voltage_LSB;
    Result_Voltage=Temporary_Voltage*70.8;
    Result_Voltage=Result_Voltage/65535;
    return Result_Voltage;

}

float Read_Coulomb_Current(void)
{
    int Read_Temp;
    unsigned int Current_MSB;
    unsigned int Current_LSB;
    int Temporary_Current;
    float Result_Current;
    //////////////For Reading 2 Bytes
    //Master in Transmitting Mode
    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    if(!I2C0_Bus_Status_Timeout())
        return 0;
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    if(!I2C0_Transmit_Timeout())
        return 0;
    UCB0TXBUF=Coulomb_Current_MSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Acknowledge_Timeout())
        return 0;
    Read_Temp=UCB0RXBUF;

    //Master In Receiving Mode
    UCB0CTL1&=~UCTR;                    //Turning on I2C0 in Receive Mode
    UCB0CTL1|=UCTXSTT;                  //Sending Start Condition         //Waiting for the I2C0 to Generate Interrupt Flag that Receive is Complete
    if(!I2C0_Receive_Timeout())
        return 0;
    Current_MSB=UCB0RXBUF;                 //Transfering Data From RX buffer to Low Byte
    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    _delay_cycles(100);



    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    if(!I2C0_Bus_Status_Timeout())
        return 0;
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    if(!I2C0_Transmit_Timeout())
        return 0;
    UCB0TXBUF=Coulomb_Current_LSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Acknowledge_Timeout())
        return 0;
    Read_Temp=UCB0RXBUF;
    //Master In Receiving Mode
    UCB0CTL1&=~UCTR;                    //Turning on I2C0 in Receive Mode
    UCB0CTL1|=UCTXSTT;                  //Sending Start Condition         //Waiting for the I2C0 to Generate Interrupt Flag that Receive is Complete
    if(!I2C0_Receive_Timeout())
        return 0;
    Current_LSB=UCB0RXBUF;                 //Transfering Data From RX buffer to Low Byte
    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    _delay_cycles(100);


    Current_MSB=Current_MSB<<8;
    Temporary_Current=Current_MSB|Current_LSB;
    Result_Current=(Temporary_Current-32767)*6.4;
    Result_Current=Result_Current/32767;

    return Result_Current;
}

float Read_Coulomb_Charge(void)
{
    int Read_Temp;
    unsigned int Charge_MSB;
    unsigned int Charge_LSB;
    int Temporary_Charge;
    float Result_Charge;
    //////////////For Reading 2 Bytes
    //Master in Transmitting Mode
    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    if(!I2C0_Bus_Status_Timeout())
        return 0;
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    if(!I2C0_Transmit_Timeout())
        return 0;
    UCB0TXBUF=Coulomb_Charge_MSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Acknowledge_Timeout())
        return 0;
    Read_Temp=UCB0RXBUF;

    //Master In Receiving Mode
    UCB0CTL1&=~UCTR;                    //Turning on I2C0 in Receive Mode
    UCB0CTL1|=UCTXSTT;                  //Sending Start Condition         //Waiting for the I2C0 to Generate Interrupt Flag that Receive is Complete
    if(!I2C0_Receive_Timeout())
        return 0;
    Charge_MSB=UCB0RXBUF;                 //Transfering Data From RX buffer to Low Byte
    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    _delay_cycles(100);



    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    if(!I2C0_Bus_Status_Timeout())
        return 0;
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    if(!I2C0_Transmit_Timeout())
        return 0;
    UCB0TXBUF=Coulomb_Charge_LSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Acknowledge_Timeout())
        return 0;
    Read_Temp=UCB0RXBUF;
    //Master In Receiving Mode
    UCB0CTL1&=~UCTR;                    //Turning on I2C0 in Receive Mode
    UCB0CTL1|=UCTXSTT;                  //Sending Start Condition         //Waiting for the I2C0 to Generate Interrupt Flag that Receive is Complete
    if(!I2C0_Receive_Timeout())
        return 0;
    Charge_LSB=UCB0RXBUF;                 //Transfering Data From RX buffer to Low Byte
    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    _delay_cycles(1000);


    Charge_MSB=Charge_MSB<<8;
    Temporary_Charge=Charge_MSB|Charge_LSB;
    Result_Charge=(Temporary_Charge-32766)*5*0.34*256;
    Result_Charge=Result_Charge/4096;

    return Result_Charge;
}

void Coulomb_Counter_Setting(void)
{

    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  // Address of Coulomb Counter
    if(!I2C0_Bus_Status_Timeout())
        return;
    UCB0CTL1|=UCTXSTT;                  //Start Condition
    if(!I2C0_Transmit_Timeout())
        return;
    UCB0TXBUF=Coulomb_Control_Address;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    if(!I2C0_Transmit_Timeout())
        return;
    UCB0TXBUF=0xE4;
    if(!I2C0_Transmit_Timeout())
        return;
    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus

}

void Coulomb_Data_Read(void)
{
    //////////Coulomb Counter Code Start //////////////////////
    if(!(Connected_I2C0_Device()))
    {
        I2C0_Reset();
    }
    _delay_cycles(1000);

    //////////Coulomb Counter Code End //////////////////////
}

/////////////////////////////////////Coulomb Counter Functions Ends Here//////////////////////


