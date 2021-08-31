/*
 * Alarm_I2C1.c
 *
 *  Created on: 30-Aug-2021
 *      Author: Nikhil Pachkor
 */
#include "msp430.h"
#include "math.h"

#define Alarm_IC_Address 0x47   //Address Number 1 of DAC IC
const int Gain=2;
//volatile int Frequency_K=0;

void I2C1_Init(void)
{
    P4SEL|=BIT1;        //Pin 4.1 Alternate Function Select UCB1SDA
    P4SEL|=BIT2;        //Pin 4.2 Alternate Function Select UCB1SCL
    UCB1CTL1|=(UCSWRST);    //Setting the Software Reset Bit to High for Configuring I2C1
    UCB1CTL0=0b00001111;    //For Setting own,slave addr bitmaster slave mode I2C1 or SPI mode synchronous or asynchronous mode
    UCB1CTL1=0b10010000;    //Setting I2C1 Input Clock to SMCLK and transmitter Select

    ////For Setting Speed to 100Kbps when SMCLK is 1MHZ
    UCB1BR0=11;             //Baud Rate Setting Register Low Byte
    UCB1BR1=0;              //Baud Rate Setting Register High Byte

    /////For Setting Speed to 100Kbps when SMCLK is 20MHz
//    UCB1BR0=200;             //Baud Rate Setting Register Low Byte
//    UCB1BR1=0;              //Baud Rate Setting Register High Byte
    _delay_cycles(1000);
    UCB1CTL1&=~(UCSWRST);    //Setting the Software Reset Bit to low for Enabling I2C1 Module
}

void I2C1_Reset(void)
{
    //4.1 SDA 4.2 SCL
    UCB1CTL1|=(UCSWRST);        //Setting the Software Reset Bit to High for Configuring I2C
    UCB1CTL1&=~UCSWRST;
    I2C1_Init();
    //Alarm_IC_Reset();
}

_Bool I2C1_Bus_Status_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(UCB1STAT&UCBBUSY)            //Checking if the Bus is Busy or Not
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            return 0;
        }
    }
    return 1;
}

_Bool I2C1_Transmit_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(!(UCB1IFG&UCTXIFG))          //The Flag UCTXIFG is set when Start Condition from previous Statement is Generated so that the Address can be Written.
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            return 0;
        }
    }
    return 1;
}

void Alarm_IC_Reset(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xD3;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x00;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x0A;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);
}

void Frequency(int Frequency)
{
    TA0CTL=TA0CTL&~MC_3;
    unsigned int Convert;
    float tem=pow(Frequency,1.004);
    if(Frequency<5000&&Frequency>5)
    {
        Convert=159028/tem;    //Transfer Function for Generating Desired Frequency.
        TA0CCR0=Convert;            //Updating Value of Register.
    }
    else
    {
        TA0CCR0=0;
    }
    TA0CTL|=MC_1;
}

void Timer_A0_Init(void)
{
    P1DIR|=BIT2;        //P1.2 Pin Set as Output Timer A Channel 1 TA0.1
    P1SEL|=BIT2;        //P1.2 Pin Set as Alternate Function Channel 1
    P1OUT|=BIT2;        //P1.2 Pin Set as Alternate Function Channel 1
    P1DS|=BIT2;         //P1.2 Pin Drive Strength full.

    TA0CTL=TASSEL_2+ID_3+MC_1;   //Stopping the Timer
    TA0CCTL1=OUTMOD_4;           //
    TA0EX0=TAIDEX_7;

}

_Bool I2C1_Acknowledge_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(UCB1CTL1&UCTXSTT)            //Waiting for the Start Flag to be cleared,which is cleared when slave Acknowledges the Address
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            return 0;
        }
    }
    return 1;
}

_Bool I2C1_Receive_Timeout(void)
{
    unsigned int Timeout_Count=1000;
    while(!(UCB1IFG&UCRXIFG))
    {
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            return 0;
        }
    }
    return 1;
}

_Bool Connected_I2C1_Device(void)
{
    int Read_Temp=0;
    UCB1CTL1|=UCTR;                     //Turning on I2C in Transmit Mode
    UCB1I2CSA=Alarm_IC_Address;  //Slave Address
    while(UCB1STAT&UCBBUSY);            //Checking if the Bus is Busy or Not
    UCB1CTL1|=UCTXSTT;                  //Start Condition is Sent
    while(!(UCB1IFG&UCTXIFG));          //The Flag UCTXIFG is set when Start Condition from previous Statement is Generated so that the Address can be Written.
    UCB1TXBUF=0xD0;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    while(UCB1CTL1&UCTXSTT);            //Waiting for the Start Flag to be cleared,which is cleared when slave Acknowledges the Address
    Read_Temp=UCB1RXBUF;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C Bus
    if(UCB1IFG&UCNACKIFG)
    {
        UCB1IFG&=~UCNACKIFG;
        return 0;
    }
    return 1;
}

void Update_Volume(int vol)
{

    vol = (vol*10)<<2;

    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
            return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1TXBUF=0x25;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1TXBUF=(vol & 0xff00)>>8;
    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1TXBUF= (vol & 0xff);
    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);

}

void High_Margin_Data(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x25;                     //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x0F;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xFC;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);

}

void Low_Margin_Data(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x26;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x00;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x00;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);
}

void Alarm_Pulse_Config(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xD2;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x18;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x00;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);

}

void Trigger_Pulse_Config(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xD3;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x04;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x08;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);

}

void Power_Up_DAC(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xD1;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x0A;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x80;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);
}

void High_Priority_Alarm_Setting(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xD2;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x20;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x00;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);
}

void High_Alarm(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
            return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1TXBUF=0xD2;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1TXBUF=0x20;
    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1TXBUF=0x0F;
    if(!I2C1_Transmit_Timeout())
            return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);

    unsigned long Timeout_Count=1000;
    int frq_domain[] = {262, 440, 349, 440, 349, 262, 440, 349, 440, 349};
    while(!(P2IN&(1<<7)))
    {
        P1OUT^=BIT3;
        _delay_cycles(100);
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            Alarm_IC_Reset();
            break;
        }
    }
    if(Timeout_Count==0)
    {
        return;
    }

    P1OUT&=~BIT3;
    int loop=0;
    for(loop=0;loop<10;loop++)
    {
        Timeout_Count=5000000;
        while(!(P2IN&(1<<7)))                  // Wait until pulse envelope starts
        {
            P1OUT^=BIT3;
            Timeout_Count-=1;
            if(Timeout_Count==0)
            {
                I2C1_Reset();
                Alarm_IC_Reset();
                break;
            }
        }
        if(Timeout_Count==0)
        {
            break;
        }
        Frequency(frq_domain[loop]*Gain);
        Timeout_Count=5000000;
        while(P2IN&(1<<7))                     // Wait until pulse envelope starts
        {
            Timeout_Count-=1;
            if(Timeout_Count==0)
            {
                I2C1_Reset();
                Alarm_IC_Reset();
                break;
            }
        }
        if(Timeout_Count==0)
        {
            break;
        }

    }
    Frequency(0);

}

void Medium_Alarm(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xD2;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x28;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x0F;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);

    //P1OUT|=BIT3;

    unsigned long Timeout_Count=10000;
    while(!(P2IN&(1<<7)))
    {
        P1OUT^=BIT3;
        _delay_cycles(100);
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            Alarm_IC_Reset();
            break;
        }
    }
    if(Timeout_Count==0)
    {
        return;
    }
    P1OUT&=~BIT3;

    int frq_domain1[] = {262, 440, 349};
    int loop=0;
    for(loop=0;loop<3;loop++)
    {
        Timeout_Count=5000000;
        while(!(P2IN&(1<<7)));                  // Wait until pulse envelope starts
        {
            P1OUT^=BIT3;
            Timeout_Count-=1;
            if(Timeout_Count==0)
            {
                I2C1_Reset();
                Alarm_IC_Reset();
                break;
            }
        }
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            Alarm_IC_Reset();
            break;
        }
        Timeout_Count=5000000;
        Frequency(frq_domain1[loop]*Gain);
        while(P2IN&(1<<7))                     // Wait until pulse envelope starts
        {
            Timeout_Count-=1;
            if(Timeout_Count==0)
            {
                I2C1_Reset();
                Alarm_IC_Reset();
                break;
            }
        }
    }
    Frequency(0);

}

void Low_Alarm(void)
{
    UCB1CTL1|=UCTR;                     //Turning on I2C in Transmit Mode

    UCB1I2CSA=Alarm_IC_Address;                   //Slave Address

    if(!I2C1_Bus_Status_Timeout())
        return ;

    UCB1CTL1|=UCTXSTT;                  //Start Condition

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0xD2;          //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register

    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x30;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1TXBUF=0x0F;
    if(!I2C1_Transmit_Timeout())
        return ;

    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus

    _delay_cycles(100);

    //P1OUT|=BIT3;
    unsigned long Timeout_Count=1000;
    while(!(P2IN&(1<<7)))
    {
        P1OUT^=BIT3;
        _delay_cycles(100);
        Timeout_Count-=1;
        if(Timeout_Count==0)
        {
            I2C1_Reset();
            Alarm_IC_Reset();
            break;
        }
    }
    if(Timeout_Count==0)
    {
        return;
    }
    P1OUT&=~BIT3;


    int frq_domain[] = {330,262};
    int loop=0;
    for(loop=0; loop<2;loop++)
    {
        Timeout_Count=5000000;
        while(!(P2IN&(1<<7)))                  // Wait until pulse envelope starts
        {
            P1OUT^=BIT3;
            Timeout_Count-=1;
            if(Timeout_Count==0)
            {
                I2C1_Reset();
                Alarm_IC_Reset();
                break;
            }
        }
        if(Timeout_Count==0)
        {
            break;
        }
        Frequency(frq_domain[loop]*Gain);
        Timeout_Count=5000000;
        while(P2IN&(1<<7))                     // Wait until pulse envelope starts
        {
            Timeout_Count-=1;
            if(Timeout_Count==0)
            {
                I2C1_Reset();
                Alarm_IC_Reset();
                break;
            }
        }
        if(Timeout_Count==0)
        {
            break;
        }
    }
    Frequency(0);

}

void Alarm_IC_Config(void)
{
    Power_Up_DAC();
    High_Margin_Data();
    Low_Margin_Data();
    Trigger_Pulse_Config();
}

/////////////////////////ALARM IC FUNCTIONS////////////////////////////

