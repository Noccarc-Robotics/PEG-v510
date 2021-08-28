//////////////////////Includes/////////////////
#include <msp430.h> 
#include "msp430f5529.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/////////////////////////////////////////////ADC Start////////////////////////////////////////////////////////////////////

/////////////////////////////Varaibles For ADC Start//////////////////

volatile unsigned int Volt_24_Sense_Raw_ADC;        //Analog Input Pin for 24 Voltage Sense is Pin 6.0
volatile unsigned int Volt_24_Current_Sense_Raw_ADC;     //Analog Input Pin for 24 Volt Current Sense is Pin 6.1
volatile unsigned int Volt_12_Sense_Raw_ADC;        //Analog Input Pin for 12 Voltage Sense is Pin 6.2
volatile unsigned int Volt_12_Current_Sense_Raw_ADC;     //Analog Input Pin for 12 Volt Current Sense is Pin 6.3
volatile unsigned int Temperature_Sensor_1_Raw_ADC; //Analog Input Pin for Temperature Sensor 1 is Pin 6.4
volatile unsigned int Temperature_Sensor_2_Raw_ADC; //Analog Input Pin For Temperature Sensor 2 is Pin 6.5
volatile unsigned int Volt_5_Sense_Raw_ADC;         //Analog Input Pin for 5 Voltage Sense is Pin is Pin 6.6
volatile unsigned int Volt_5_Current_Sense_Raw_ADC;      //Analog Input Pin for 5 Volt Current Sense is Pin 7.0
float Volt_5_Current=0;
float Volt_12_Current=0;
float Volt_24_Current=0;

void ADC_Init(void);            //Function Declaration for ADC Initialization
void ADC_Data_Read(void);       //Function for Reading ADC Value
void ADC_Data_Send(void);

//////////////////////////////////////////////ADC End//////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////COULOMB COUNTER/////////////////////////////

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
int ACK=0;
int Temporary_Hold=0;
unsigned int Read_Temp;

float Voltage;
unsigned int Voltage_MSB;
unsigned int Voltage_LSB;
int Temporary_Voltage;
float Result_Voltage;

float Current;
unsigned int Current_MSB;
unsigned int Current_LSB;
int Temporary_Current;
float Result_Current;

float Charge;
unsigned int Charge_MSB;
unsigned int Charge_LSB;
int Temporary_Charge;
float Result_Charge;


//////////////////Coulomb Counter Functions//////////////
_Bool Connected_I2C0_Device(void);
_Bool I2C0_Bus_Status_Timeout(void);
_Bool I2C0_Transmit_Timeout(void);
_Bool I2C0_Receive_Timeout(void);
_Bool I2C0_Acknowledge_Timeout(void);
void I2C0_Init(void);
void I2C0_Reset(void);
float Read_Coulomb_Charge(void);
float Read_Coulomb_Voltage(void);
float Read_Coulomb_Current(void);
void Coulomb_Counter_Setting(void);
void Coulomb_Data_Read(void);

//////////////////////////////////////////////COULOMB COUNTER/////////////////////////////


////////////////////////////////ALARM/////////////////////////


#define Alarm_IC_Address 0x47   //Address Number 1 of DAC IC

const int Gain=2;
int volume = 10;

void High_Margin_Data(void);                //Setting For High Margin Data
void Low_Margin_Data(void);                 //Setting For Low Margin Data
void Alarm_Pulse_Config(void);              //Setting for Alarm Pulse Config
void Trigger_Pulse_Config(void);            //S
void Power_Up_DAC(void);
void High_Priority_Alarm_Setting(void);
void High_Alarm(void);
void Medium_Alarm(void);
void Low_Alarm(void);
void Read_Bytes(int Address);
void Update_Volume(int);
void Alarm_IC_Reset(void);
void Alarm_IC_Config(void);
int Byte_1;
int Byte_2;
char res[20];

void Timer_A0_Init(void);
void Frequency(int Frequency);
volatile int Frequency_K=0;
void I2C1_Init(void);
void I2C1_Reset(void);
_Bool Connected_I2C1_Device(void);
_Bool I2C1_Bus_Status_Timeout(void);
_Bool I2C1_Transmit_Timeout(void);
_Bool I2C1_Receive_Timeout(void);
_Bool I2C1_Acknowledge_Timeout(void);


////////////////////////////ALARM////////////////////////////

//////////////////////////D FLIP FLOP BUZZER/////////////////
void Soft_Switch_Sequence(void);

void Normal_On_Sequence(void);

void Normal_off_Sequence(void);

void Turn_on_5V(void);

void Turn_off_5V(void);
int Soft_Switch_Lock_Count=10;
int Soft_Switch_Lock=0;
int i0=0;
int Once=0;
_Bool OFF_Condition=0;
_Bool Soft_Switch_Input=0;
int Switch_Press=0;
//////////////////////////D FLIP FLOP BUZZER/////////////////

//////////////////////Communication///////////////////////////
int Toggle=1;
char Data_TX='B';
char Data_TX_1[10]="Data 1\n";
char Data_TX_2[10]="Data 2\n";
char TX_Buffer[40]="Transmit and Receive both working\n";
char RX_Buffer[100];
char Buffer_Clear[100]={'0'};
//////////////////////////Transmit Packets////////////////////////////////////////////

char TX_Packet[11];
char Clear_String[11]={0};
char Start_of_Frame[1]={0x23};              //#
char End_of_Frame[1]={0x40};                //@
char Data_Mode[4]={0x47,0xF5,0xBB,0xBB};     //

char UART_RX_Buffer[100];   //RX Buffer For Receiving DMA Data
void RX_Break(void);
void Alarm_Check(void);
char RX_Mode[3];
char RX_Parameter_ID[2];
char RX_Payload[4];

int High_Priority_Alarm_Set_Flag=0;
int Medium_Priority_Alarm_Set_Flag=0;
int Low_Priority_Alarm_Set_Flag=0;
int Buzzer_Set_Flag=0;

void Flag_Check(void);
void Fuel_Current_Packet(void);
void Fuel_Voltage_Packet(void);
void Fuel_Temperature_Packet(void);
void Fuel_SOC_Packet(void);

void Low_Battery_Packet(void);
void Full_Battery_Packet(void);
void UPS_Rail_Packet(void);
void Rail_12V_Packet(void);
void Rail_5V_Packet(void);
void UPS_Current_Packet(void);
void Current_12V_Packet(void);
void Current_5V_Packet(void);
void Temperature_NTC1_Packet(void);
void Temperature_NTC2_Packet(void);
void Temperature_MCU_Packet(void);
void Temperature_Coulomb_Counter_Packet(void);
void Packet_To_Mother_Board(void);

//////////////////////////Transmit Packets////////////////////////////////////////////
//////////////////////Communication///////////////////////////



///////////////////////////Private System Function Decalaration Start/////////////////////
void System_Clock_Init(void);           //Function for Initializing System Clock
void Clock_Check(void);                 //Function for Checking Clock ACLK MCLK and SMCLK
void SetVCoreUp (unsigned int level);   //Function for Setting the Core Voltage level for increased Clock Speed
void GPIO_Init(void);                   //Function for Initializing GPIO

void UART_Init(void);
void UART_TX_String(char *String);
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);
void Newline(void);
volatile unsigned int MS_Delay;
void Delay(int Milli_Second);
void Timer_A1_Init(void);
int i,j,k;
char ASCIIstring[5];
void UART_DMA_TX_Init(void);
void UART_DMA_RX_Init(void);
void ConvertInt16toACSIIstring (unsigned int Value)
{
    unsigned char c = 5;

    memcpy(ASCIIstring, "    0", 5);

    while (c > 0 && Value != 0)
    {
        ASCIIstring[--c] = Value % 10 + '0';
        Value /= 10;
    }
};
//////////////////////////Private System Function Decalaration End/////////////////////


///////////////////////////////////////////////////////////////////////
/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    SetVCoreUp(3);      //Core Voltage Level 3
    System_Clock_Init();//System Clock 20MHz
    Clock_Check();      //For Clock Check
    GPIO_Init();        //For GPIO init
    Timer_A1_Init();    //Timer used for Delay Generation

    UART_Init();
    UART_DMA_RX_Init();
    //UART_DMA_TX_Init();

    ///////////////////Alarm IC Init////////////////////////

    I2C1_Init();         //I2C1 used for Interfacing DAC IC
    Timer_A0_Init();    //Timer used for Variable Frequency Generation
    Alarm_IC_Reset();
    Delay(100);
    Alarm_IC_Config();
    Delay(100);
    Power_Up_DAC();
    High_Margin_Data();
    Low_Margin_Data();
    Trigger_Pulse_Config();
    Delay(100);

    ///////////////////Alarm IC Init////////////////////////

    //////////////////Coulomb Counter Init//////////////////

    I2C0_Init();
    I2C0_Reset();

    /////////////////Coulomb Counter Init///////////////////

    ////////////////ADC INIT/////////////
    ADC_Init();
    ///////////////ADC INIT/////////////

    ///////////////D FLIP FLOP/////////
    Turn_off_5V();
    Normal_off_Sequence();
    //////////////D FLIP FLOP//////////

    while(1)
    {
        RX_Break();
        Coulomb_Data_Read();
        ADC_Data_Read();
        Flag_Check();
        Alarm_Check();
        Soft_Switch_Sequence();
        P4OUT^=BIT7; ////LED FOR Common Indication
        Delay(100);
    }


}

///////////////////////////COMMON FUNCTIONS///////////////////////
void SetVCoreUp (unsigned int level)
{
    PMMCTL0_H= 0xA5;
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    PMMCTL0_L = PMMCOREV0 * level;
    if ((PMMIFG & SVMLIFG))
    while ((PMMIFG & SVMLVLRIFG) == 0);
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    PMMCTL0_H = 0x00;
}

void Clock_Check(void)
{
    //P7SEL|=BIT7;
    //P7DIR|=BIT7;
    //P2SEL|=BIT2;
    //P2DIR|=BIT2;
    P4DIR|=BIT7;
}

void System_Clock_Init(void)
{
    UCSCTL1|=DCORSEL_5; //For Exact 20MHz Clock

    UCSCTL2=0x025D;

    UCSCTL4=0x0333;     //ACLK SMCLK MCLK Clock Select to DCO

    for(i=10000;i>0;i--);
}

void Delay(int Milli_Seconds)
{
    MS_Delay=0;
    TA1CTL=(TA1CTL&~MC_3)|MC_1;     //Start Up Counting to CCR Value
    TA1CCTL0 |= CCIE;               //Enable Interrupt of Timer A1

    while(1)
    {
        if(MS_Delay >= Milli_Seconds)     //Comparing whether the Delay Count is Reached or not
        {
            TA1CTL = (TA1CTL&~MC_3)|MC_0; //Stop Timer Counting
            MS_Delay = 0;                 //Variable again Set to zero for futher Use
            break;                        //Breaking the While loop
        }
    }
}

void Timer_A1_Init(void)
{
    __enable_interrupt();
    TA1CTL=TASSEL_2+ID_3+MC_0;      //Selecting MCLK as the Clock Source for Timer A1 divide by 8 and Stop Timer
    //TA1EX0=TAIDEX_7;              //Testing for Extended timer Clock Division
    TA1CCR0=2500;                   //Value Required to count 1 milli second of time
    TA1CCTL0 |= CCIE;               //Enable Timer A1 Interrupt
}
#pragma vector=TIMER1_A0_VECTOR

__interrupt void timer1_A0_ISR(void)
{
    MS_Delay++;         //Increment Tick
    TA1CCTL0 &= ~CCIFG; //Clear Interrupt
}

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
    for(i=0;i<strlen(String);i++)
    {
        while(!(UCA0IFG&UCTXIFG));
        UCA0TXBUF=String[i];
        while(UCA0CTL1 & UCBUSY);
    }
}

void reverse(char *str, int len)
{
    int u=0, p=len-1, temp;
    while (u<p)
    {
        temp = str[u];
        str[u] = str[p];
        str[p] = temp;
        u++; p--;
    }
}

int intToStr(int x, char str[], int d)
{
    int r = 0;
    while (x)
    {
        str[r++] = (x%10) + '0';
        x = x/10;
    }

    while (r < d)
        str[r++] = '0';

    reverse(str, r);
    str[r] = ' ';
    return r;
}

void ftoa(float n, char *res, int afterpoint)
{
    int ipart = (int)n;

    float fpart = n - (float)ipart;

    int q = intToStr(ipart, res, 0);

    if (afterpoint != 0)
    {
        res[q] = '.';

        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + q + 1, afterpoint);
    }
}

void Newline(void)
{
    while(!(UCA0IFG&UCTXIFG));
    UCA0TXBUF='\n';
    while(UCA0CTL1 & UCBUSY);
}

void UART_DMA_TX_Init(void)
{
    //DMA0 Init for UART TX with Interrupt Enabled
    DMA0CTL&=~DMAEN;                        //DMA Channel 0 Disabled
    DMACTL0|=DMA0TSEL_17;                   //DMA Channel 0 Trigger Selected as UART 0 Transmit Complete Flag
    DMA0SA=&TX_Buffer;                      //Selecting Source Address as Transmit Buffer Address
    DMA0DA=&UCA0TXBUF;                      //Selecting Destination Address as Uart 0 Tx Buffer
    DMA0SZ=sizeof(TX_Buffer);               //Size of Block Defined by Size of Function
    DMA0CTL= DMADT_4 + DMASRCINCR_3 + DMADSTBYTE +DMASRCBYTE+DMALEVEL+DMAIE;  //Repeated Block Transfer,Source Address Increment,Destination Byte, Source Byte,DMA level Trig  DMA interrupt Enable
    DMA0CTL|=DMAEN;                         //DMA Channel 0 Enabled
}

void UART_DMA_RX_Init(void)
{
    //DMA1 Init for UART RX with Interrupt Enabled
    DMA1CTL&=~DMAEN;                        //DMA Channel 1 Disabled
    DMACTL0|=DMA1TSEL_16;                   //DMA Channel 1 Trigger Selected as UART 0 Receive Complete Flag
    DMA1SA=&UCA0RXBUF;                      //Selecting Source Address as Receive transmit buffer
    DMA1DA=&UART_RX_Buffer;                      //Selection Destination Address as Block of Receive buffer
    DMA1SZ=sizeof(UART_RX_Buffer);               //Size of Block
    DMA1CTL = DMADT_5 + DMADSTINCR_3 + DMADSTBYTE + DMASRCBYTE + DMALEVEL+DMAIE;  //Repeat Block transfer,Destination increment,Destination Byte, Source Byte,DMA level Trig DMA Interrupt Enable
    DMA1CTL|=DMAEN;
    ////////////////////DMA Init for Reception End//////////////////////////////////
}

#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
{
    switch(DMAIV)
    {
        case DMAIV_DMA0IFG:
            DMA0CTL&=~DMAEN;     //DMA Channel 0 Disabled for UART TX
            Toggle^=1;
            if(Toggle==1)
            {
                memcpy(TX_Buffer, Buffer_Clear, sizeof(Data_TX_1)); //Clear Buffer
                memcpy(TX_Buffer,Data_TX_1,sizeof(Data_TX_1));      //Copy New Data DMA TX Buffer
            }
            if(Toggle==0)
            {
                memcpy(TX_Buffer, Buffer_Clear, sizeof(Data_TX_2)); //Clear Buffer
                memcpy(TX_Buffer,Data_TX_2,sizeof(Data_TX_2));      //Copy New Data to DMA TX Buffer
            }
            DMA0CTL&=~DMAIFG;
            DMA0CTL|=DMAEN;     //DMA Channel 0 Enable for UART TX
            break;

        case DMAIV_DMA1IFG:
            DMA1CTL&=~DMAEN;    //DMA Channel 1 Disabled for UART RX
            UCA0CTL1|=UCSWRST;  //Resetting USCI for Configuring Uart Module as Indicated in User Guide at page 940
            memcpy(RX_Buffer, UART_RX_Buffer, sizeof(RX_Buffer));  //Copying Data from UART RX buffer to Internal RX Buffer
            UCA0CTL1&=~UCSWRST;  //Resetting USCI for Configuring Uart Module as Indicated in User Guide at page 940
            DMA1CTL&=~DMAIFG;
            DMA1CTL|=DMAEN;     //DMA Channel 1 Enabled for UART RX
            break;
        default:
            break;
    }

}


///////////////////////////COMMON FUNCTIONS///////////////////////

//////////////////////////DFLIP FLOP /////////////////////////////

void Soft_Switch_Sequence(void)
{
    if(Switch_Press==1)
    {
        Switch_Press=0;
        if(Once==1)
        {
            Once=0;
            Turn_off_5V();
            OFF_Condition=1;
        }
        if(Once==0&&OFF_Condition==0)         //The Once variable is used for determining whether the soft switch was pressed or not earlier.
        {
            Turn_on_5V();   //Turn on 5V so that Main Board and HMI Board Becomes Operational
            Once=1;
            Normal_On_Sequence();   //This Function is Called for the IO and D Flip Flop so that Buzzer is not switched ON.
        }

        if(OFF_Condition==1)
        {
            OFF_Condition=0;
            Normal_off_Sequence();
        }
    }
    if(Soft_Switch_Lock==1)
    {
        Soft_Switch_Lock_Count-=1;
        if(Soft_Switch_Lock_Count==0)
        {
            Soft_Switch_Lock_Count=10;
            Soft_Switch_Lock=0;
            P2IFG&=~BIT0;
            P2IE|=BIT0;    //Interrupt Disable
        }
    }

}

void Turn_off_5V(void)
{
    //Set GPIO Pin Low which is connected to PMOS Gate
    P3OUT&=~BIT5;   //P3.5 Pin Low Turning of the 5V Line.
}

void Turn_on_5V(void)
{
    //Set GPIO Pin high which is connected to PMOS Gate
    P3OUT|=BIT5;    //P3.5 Pin High Turning on The 5V line
}

void Normal_On_Sequence(void)
{

    P8OUT&=~BIT2;    //D pin High
    P8OUT&=~BIT1;   //Clock low
    Delay(100);
    P8OUT|=BIT1;    //Clock High for latching Data.
    Delay(100);
    P8OUT&=~BIT1;   //Clock low
    Delay(100);
    P2OUT|=BIT3;    //Io Pin High

}

void Normal_off_Sequence(void)
{

    P8OUT|=BIT2;    //D pin High
    P8OUT&=~BIT1;   //Clock low
    Delay(100);
    P8OUT|=BIT1;    //Clock High for latching Data.
    Delay(100);
    P8OUT&=~BIT1;   //Clock low
    Delay(100);
    P2OUT|=BIT3;    //Io Pin High

}

#pragma vector=PORT2_VECTOR
__interrupt void PORT_2_ISR(void)
{
    switch(P2IFG)
    {

    default:
        Switch_Press=1;
        P2IFG&=~BIT0;   //Interrupt on P2.0 Clear
        P2IE&=~BIT0;    //Interrupt Disable
        Soft_Switch_Lock=1;
        break;
    }
    //P4OUT^=BIT7;
    //P2IFG&=~BIT0;   //Interrupt on P2.0 Clear
}

//////////////////////////DFLIP FLOP//////////////////////////////

/////////////////////////ALARM IC FUNCTIONS////////////////////////////

void GPIO_Init(void)
{
    //P1DIR|=BIT2;        //P1.2 Pin Set as Output Timer A Channel 1 TA0.1

    //P1SEL|=BIT2;        //P1.2 Pin Set as Alternate Function Channel 1

    //P1OUT|=BIT2;        //P1.2 Pin Set as Alternate Function Channel 1


    P6SEL|=BIT0;    //Pin 6.0 Alternate Function Select ADC Channel A0 For 24 Volt Voltage Sense

    P6SEL|=BIT1;    //Pin 6.1 Alternate Function Select ADC Channel A1 For 24 Volt Current Sense

    P6SEL|=BIT2;    //Pin 6.2 Alternate Function Select ADC Channel A2 For 12 Volt Voltage Sense

    P6SEL|=BIT3;    //Pin 6.3 Alternate Function Select ADC Channel A3 For 12 Volt Current Sense

    P6SEL|=BIT4;    //Pin 6.4 Alternate Function Select ADC Channel A4 For Temperature Sensor 1

    P6SEL|=BIT5;    //Pin 6.5 Alternate Function Select ADC Channel A5 For Temperature Sensor 2

    P6SEL|=BIT6;    //Pin 6.6 Alternate Function Select ADC Channel A6 For 5 Volt Voltage Sense

    P7SEL|=BIT0;    //Pin 7.0 Alternate Function Select ADC Channel A12 For 5 Volt Current Sense

    P1DIR|=BIT3;        //Pin 1.3 Set as GPIO Output for Alarm DAC

    P4DIR|=BIT7;

    P2DIR&=~BIT7;        //DAC input

    P2DIR|=BIT3;        //IO pin For Flip Flop Set as Output

    P8DIR|=BIT1;        //CK pin for Flip Flop

    P8DIR|=BIT2;        //D Pin for D Flip Flop

    P2DIR&=~BIT0;       //Pin 2.0 Set as Input for Soft Switch

    P2REN|=BIT0;        //Pin 2.0 Pullup pull Down Resistor Enable

    P2OUT&=~BIT0;        //Pin 2.0 Pullup

    P2IES|=BIT0;        //Interrupt Edge Select

    P2IFG&=~BIT0;       //Interrupt on P2.0 Clear

    P2IE|=BIT0;         //P2.0 Interrupt Enable

    P3DIR|=BIT5;        //P3.5 Set as Output for 5V line

    for(i=1000;i>0;i--);

}

void Read_Bytes(int Address)
{
    //////////////For Reading 2 Bytes
    //Master in Transmitting Mode
    UCB1CTL1|=UCTR;                     //Turning on I2C1 in Transmit Mode
    UCB1I2CSA=Alarm_IC_Address;  //Slave Address
    while(UCB1STAT&UCBBUSY);            //Checking if the Bus is Busy or Not
    UCB1CTL1|=UCTXSTT;                  //Start Condition is Sent
    while(!(UCB1IFG&UCTXIFG));          //The Flag UCTXIFG is set when Start Condition from previous Statement is Generated so that the Address can be Written.
    UCB1TXBUF=Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    while(UCB1CTL1&UCTXSTT);            //Waiting for the Start Flag to be cleared,which is cleared when slave Acknowledges the Address
    Read_Temp=UCB1RXBUF;
    //Master In Receiving Mode
    UCB1CTL1&=~UCTR;                    //Turning on I2C1 in Receive Mode
    UCB1CTL1|=UCTXSTT;                  //Sending Start Condition         //Waiting for the I2C1 to Generate Interrupt Flag that Receive is Complete
    while(!(UCB1IFG&UCRXIFG));
    Byte_1=UCB1RXBUF;                 //Transfering Data From RX buffer to Low Byte
    //while(!(UCB1IFG&UCRXIFG));
    //Byte_2=UCB1RXBUF;                 //Transfering Data From RX buffer to Low Byte
    UCB1CTL1|=UCTXSTP;                  //Stopping the I2C1 Bus
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

void I2C1_Reset(void)
{
    //4.1 SDA 4.2 SCL
    UCB1CTL1|=(UCSWRST);        //Setting the Software Reset Bit to High for Configuring I2C
    UCB1CTL1&=~UCSWRST;
    I2C1_Init();
    Alarm_IC_Reset();
}

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
    for(i=1000;i>0;i--);
    UCB1CTL1&=~(UCSWRST);    //Setting the Software Reset Bit to low for Enabling I2C1 Module
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
    for(i=10;i>0;i--);
}

void Alarm_IC_Config(void)
{
    Power_Up_DAC();
    High_Margin_Data();
    Low_Margin_Data();
    Trigger_Pulse_Config();
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
    for(i=10;i>0;i--);

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
    for(i=10;i>0;i--);

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
    for(i=10;i>0;i--);
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
    for(i=10;i>0;i--);

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
    for(i=10;i>0;i--);

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
    for(i=10;i>0;i--);
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
    for(i=10;i>0;i--);
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
    for(i=10;i>0;i--);

    unsigned long Timeout_Count=1000;
    int frq_domain[] = {262, 440, 349, 440, 349, 262, 440, 349, 440, 349};
    while(!(P2IN&(1<<7)))
    {
        P1OUT^=BIT3;
        Delay(1);
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

    for(i=0;i<10;i++)
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
        Frequency(frq_domain[i]*Gain);
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
//    if(Timeout_Count==0)
//    {
//        return;
//    }
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
    for(i=10;i>0;i--);

    //P1OUT|=BIT3;

    unsigned long Timeout_Count=10000;
    while(!(P2IN&(1<<7)))
    {
        P1OUT^=BIT3;
        Delay(1);
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

    for(i=0;i<3;i++)
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
        Frequency(frq_domain1[i]*Gain);
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
    for(i=10;i>0;i--);

    //P1OUT|=BIT3;
    unsigned long Timeout_Count=1000;
    while(!(P2IN&(1<<7)))
    {
        P1OUT^=BIT3;
        Delay(1);
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
    for(i=0; i<2;i++)
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
        Frequency(frq_domain[i]*Gain);
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

/////////////////////////ALARM IC FUNCTIONS////////////////////////////

///////////////////////////Coulomb Counter Function Starts here///////////////////////

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

    UCB0CTL1|=UCTR;                     //Turning on I2C0 in Transmit Mode
    UCB0I2CSA=Coulomb_Counter_Address;  //Slave Address
    while(UCB0STAT&UCBBUSY);            //Checking if the Bus is Busy or Not
    UCB0CTL1|=UCTXSTT;                  //Start Condition is Sent
    while(!(UCB0IFG&UCTXIFG));          //The Flag UCTXIFG is set when Start Condition from previous Statement is Generated so that the Address can be Written.
    UCB0TXBUF=Coulomb_Voltage_MSB_Address;       //Writing the Address which is to be Read from Slave in Reading Mode in Transmit Buffer Register
    while(UCB0CTL1&UCTXSTT);            //Waiting for the Start Flag to be cleared,which is cleared when slave Acknowledges the Address
    Read_Temp=UCB0RXBUF;

    UCB0CTL1|=UCTXSTP;                  //Stopping the I2C0 Bus
    if(UCB0IFG&UCNACKIFG)
    {
        UCB0IFG&=~UCNACKIFG;
        return 0;
    }
    return 1;
}

void I2C0_Reset(void)
{
    UCB0CTL1|=(UCSWRST);        //Setting the Software Reset Bit to High for Configuring I2C0
    UCB0CTL1&=~(UCSWRST);
    I2C0_Init();
}

void I2C0_Init(void)
{

    P3SEL|=BIT0;                //Pin 3.0 Alternate Function Select UCB0SDA

    P3SEL|=BIT1;                //Pin 3.1 Alternate Function Select UCB0SCL

    UCB0CTL1|=(UCSWRST);        //Setting the Software Reset Bit to High for Configuring I2C0

    UCB0CTL0=0b00001111;        //For Setting own,slave Address bit, Master slave mode I2C0 or SPI mode synchronous or asynchronous mode

    UCB0CTL1=0b10010000;        //Setting I2C0 Input Clock to SMCLK and Transmitter Select

    UCB0BR0=50;                 //Baud Rate Setting Register Low Byte for 100Kbps

    UCB0BR1=0;                  //Baud Rate Setting Register High Byte for 100Kbps

    for(i=1000;i>0;i--);         //Software Delay

    UCB0CTL1&=~(UCSWRST);       //Setting the Software Reset Bit to low for Enabling I2C0 Module

}

float Read_Coulomb_Voltage(void)
{
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
    //for(i=10;i>0;i--);
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
    //for(i=10;i>0;i--);

    /////////////////////////Byte 2 Reading End////////////////////////////
    Voltage_MSB=Voltage_MSB<<8;
    Temporary_Voltage=Voltage_MSB|Voltage_LSB;
    Result_Voltage=Temporary_Voltage*70.8;
    Result_Voltage=Result_Voltage/65535;
    return Result_Voltage;

}

float Read_Coulomb_Current(void)
{
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
    //for(i=10;i>0;i--);



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
    //for(i=10;i>0;i--);


    Current_MSB=Current_MSB<<8;
    Temporary_Current=Current_MSB|Current_LSB;
    Result_Current=(Temporary_Current-32767)*6.4;
    Result_Current=Result_Current/32767;

    return Result_Current;
}

float Read_Coulomb_Charge(void)
{
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
    //for(i=10;i>0;i--);



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
    //for(i=10;i>0;i--);


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
    for(i=10;i>0;i--);
}

void Coulomb_Data_Read(void)
{
    //////////Coulomb Counter Code Start //////////////////////
    if(!(Connected_I2C0_Device()))
    {
        I2C0_Reset();
    }
    Delay(100);
    if(Connected_I2C0_Device())
    {
        Coulomb_Counter_Setting();
        Voltage=Read_Coulomb_Voltage();
        //ftoa(Voltage, res, 2);
        Current=Read_Coulomb_Current();
        //ftoa(Current, res, 2);
        Charge=Read_Coulomb_Charge();
        //ftoa(Charge, res, 2);

    }
    //////////Coulomb Counter Code End //////////////////////
}

/////////////////////////////////////Coulomb Counter Functions Ends Here//////////////////////

/////////////////////////////ADC Start///////////////////////////////////////

void ADC_Init(void)
{
    //ADC12CTL0=0b0010 0010 1001 0010;
    //ADC12CTL1=0b0000 0010 0000 0010;  Working
    //ADC12CTL1=0b0000 0010 0001 1010;  Working
    //ADC12CTL1=0b0000 0010 0011 1010;  Working ADC Clock Selected as ACLK which is 32Khz and The Clock is Divided by 2 gives 5 ADC count error.
    //ADC12CTL1=0b0000 0010 1111 1010;  Working ADC Clock Selected as ACLK which is 32Khz and The Clock is Divided by 2 gives 5 ADC count error.

    ADC12CTL0&=~ADC12ENC;                   //Disabling Enable Pin For Configuring ADC Module note For Configuring ADC Control Register Enable Should be 0

    ADC12CTL0=0xFF90;                       //Setting ADC Conversion Clock Cycles Multiple Samples and ADC ON

    ADC12CTL1=0x02FA;                       //Selecting Pulse Mode and Multiple Conversion

    ADC12MCTL0=ADC12INCH_0;                 //Storing the Result of ADC Conversion in Memory Location 0 of ADC Channel 0 Pin 6.0

    ADC12MCTL1=ADC12INCH_1;                 //Storing the Result of ADC Conversion in Memory Location 1 of ADC Channel 1 Pin 6.1

    ADC12MCTL2=ADC12INCH_2;                 //Storing the Result of ADC Conversion in Memory Location 2 of ADC Channel 2 Pin 6.2

    ADC12MCTL3=ADC12INCH_3;                 //Storing the Result of ADC Conversion in Memory Location 3 of ADC Channel 3 Pin 6.3

    ADC12MCTL4=ADC12INCH_4;                 //Storing the Result of ADC Conversion in Memory Location 4 of ADC Channel 4 Pin 6.4

    ADC12MCTL5=ADC12INCH_5;                 //Storing the Result of ADC Conversion in Memory Location 5 of ADC Channel 5 Pin 6.5

    ADC12MCTL6=ADC12INCH_6;                 //Storing the Result of ADC Conversion in Memory Location 6 of ADC Channel 6 Pin 6.6

    ADC12MCTL7=ADC12EOS|ADC12INCH_12;       //Storing the Result of ADC Conversion in Memory Location 7 of ADC Channel 12 and Indicating Last Conversion

    //ADC12MCTL8|=ADC12INCH_12;
    //ADC12MCTL9|=ADC12INCH_13;
    //ADC12MCTL10|=ADC12INCH_14|ADC12EOS;

}

void ADC_Data_Read(void)
{
    ADC12CTL0|=ADC12SC|ADC12ENC;        //Enabling the ADC and Setting the Flag for Starting the Conversion

    while(ADC12CTL1&ADC12BUSY);         //Waiting for the ADC Core to Complete the Conversion and Store the Result in their Specified Memory Registers

    Volt_24_Sense_Raw_ADC=ADC12MEM0;    //Copying The Contents of Memory Location 0 to Variable

    Volt_24_Current_Sense_Raw_ADC=ADC12MEM1;                //Copying The Contents of Memory Location 1 to Variable

    Volt_12_Sense_Raw_ADC=ADC12MEM2;                //Copying The Contents of Memory Location 2 to Variable

    Volt_12_Current_Sense_Raw_ADC=ADC12MEM3;                //Copying The Contents of Memory Location 3 to Variable

    Temperature_Sensor_1_Raw_ADC=ADC12MEM4;                //Copying The Contents of Memory Location 4 to Variable

    Temperature_Sensor_2_Raw_ADC=ADC12MEM5;                //Copying The Contents of Memory Location 5 to Variable

    Volt_5_Sense_Raw_ADC=ADC12MEM6;                //Copying The Contents of Memory Location 6 to Variable

    Volt_5_Current_Sense_Raw_ADC=ADC12MEM7;                //Copying The Contents of Memory Location 7 to Varialble

    //ADC_Val_9=ADC12MEM8;

    //ADC_Val_10=ADC12MEM9;

    ADC12CTL0&=~ADC12SC;

    ADC12CTL0&=~ADC12ENC;

}

void ADC_Data_Send(void)
{
    Volt_5_Current=Volt_5_Current_Sense_Raw_ADC/20;
    Volt_5_Current=Volt_5_Current*0.0805;
    Volt_12_Current=Volt_12_Current_Sense_Raw_ADC/20;
    Volt_12_Current=Volt_12_Current*0.0805;
    Volt_24_Current=Volt_24_Current_Sense_Raw_ADC/20;
    Volt_24_Current=Volt_24_Current*0.0805;
    UART_TX_String("Volt_24_Sense_Raw_ADC: ");
    ftoa(Volt_24_Sense_Raw_ADC*0.0008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

    UART_TX_String("Volt_24_Current_Sense_Raw_ADC: ");
    ftoa(Volt_24_Current_Sense_Raw_ADC*0.0008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

    UART_TX_String("Volt_12_Sense_Raw_ADC: ");
    ftoa(Volt_12_Sense_Raw_ADC*0.0008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

    UART_TX_String("Volt_12_Current_Sense_Raw_ADC: ");
    ftoa(Volt_12_Current_Sense_Raw_ADC*0.0008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

    UART_TX_String("Volt_5_Sense_Raw_ADC: ");
    ftoa(Volt_5_Sense_Raw_ADC*0.0008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

    UART_TX_String("Volt_5_Current : ");
    ftoa((Volt_5_Current_Sense_Raw_ADC/20)*0.008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

    UART_TX_String("Temperature_Sensor_1_Raw_ADC: ");
    ftoa(Temperature_Sensor_1_Raw_ADC*0.0008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

    UART_TX_String("Temperature_Sensor_2_Raw_ADC: ");
    ftoa(Temperature_Sensor_2_Raw_ADC*0.0008, res, 2);
    UART_TX_String(res);
    Newline();
    Newline();

}

/////////////////////////////ADC End///////////////////////////////////////

///////////////////////////TRANSMIT PACKETS///////////////////////

void Fuel_Current_Packet(void)
{
    //char Send_Data[4]={0xF5,0xF5,0x12,0x13};
//    strcpy(TX_Packet, Clear_String);    //Clearing the String.
//    strcat(TX_Packet, Start_of_Frame);  //Adding Start of Frame
//    strcat(TX_Packet, Data_Mode);       //Mode
//    strcat(TX_Packet, (const char *)0x01);            //Current
//    strcat(TX_Packet,Send_Data);
//    strcat(TX_Packet, End_of_Frame);

    TX_Packet[0]=0x23;
    TX_Packet[1]=0xBB;
    TX_Packet[2]=0xCC;
    TX_Packet[3]=0xBB;
    TX_Packet[4]=0xF5;
    TX_Packet[5]=0x01;
    TX_Packet[6]=0xF5;
    TX_Packet[7]=0xF5;
    TX_Packet[8]=0x12;
    TX_Packet[9]=0x13;
    TX_Packet[10]=0x40;
    UART_TX_String(TX_Packet);
}

void Fuel_Voltage_Packet(void)
{

}

void Fuel_Temperature_Packet(void)
{

}

void Fuel_SOC_Packet(void)
{

}

void Low_Battery_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0x01;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x01;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Full_Battery_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x02;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void UPS_Rail_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x03;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Rail_12V_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x04;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Rail_5V_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x05;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void UPS_Current_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x06;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Current_12V_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x07;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Current_5V_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x08;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Temperature_NTC1_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x09;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Temperature_NTC2_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x0A;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Temperature_MCU_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x0B;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void Temperature_Coulomb_Counter_Packet(void)
{
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0xBB;      //Mode
    TX_Packet[2]=0xBB;      //Mode
    TX_Packet[3]=0xBB;      //Mode
    TX_Packet[4]=0xF5;      //Param ID
    TX_Packet[5]=0xA8;      //Param ID
    TX_Packet[6]=0xF5;      //Data
    TX_Packet[7]=0xF5;      //Data
    TX_Packet[8]=0xF5;      //Data
    TX_Packet[9]=0x0C;      //Data
    TX_Packet[10]=0x40;     //End of Frame.
    UART_TX_String(TX_Packet);
}

void RX_Break(void)
{

    int p=0;
    for(p=0;p<100;p++)
    {
        if(RX_Buffer[p]==0x23&&RX_Buffer[p+10]==0x40)
        {
            RX_Mode[0]=RX_Buffer[p+1];          //1st Byte
            RX_Mode[1]=RX_Buffer[p+2];          //2nd Byte
            RX_Mode[2]=RX_Buffer[p+3];          //3rd Byte
            RX_Parameter_ID[0]=RX_Buffer[p+4];  //4th Byte
            RX_Parameter_ID[1]=RX_Buffer[p+5];  //5th Byte
            RX_Payload[0]=RX_Buffer[p+6];       //6th Byte
            RX_Payload[1]=RX_Buffer[p+7];       //7th Byte
            RX_Payload[2]=RX_Buffer[p+8];       //8th Byte
            RX_Payload[3]=RX_Buffer[p+9];       //9th Byte
        }
    }
    memcpy(RX_Buffer, Buffer_Clear, sizeof(RX_Buffer));
}

void Flag_Check(void)
{
    if(RX_Payload[3]==0x01)
    {
        High_Priority_Alarm_Set_Flag|=1;
    }
    if(RX_Payload[3]==0x02)
    {
        Medium_Priority_Alarm_Set_Flag|=1;
    }
    if(RX_Payload[3]==0x04)
    {
        Low_Priority_Alarm_Set_Flag|=1;
    }
    if(RX_Payload[3]==0x08)
    {
        Buzzer_Set_Flag|=1;
    }
    RX_Payload[0]=0x00;
    RX_Payload[1]=0x00;
    RX_Payload[2]=0x00;
    RX_Payload[3]=0x00;


}

void Alarm_Check()
{
    if(High_Priority_Alarm_Set_Flag==1)
    {
        P4OUT|=BIT7;
        P1OUT|=BIT0;
        High_Priority_Alarm_Set_Flag=0;
        Update_Volume(volume);
        Delay(10);
        High_Alarm();
        Delay(1000);

        Alarm_IC_Reset();
        Delay(100);
        Alarm_IC_Config();
        Delay(100);
    }
    if(Medium_Priority_Alarm_Set_Flag==1)
    {
        P4OUT&=~BIT7;
        P1OUT|=BIT0;
        Medium_Priority_Alarm_Set_Flag=0;

        Update_Volume(volume);
        Delay(10);
        Medium_Alarm();
        Delay(1000);

        Alarm_IC_Reset();
        Delay(100);
        Alarm_IC_Config();
        Delay(100);
    }
    if(Low_Priority_Alarm_Set_Flag==1)
    {
        P4OUT|=BIT7;
        P1OUT&=~BIT0;
        Low_Priority_Alarm_Set_Flag=0;

        Update_Volume(volume);
        Delay(10);
        Low_Alarm();
        Delay(1000);

        Alarm_IC_Reset();
        Delay(100);
        Alarm_IC_Config();
        Delay(100);
    }
    if(Buzzer_Set_Flag==1)
    {
        P4OUT&=~BIT7;
        P1OUT&=~BIT0;
        P2OUT&=~BIT3;    //Io Pin High
        Delay(1000);
        P2OUT|=BIT3;
        Buzzer_Set_Flag=0;
    }
}

void Packet_To_Mother_Board(void)
{


}

///////////////////////////TRANSMIT PACKETS///////////////////////
