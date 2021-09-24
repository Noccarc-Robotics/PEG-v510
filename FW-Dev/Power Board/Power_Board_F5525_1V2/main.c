/*Includes BEGIN*/
#include <msp430.h> 
#include "msp430f5529.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "Sys_Config.h"
#include "Coulomb_I2C.h"
#include "Alarm_I2C1.h"
#include "Uart0.h"
/*Includes END*/

/*INPUT CAPTURE BEGIN*/
float Frequency_Send=0;     //This Variable is used for Storing the Final Value of Frequency of Microphone
long unsigned int Frequency_Input=0;    //Used for Average Calculation
unsigned int Counter_Input_Capture;
unsigned int Prev_Counter_Input_Capture;
unsigned int Difference_Input_Capture;
unsigned int Frequency_Array[100];
unsigned int Frequency_Array_Count=0;
void Timer_A2_Init(void);
void Timer_A2_DeInit(void);
void GPIO_Init_Input_Capture(void);
/*INPUT CAPTURE END*/

/*ADC BEGIN*/
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
float Read_Temperature_NTC_2(void);
/*ADC END*/

/*Coulomb Counter BEGIN*/
float Voltage;  //Used for Storing Voltage of Coulomb Counter
float Current;  //Used for Storing Current of Coulomb Counter
float Charge;   //Used for Storing Charge of Coulomb Counter
char res[20];
/*Coulomb Counter END*/

/*Alarm IC BEGIN*/
unsigned int volume=50; //Used for Changing Volume os Speaker
/*Alarm IC END*/

/*D Flip Flop BEGIN*/
void Turn_on_Fan(void);
void Turn_off_Fan(void);
void Soft_Switch_Sequence(void);
void Normal_On_Sequence(void);
void Normal_off_Sequence(void);
void Turn_on_5V(void);
void Turn_off_5V(void);
int Soft_Switch_Lock_Count=10;
int Soft_Switch_Lock=0;
int Once=0;
_Bool OFF_Condition=0;
int Switch_Press=0;
/*D Flip Flop END*/

/*Communication Variables and Functions BEGIN*/
char UART_RX_Buffer[100];   //RX Buffer For Receiving DMA Data
char UART_TX_Buffer[11];    //This Buffer is used by UART DMA
char Buffer_Clear[100]={0}; //Used for Clearing Buffer with size of 100
char UART_TX_Buffer_Clear[11]={0};  //This Buffer is used for Clearing UART TX Buffer
char RX_Buffer[100];        //This Buffer is used by UART DMA
char TX_Packet[11];         //Internal buffer for transmit.

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

char Low_Battery_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x01,0x40};
char Full_Battery_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x02,0x40};
char UPS_Rail_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x03,0x40};
char Rail_12V_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x04,0x40};
char Rail_5V_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x05,0x40};
char UPS_Current_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x06,0x40};
char Current_12V_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x07,0x40};
char Current_5V_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x08,0x40};
char Temperature_NTC1_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x09,0x40};
char Temperature_NTC2_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x0A,0x40};
char Temperature_MCU_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x0B,0x40};
char Temperature_Coulomb_Counter_Packet[11]={0x23,0xBB,0xBB,0xBB,0xF5,0xA8,0xF5,0xF5,0xF5,0x0C,0x40};

/*Communication Variables and Functions END*/

void GPIO_Init(void);                   //Function for Initializing GPIO
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);

int i,j,k;
char ASCIIstring[5];
void UART_DMA_TX_Init(void);
void UART_DMA_RX_Init(void);
void UART0_DMA_TX_Callback(void);
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


/*PREUSE BEGIN */
int x=0;    //Used only for Testing Purpose
char Preuse_Param_ID[2];
char Preuse_Payload[4];
int Preuse_Check_Flag=0;
char Preuse_TX_Buffer[8];
void Preuse_Check(void);
void Preuse_UART_TX(char data);
void Preuse_TX_Packet(char s[]);
void Preuse_TX_Pass_Packet(void);
void Preuse_TX_Fail_Packet(void);
void Preuse_TX_Echo_Packet(void);
void Preuse_TX_Echo_NULL(void);
void Preuse_Clear(void);
/*PREUSE END*/
int Test=0;
float Voltage_Divder_R1=10000; //Voltage Divider Resistance value.
float R1_NTC=10000;                //Resistance of NTC in ambient 25
float B_Value=3965;            //BValue given in datasheet
float T1_NTC=298.15;                //Ambient room temperature
float e_NTC=2.718281;
double R2_NTC=0;
double T2_NTC=0;
double a_NTC;
double b_NTC;
double c_NTC;
double d_NTC;
float myvar=0;
///////////////////////////////////////////////////////////////////////
/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    _delay_cycles(1000);
    System_Clock_Init();//System Clock 20MHz
    Clock_Check();      //For Clock Check
    GPIO_Init();        //For GPIO init
    Timer_A1_Init();    //Timer used for Delay Generation

    UART_Init();
    //UART_DMA_RX_Init();
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

    GPIO_Init_Input_Capture();

    while(1)
    {

        RX_Break();
        if(Preuse_Check_Flag==1)
        {
            Preuse_Check();    //Respond to Preuse Check
        }

        /*Coulomb Counter BEGIN*/
        if(Connected_I2C0_Device())
        {
            Coulomb_Counter_Setting();
            Voltage=Read_Coulomb_Voltage();
            Current=Read_Coulomb_Current();
            Charge=Read_Coulomb_Charge();
        }
        /*Coulomb Counter END*/

        ADC_Data_Read();
        Flag_Check();
        Alarm_Check();
        Soft_Switch_Sequence();
        P8OUT^=BIT2;
        High_Priority_Alarm_Set_Flag=1;
        Turn_on_Fan();
        Turn_on_5V();
        Delay(100);
        Turn_off_Fan();
        Turn_off_5V();
        Delay(100);
        myvar=((4095-Temperature_Sensor_2_Raw_ADC)*Voltage_Divder_R1)/Temperature_Sensor_2_Raw_ADC;
        //myvar=myvar*Voltage_Divder_R1;
        //myvar/=Temperature_Sensor_2_Raw_ADC;
        a_NTC = 1/T1_NTC;
        b_NTC = log10(10000/myvar);
        c_NTC = b_NTC / log10(e_NTC);
        d_NTC = c_NTC / B_Value ;
        T2_NTC = 1 / (a_NTC- d_NTC);
        T2_NTC= T2_NTC-273.15;
        Test+=1;

    }


}

/*Common Functions BEGIN*/
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

void UART_DMA_TX_Init(void)
{
    //DMA0 Init for UART TX with Interrupt Enabled
    DMA0CTL&=~DMAEN;                        //DMA Channel 0 Disabled
    DMACTL0|=DMA0TSEL_17;                   //DMA Channel 0 Trigger Selected as UART 0 Transmit Complete Flag
    DMA0SA=&UART_TX_Buffer;                      //Selecting Source Address as Transmit Buffer Address
    DMA0DA=&UCA0TXBUF;                      //Selecting Destination Address as Uart 0 Tx Buffer
    DMA0SZ=sizeof(UART_TX_Buffer);               //Size of Block Defined by Size of Function
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
            //UCA0CTL1|=UCSWRST;
            memcpy(UART_TX_Buffer, UART_TX_Buffer_Clear, sizeof(UART_TX_Buffer_Clear)); //Buffer Clear
            UART0_DMA_TX_Callback();
            memcpy(UART_TX_Buffer, TX_Packet, sizeof(TX_Packet)); //Buffer Fill
            //UCA0CTL1&=~UCSWRST;
            DMA0CTL&=~DMAIFG;
            DMA0CTL|=DMAEN;     //DMA Channel 0 Enable for UART TX
            break;

        case DMAIV_DMA1IFG:
            DMA1CTL&=~DMAEN;    //DMA Channel 1 Disabled for UART RX
            UCA0CTL1|=UCSWRST;  //Resetting USCI for Configuring Uart Module as Indicated in User Guide at page 940
            memcpy(RX_Buffer, UART_RX_Buffer, sizeof(RX_Buffer));  //Copying Data from UART RX buffer to Internal RX Buffer
            memcpy(UART_RX_Buffer,Buffer_Clear,sizeof(UART_RX_Buffer));//Clearing the Receive Buffer of UART
            UCA0CTL1&=~UCSWRST;  //Resetting USCI for Configuring Uart Module as Indicated in User Guide at page 940
            DMA1CTL&=~DMAIFG;
            DMA1CTL|=DMAEN;     //DMA Channel 1 Enabled for UART RX
            break;
        default:
            break;
    }

}

void UART0_DMA_TX_Callback(void)
{
    if(Test==0)
    {
        memcpy(TX_Packet, Low_Battery_Packet, sizeof(Low_Battery_Packet));
    }
    if(Test==1)
    {
        memcpy(TX_Packet, Full_Battery_Packet, sizeof(Full_Battery_Packet));
    }
    if(Test==2)
    {
        memcpy(TX_Packet, UPS_Rail_Packet, sizeof(UPS_Rail_Packet));
    }
    if(Test==3)
    {
        memcpy(TX_Packet, Rail_12V_Packet, sizeof(Rail_12V_Packet));
    }
    if(Test==4)
    {
        memcpy(TX_Packet, Rail_5V_Packet, sizeof(Rail_5V_Packet));
    }
    if(Test==5)
    {
        memcpy(TX_Packet, UPS_Current_Packet, sizeof(UPS_Current_Packet));
    }
    if(Test==6)
    {
        memcpy(TX_Packet, Current_12V_Packet, sizeof(Current_12V_Packet));
    }
    if(Test==7)
    {
        memcpy(TX_Packet, Current_5V_Packet, sizeof(Current_5V_Packet));
    }
    if(Test==8)
    {
        memcpy(TX_Packet, Temperature_NTC1_Packet, sizeof(Temperature_NTC1_Packet));
    }
    if(Test==9)
    {
        memcpy(TX_Packet, Temperature_NTC2_Packet, sizeof(Temperature_NTC2_Packet));
    }
    if(Test==10)
    {
        memcpy(TX_Packet, Temperature_MCU_Packet, sizeof(Temperature_MCU_Packet));
    }
    if(Test==11)
    {
        memcpy(TX_Packet, Temperature_Coulomb_Counter_Packet, sizeof(Temperature_Coulomb_Counter_Packet));
        Test=0;
    }
}

/*Common Functions END*/

/*D Flip Flop BEGIN*/
void Turn_on_Fan(void)
{
    P1OUT|=BIT0;
}

void Turn_off_Fan(void)
{
    P1OUT&=~BIT0;
}

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
            P2IFG&=~BIT1;
            P2IE|=BIT1;    //Interrupt Disable
        }
    }

}

void Turn_off_5V(void)
{
    //Set GPIO Pin Low which is connected to PMOS Gate
    P2OUT&=~BIT0;   //P2.0 Pin Low Turning off the 5V Line. (Power_Board_1V2)
}

void Turn_on_5V(void)
{
    //Set GPIO Pin high which is connected to PMOS Gate
    P2OUT|=BIT0;    //P2.0 Pin High Turning on The 5V line. (Power_Board_1V2).s
}

void Normal_On_Sequence(void)
{
    ///D=0 IO=1             Inverted Ouput D=1
    P1OUT&=~BIT6;    //D pin High
    P1OUT&=~BIT7;   //Clock low
    Delay(100);
    P1OUT|=BIT7;    //Clock High for latching Data.
    Delay(100);
    P1OUT&=~BIT7;   //Clock low
    Delay(100);
    P1OUT|=BIT5;    //Io Pin High
}

void Normal_off_Sequence(void)
{
    P1OUT|=BIT6;    //D pin High
    P1OUT&=~BIT7;   //Clock low
    Delay(100);
    P1OUT|=BIT7;    //Clock High for latching Data.
    Delay(100);
    P1OUT&=~BIT7;   //Clock low
    Delay(100);
    P1OUT|=BIT5;    //Io Pin High
}

#pragma vector=PORT2_VECTOR
__interrupt void PORT_2_ISR(void)
{
    switch(P2IFG)
    {
    default:
        Switch_Press=1;
        P2IFG&=~BIT1;   //Interrupt on P2.0 Clear. (Power_Board_1V2)
        P2IE&=~BIT1;    //Interrupt Disable. (Power_Board_1V2)
        Soft_Switch_Lock=1;
        break;
    }

}

/*D Flip Flop END*/

/*INIT BEGIN*/

void GPIO_Init(void)
{

    P6SEL|=BIT0;        //Pin 6.0 Alternate Function Select ADC Channel A0 For 24 Volt Voltage Sense

    P6SEL|=BIT1;        //Pin 6.1 Alternate Function Select ADC Channel A1 For 24 Volt Current Sense

    P6SEL|=BIT2;        //Pin 6.2 Alternate Function Select ADC Channel A2 For 12 Volt Voltage Sense

    //P6SEL|=BIT3;        //Pin 6.3 Alternate Function Select ADC Channel A3 For 12 Volt Current Sense

    P6SEL|=BIT3;        //Pin 6.3 Alternate Function Select ADC Channel A3 For NTC 2 Temperature sense

    P6SEL|=BIT4;        //Pin 6.4 Alternate Function Select ADC Channel A4 For Temperature Sensor 1

    P6SEL|=BIT5;        //Pin 6.5 Alternate Function Select ADC Channel A5 For Temperature Sensor 2

    P6SEL|=BIT6;        //Pin 6.6 Alternate Function Select ADC Channel A6 For 5 Volt Voltage Sense

    P7SEL|=BIT0;        //Pin 7.0 Alternate Function Select ADC Channel A12 For 5 Volt Current Sense

    P7DIR|=BIT7;        //P7.7 GPI Output for Alarm DAC

    P1DIR&=~BIT4;       //P1.4 DAC Input from comparator

    P1DIR|=BIT5;        //IO pin For Flip Flop Set as Output (Power_Board_1V2)

    P1DIR|=BIT7;        //CK pin for Flip Flop Set as Output (Power_Boad_1V2)

    P1DIR|=BIT6;        //D Pin for D Flip Flop Set as Output (Power_Board_1V2)

    P2DIR&=~BIT1;       //Pin 2.1 Set as Input for Soft Switch (Power_Board_1V2)

    P2REN|=BIT1;        //Pin 2.1 Pullup pull Down Resistor Enable (Power_Board_1V2)

    P2OUT&=~BIT1;       //Pin 2.1 Pulldown (Power_Board_1V2)

    P2IES|=BIT1;        //Interrupt Edge Select (Power_Board_1V2)

    P2IFG&=~BIT1;       //Interrupt on P2.1 Clear   (Power_Board_1V2)

    P2IE|=BIT1;         //P2.1 Interrupt Enable (Power_Board_1V2)

    P2DIR|=BIT0;        //P2.0 Set as Output for 5V line Control (Power_Board_1V2)

    P8DIR|=BIT2;        //P8.2 Set as Output for Status LED 3.

    P8DIR|=BIT1;        //P8.1 Set as Output for Status LED 2.

    P8DIR|=BIT0;        //P8.0 Set as Output for Status LED 1.

    P8OUT&=~BIT2;       //Set Low

    P8OUT&=~BIT1;       //Set Low

    P8OUT&=~BIT0;       //Set Low

    P1DIR|=BIT0;        //P1.0 Set as output for controlling  Fan

    _delay_cycles(100);

}

/*INIT END*/

/*ADC BEGIN*/


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

    //Temperature_Sensor_2_Raw_ADC=ADC12MEM5;                //Copying The Contents of Memory Location 5 to Variable

    Temperature_Sensor_2_Raw_ADC=ADC12MEM3;                //Copying The Contents of Memory Location 5 to Variable

    Volt_5_Sense_Raw_ADC=ADC12MEM6;                //Copying The Contents of Memory Location 6 to Variable

    Volt_5_Current_Sense_Raw_ADC=ADC12MEM7;                //Copying The Contents of Memory Location 7 to Varialble

    //ADC_Val_9=ADC12MEM8;

    //ADC_Val_10=ADC12MEM9;

    ADC12CTL0&=~ADC12SC;

    ADC12CTL0&=~ADC12ENC;

}

float Read_Temperature_NTC_2(void)
{

    return 0;
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
/*ADC END*/

/* Transmit Packet BEGIN*/
void Fuel_Current_Packet(void)
{
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

void RX_Break(void)
{
    DMA1CTL&=~DMAEN;                        //DMA Channel 1 Disabled
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
    for(p=0;p<100;p++)
    {
        if(RX_Buffer[p]==0x23&&RX_Buffer[p+7]==0x40)
        {
            Preuse_Param_ID[0]=RX_Buffer[p+1];          //1st Byte
            Preuse_Param_ID[1]=RX_Buffer[p+2];          //2nd Byte
            Preuse_Payload[0]=RX_Buffer[p+3];          //3rd Byte
            Preuse_Payload[1]=RX_Buffer[p+4];  //4th Byte
            Preuse_Payload[2]=RX_Buffer[p+5];  //5th Byte
            Preuse_Payload[3]=RX_Buffer[p+6];       //6th Byte
            Preuse_Check_Flag=1;
            break;
        }
    }
    memcpy(RX_Buffer, Buffer_Clear, sizeof(RX_Buffer));
    DMA1CTL|=DMAEN;
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
        P1OUT&=~BIT0;
        P2OUT&=~BIT3;    //Io Pin High
        Delay(1000);
        P2OUT|=BIT3;
        Buzzer_Set_Flag=0;
    }
}

/* Transmit Packet END*/


/*INPUT CAPTURE BEGIN*/

void Timer_A2_Init(void)
{
    //TA2CTL|=TACLR;  //Clear
    TA2CCTL2=CM_1+CAP+CCIE+CCIS_0+SCS;//Capture on Rising Edge,Capture Mode,Compare Capture Interrupt Enabled,Compare Capture Input Select,Synchronous Capture
    TA2CTL=TASSEL_2+ID_3+MC_2;  //Timer A2 Clock Set as SMCLK, Clock Divider Set as 8, Continous Counting Mode

}

void Timer_A2_DeInit(void)
{
    TA2CTL|=TACLR;  //Clear
    TA2CCTL2=0x00;//Capture on Rising Edge,Capture Mode,Compare Capture Interrupt Enabled,Compare Capture Input Select,Synchronous Capture
    TA2CTL=0x00;  //Timer A2 Clock Set as SMCLK, Clock Divider Set as 8, Continous Counting Mode
}

void GPIO_Init_Input_Capture(void)
{
    P2SEL&=~BIT5;   //Pin P2.5 Set as Input
    P2SEL|=BIT5;    //Pin P2.5 Set as Alternate Function
}

#pragma vector=TIMER2_A1_VECTOR //FFD6 is the Address

__interrupt void Timer_A2_ISR(void)
{

    if(TA2CCTL2&COV)
    {
        TA2CCTL2&=~COV;
    }
    Counter_Input_Capture=TA2CCR2;                   //Copying Value of Timer Compare Register Counter Value.
    Difference_Input_Capture=Counter_Input_Capture-Prev_Counter_Input_Capture;//Calculating Difference
    Prev_Counter_Input_Capture=Counter_Input_Capture;   //Setting New Counter to previous Counter
    Frequency_Array[Frequency_Array_Count]=Difference_Input_Capture;
    if(Frequency_Array_Count==100)
    {

        Frequency_Array_Count=0;
        int k=0;
        for(k=0;k<100;k++)
        {
            Frequency_Input+=Frequency_Array[k];
        }
        Frequency_Input/=100;
        Frequency_Send=1/(Frequency_Input*0.0000004);
    }
    Frequency_Array_Count+=1;
    TA2CCTL2&=~CCIFG;   //Clear Interrupt

}

/*INPUT CAPTURE END*/

/*PREUSE BEGIN*/
int Volt_Once=0;    //For Testing
int Mem_Once=0;
int Com_Once=0;
int Temp_Once=0;
int Speaker_Once=0;
int Buzzer_Once=0;
int Battery_Once=0;

void Preuse_Check(void)
{
    Preuse_Check_Flag=0;    //Resetting Preuse Check Flag to 0
    if(Preuse_Param_ID[0]==0x31&&Preuse_Param_ID[1]==0x97)
    {
        if(Preuse_Payload[3]==0x01&&Volt_Once==0)//Voltage Test
        {
            Volt_Once=1;
            if(x==0)
            {
                //for(k=0;k<1000;k++)
                //{
                    Preuse_TX_Pass_Packet();
                    Delay(100);
                    Preuse_TX_Echo_NULL();
                //}
            }
            if(x==1)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Fail_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }
            //Preuse_Clear();
        }
        if(Preuse_Payload[3]==0x02&&Mem_Once==0)//Memory Test
        {
            Mem_Once=1;
            if(x==0)
            {
                Preuse_TX_Pass_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
            }
            if(x==1)
            {
                Preuse_TX_Fail_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
            }
        }

        if(Preuse_Payload[3]==0x03&&Com_Once==0)//Communication Test
        {
            Com_Once=1;
            if(x==0)
            {
                Preuse_TX_Echo_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
            }
            if(x==1)
            {
                Preuse_TX_Fail_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
            }
        }

        if(Preuse_Payload[3]==0x04&&Temp_Once==0)//Temperature Test
        {
            Temp_Once=1;
            if(x==0)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Pass_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }
            if(x==1)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Fail_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }
            //Preuse_Clear();
        }
        if(Preuse_Payload[3]==0x05&&Speaker_Once==0)//Speaker Test
        {
            Timer_A2_Init();
            Delay(10);
            for(j=0;j<5;j++)
            {
                Update_Volume(volume);
                Delay(10);
                Low_Alarm();
                Delay(100);

                Alarm_IC_Reset();
                Delay(100);
                Alarm_IC_Config();
                Delay(100);
            }
            Timer_A2_DeInit();
            Speaker_Once=1;
            if(x==0)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Pass_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }
            if(x==1)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Fail_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }

        }
        if(Preuse_Payload[3]==0x06&&Buzzer_Once==0)//Buzzer Test
        {
            P1OUT&=~BIT0;
            P2OUT&=~BIT3;    //Io Pin High
            Delay(3000);
            P2OUT|=BIT3;
            Buzzer_Once=1;
            if(x==0)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Pass_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }
            if(x==1)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Fail_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }

        }
        if(Preuse_Payload[3]==0x07&&Battery_Once==0)//Battery Test
        {
            Battery_Once=1;
            if(x==0)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Pass_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }
            if(x==1)
            {
                //for(k=0;k<1000;k++)
                //{
                Preuse_TX_Fail_Packet();
                Delay(100);
                Preuse_TX_Echo_NULL();
                //}
            }

        }
       if(Volt_Once==1&&Mem_Once==1&&Temp_Once==1&&Speaker_Once==1&&Buzzer_Once==1&&Battery_Once==1&&Com_Once==1)
       {
           Volt_Once=0;
           Mem_Once=0;
           Temp_Once=0;
           Speaker_Once=0;
           Buzzer_Once=0;
           Battery_Once=0;
           Com_Once=0;
           Preuse_Clear();
           x^=1;
       }
    }

}

void Preuse_TX_Pass_Packet(void)
{
    DMA0CTL&=~DMAEN;     //DMA Channel 0 Disabled for UART TX
    TX_Packet[0]=0x23;      //Start of Frame
    TX_Packet[1]=0x33;      //Parameter ID
    TX_Packet[2]=0x97;      //Parameter ID
    TX_Packet[3]=0x00;      //Payload
    TX_Packet[4]=0x00;      //Payload
    TX_Packet[5]=0x00;      //Payload
    TX_Packet[6]=0x01;      //ID for Pass Packet
    TX_Packet[7]=0x40;      //End of Frame
    DMA0CTL|=DMAEN;     //DMA Channel 0 Enable for UART TX
}

void Preuse_TX_Fail_Packet(void)
{
    DMA0CTL&=~DMAEN;     //DMA Channel 0 Disabled for UART TX
    TX_Packet[0]=0x23;
    TX_Packet[1]=0x33;
    TX_Packet[2]=0x97;
    TX_Packet[3]=0x00;
    TX_Packet[4]=0x00;
    TX_Packet[5]=0x00;
    TX_Packet[6]=0x02;
    TX_Packet[7]=0x40;
    DMA0CTL|=DMAEN;     //DMA Channel 0 Enable for UART TX
}

void Preuse_TX_Echo_Packet(void)
{
    DMA0CTL&=~DMAEN;     //DMA Channel 0 Disabled for UART TX
    TX_Packet[0]=0x23;
    TX_Packet[1]=0x32;
    TX_Packet[2]=0x97;
    TX_Packet[3]=0x00;
    TX_Packet[4]=0x00;
    TX_Packet[5]=0x00;
    TX_Packet[6]=0x00;
    TX_Packet[7]=0x40;
    DMA0CTL|=DMAEN;     //DMA Channel 0 Enable for UART TX
}

void Preuse_TX_Echo_NULL(void)  //For Testing Only
{
    DMA0CTL&=~DMAEN;     //DMA Channel 0 Disabled for UART TX
    TX_Packet[0]=0x00;
    TX_Packet[1]=0x00;
    TX_Packet[2]=0x00;
    TX_Packet[3]=0x00;
    TX_Packet[4]=0x00;
    TX_Packet[5]=0x00;
    TX_Packet[6]=0x00;
    TX_Packet[7]=0x00;
    DMA0CTL|=DMAEN;     //DMA Channel 0 Enable for UART TX
}

void Preuse_Clear(void)
{
    Preuse_Param_ID[0]=0x00;
    Preuse_Param_ID[1]=0x00;
    Preuse_Payload[0]=0x00;
    Preuse_Payload[1]=0x00;
    Preuse_Payload[2]=0x00;
    Preuse_Payload[3]=0x00;
}
/*PREUSE END*/

