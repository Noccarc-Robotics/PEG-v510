/*
 * Sys_Config.c
 *
 *  Created on: 30-Aug-2021
 *      Author: Nikhil Pachkor
 */
#include "msp430.h"

volatile unsigned int MS_Delay;

void SetVCoreUp (unsigned int level)
{
    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;
    // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
    // Wait till SVM is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0);
    // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Wait till new level reached
    if ((PMMIFG & SVMLIFG))
        while ((PMMIFG & SVMLVLRIFG) == 0);
    // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
}

void Clock_Check(void)
{
    //P7SEL|=BIT7;
    //P7DIR|=BIT7;
    P2SEL|=BIT2;
    P2DIR|=BIT2;
    P4DIR|=BIT7;
}

void System_Clock_Init(void)
{
    /* Power settings */
    SetVCoreUp(1u);
    SetVCoreUp(2u);
    SetVCoreUp(3u);

    P5SEL|=BIT2+BIT3;

    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL3 = SELREF__XT2CLK ;                // select XT2 as FLL source
    UCSCTL6 &= ~XT2OFF;
    UCSCTL6 |= XT2DRIVE_2;
    UCSCTL0 = 0x0000u;                        // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                      // Set RSELx for DCO = 50 MHz
    UCSCTL2 = 4u;                            // Set DCO Multiplier for 25MHz  // (N + 1) * FLLRef = Fdco // (762 + 1) * 32768 = 25.00MHz
    UCSCTL4 = SELA__DCOCLK | SELS__DCOCLK | SELM__DCOCLK;
    __bic_SR_register(SCG0);                  // Enable the FLL control loop
    __delay_cycles(10000u);

    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);     // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                      // Clear fault flags
    }
    while (SFRIFG1&OFIFG);                    // Test oscillator fault flag

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
    TA1CCR0=2500;                   //Value Required to count 1 milli second of time
    TA1CCTL0 |= CCIE;               //Enable Timer A1 Interrupt
}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void timer1_A0_ISR(void)
{
    MS_Delay++;         //Increment Tick
    TA1CCTL0 &= ~CCIFG; //Clear Interrupt
}
