/*
 * Sys_Config.h
 *
 *  Created on: 30-Aug-2021
 *      Author: Nikhil Pachkor
 */

#ifndef SYS_CONFIG_H_
#define SYS_CONFIG_H_

void System_Clock_Init(void);           //Function for Initializing System Clock

void SetVCoreUp (unsigned int level);   //Function for Setting the Core Voltage level for increased Clock Speed

void Clock_Check(void);                 //Function for Checking Clock ACLK MCLK and SMCLK

void Delay(int Milli_Second);

void Timer_A1_Init(void);

#endif /* SYS_CONFIG_H_ */
