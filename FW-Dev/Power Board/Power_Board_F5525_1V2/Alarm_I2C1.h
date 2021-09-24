/*
 * Alarm_I2C1.h
 *
 *  Created on: 30-Aug-2021
 *      Author: Nikhil Pachkor
 */

#ifndef ALARM_I2C1_H_
#define ALARM_I2C1_H_

////////////////////////////////ALARM/////////////////////////




void High_Margin_Data(void);                //Setting For High Margin Data

void Low_Margin_Data(void);                 //Setting For Low Margin Data

void Alarm_Pulse_Config(void);              //Setting for Alarm Pulse Config

void Trigger_Pulse_Config(void);            //S

void Power_Up_DAC(void);

void High_Priority_Alarm_Setting(void);

void High_Alarm(void);

void Medium_Alarm(void);

void Low_Alarm(void);

void Update_Volume(int);

void Alarm_IC_Reset(void);

void Timer_A0_Init(void);

void Frequency(int Frequency);

void I2C1_Init(void);

void I2C1_Reset(void);

_Bool Connected_I2C1_Device(void);

_Bool I2C1_Bus_Status_Timeout(void);

_Bool I2C1_Transmit_Timeout(void);

_Bool I2C1_Receive_Timeout(void);

_Bool I2C1_Acknowledge_Timeout(void);

void Alarm_IC_Config(void);
////////////////////////////ALARM////////////////////////////



#endif /* ALARM_I2C1_H_ */
