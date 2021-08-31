/*
 * Coulomb_I2C.h
 *
 *  Created on: 30-Aug-2021
 *      Author: Nikhil Pachkor
 */

#ifndef COULOMB_I2C_H_
#define COULOMB_I2C_H_

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



#endif /* COULOMB_I2C_H_ */
