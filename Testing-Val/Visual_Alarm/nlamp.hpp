/********************************************************************
** @file     nlamp.hpp
** @brief    Notification Lamp Interface
** @author   Copyright (C) 2021  Noccarc Robotics
** @author   Nipun Pal <nipun.k@noccarc.com>
** @version  v1.03
** @modified 24/12/21  Added Nlamp support as per IEC standard
**
********************************************************************/
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <stdlib.h>

// flags
#define TRUE 1
#define FALSE 0

// Pulse count
#define MP_PULSE_COUNT 3
#define HP_PULSE_COUNT 5

// Step size definitions in nanoseconds
#define HP_TR_STEP_SIZE 200000
#define MP_TR_STEP_SIZE 100000

#define LP_TR_STEP_SIZE 20000
#define LP_TF_STEP_SIZE 25000

#define MAX_PULSE_AMPLITUDE 1000000

// Pulse on of time  definitions in microseconds
#define HP_PULSE_ON_TIME 200000
#define HP_PULSE_OFF_TIME 300000

#define MP_PULSE_ON_TIME 500000
#define MP_PULSE_OFF_TIME 800000

#define LP_PULSE_ON_TIME 1000000

// Control flags for the lamp operation
bool lp = FALSE;
bool mp = FALSE;
bool hp = FALSE;

// Buffer for creation of string to be sent to bash
char buf[200];


//***************************************************** Function prototypes *******************************************************
void disable_all_channels();
void enable_HP();
void enable_MP();
void enable_LP();
