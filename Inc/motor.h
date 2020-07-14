/**
 * This file contains the function to control the motor module
 * */

#ifndef __INCLUDE_MOTOR_H_
#define __INCLUDE_MOTOR_H_

#include "main.h"

void Motor_Power(GPIO_PinState onoff);

void Motor_Init_Communication(void);

void Motor_Send(uint8_t* data, uint32_t size);

#endif
