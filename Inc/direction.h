/**
 * This file contains the function to control the direction_detection module
 * */

#ifndef __INCLUDE_DIRECTION_H_
#define __INCLUDE_DIRECTION_H_

#include "main.h"

void Direction_Power(GPIO_PinState onoff);

void Direction_Init_Communication(void);

void Direction_Send(uint8_t *data, uint32_t size);

#endif
