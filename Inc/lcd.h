/**
 * This file contains the function to control eyes of robot
 * */

#ifndef __INCLUDE_LCD_H_
#define __INCLUDE_LCD_H_

#include "main.h"
#include <stdint.h>

void Lcd_Power(GPIO_PinState onoff);

void Lcd_Init_Communication(void);

/**
 * @brief: Send command to lcd
 * @param data: The buffer want to send
 * @param len: The length of buffer
 * */
void Lcd_Send(uint8_t *data, uint32_t size);

/**
 * @brief: Generate a signal to notifi other task that uart tx done
 * */
void lcd_cmd_done_cb(void);

/**
 * @brief: Enable STM32 RX lcd data
 * */
void lcd_uart_rx_enable(void);

/**
 * @brief: Get last byte lcd rx data
 * */
uint8_t lcd_uart_get_last_in_data(void);

#endif
