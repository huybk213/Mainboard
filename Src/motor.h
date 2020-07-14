#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"

/**
 * @brief: Generate a signal to notifi other task that uart tx done
 * */
void motor_cmd_done_cb(void);

/**
 * @brief: Set power of motor
 * */
void motor_power(GPIO_PinState onoff);

/**
 * @brief: Enable communication between STM32 and motor
 * */
void motor_init_communication(void);

/**
 * @brief: Send command to motor
 * @param data: The buffer want to send
 * @param len: The length of buffer
 * */
void motor_send(uint8_t *data, uint32_t size);

/**
 * @brief: Enable STM32 RX motor data
 * */
void motor_uart_rx_enable(void);

/**
 * @brief: Get the last received byte
 * */
uint8_t esp32_communication_get_last_in_data(void);

#endif /* MOTOR_H */
