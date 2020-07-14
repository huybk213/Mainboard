#ifndef INCLUDE_ESP32_H_
#define INCLUDE_ESP32_H_

#include "stddef.h"
#include "main.h"
#include "audio_play.h"
#include "command.h"

typedef struct {
	uint8_t *data;
	uint32_t data_size;
	uint16_t cmd_id;
	CommandTypeDef cmd_type;
	Result_TypeDef status;
} Esp_ResponseTypeDef;


/**
 * @brief ON/OFF cs pin of spi
 * */
void Esp32_Change_Spi_Cs(GPIO_PinState State);


/**
 * @brief Check whether esp32 is available or not
 * by sending a ping message
 */
uint8_t Esp32_Is_Available(void);

/**
 * @brief Send  data to server
 * @param audio: The ponter to the audio
 * @param size: The length of the audio (in bytes)
*/

void Esp32_Send(uint8_t *cmd, size_t size);

void Esp32_Send2(Audio_Node* audio);

/**
 * @brief: Receive the response sent by esp32
 * @param buffer: The buffer in which response's data is saved
 * @param maxSize: The maximum of buffer
 * */
Esp_ResponseTypeDef* Esp32_Get_Response(void);

void Esp32_Resend_Last_Command_DMA(void);

/**
 * @brief: Test
 * */
void audio_play_request_more_done(void);

/**
 * @brief: Initialize SPI communication between STM32 and ESP32
 * */
void esp32_spi_communication_initialize(void);

/**
 * @brief: Initialize UART communication between STM32 and ESP32
 * */
void esp32_uart_communication_initialize(void);

/**
 * @brief: Send a UART buffer to ESP32
 * @param data: The buffer want to send
 * @param len: The length of buffer
 * */
void esp32_communication_uart_tx(uint8_t * data, uint16_t len);

/**
 * @brief: Send a signal to other task to notifi that uart tx done
 * */
void esp32_cmd_uart_tx_done(void);

/**
 * @brief: Restart UART communication between STM32 and ESP32
 * */
void esp32_uart_rx_enable(void);

/**
 * @brief: Get current data size in ring buffer
 * @retval: Data length
 * */
uint8_t esp32_communication_get_bytes_availble(void);

/**
 * @brief: Get c1 byte from ring buffer
 * @retval: Data
 * */
uint8_t esp32_communication_get_bytes(void);

/**
 * @brief: Put uart received data into ring buffer
 * @param in_isr: Function called in interrupt service routine
 * */
void esp32_communication_rx_data(bool in_isr);

/**
 * @brief: Send a signal to other task to notifi that spi tx-rx done
 * @param in_isr: Function called in interrupt service routine
 * @retval TRUE : success
 *         FALSE : Send signal failed
 * */
bool esp32_signal_spi_done(bool in_isr);

/**
 * @brief: Get esp32 audio data
 * @param wait_time: Maximum wait time
 * */
uint8_t* esp32_get_audio_data(uint32_t wait_time);

/**
 * @brief: Send a signal to other task to notifi that spi slave ready for new spi transaction
 * */
void esp32_notify_slave_ready(bool isr);

void esp32_notify_slave_busy(void);

#endif /* INCLUDE_ESP32_H_ */
