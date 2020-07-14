#include "lcd.h"
#include "constant.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define LCD_WAIT_UART_TX_DONE()    xSemaphoreTake(m_signal_lcd_uart, portMAX_DELAY)   

extern UART_HandleTypeDef huart4;
extern void app_error_handler(char * file, int line, int error_code);
static SemaphoreHandle_t m_signal_lcd_uart;
static uint8_t m_tmp_uart[1];

void lcd_cmd_done_cb(void)
{
    BaseType_t ctx_sw;
    xSemaphoreGiveFromISR(m_signal_lcd_uart, &ctx_sw);
    portYIELD_FROM_ISR(ctx_sw);
}

void lcd_uart_rx_enable()
{
    HAL_UART_Receive_IT(&huart4, m_tmp_uart, 1);
}

uint8_t lcd_uart_get_last_in_data(void)
{
    return m_tmp_uart[0];
}

void Lcd_Power(GPIO_PinState onoff) {
	HAL_GPIO_WritePin(PW_LCD_GPIO_Port, PW_LCD_Pin, onoff);
}

void Lcd_Init_Communication() {
    
////	huart4.Instance = UART4;
////	huart4.Init.BaudRate = 115200;
////	huart4.Init.WordLength = UART_WORDLENGTH_8B;
////	huart4.Init.StopBits = UART_STOPBITS_1;
////	huart4.Init.Parity = UART_PARITY_NONE;
////	huart4.Init.Mode = UART_MODE_TX_RX;
////	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
////	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
////	if (HAL_UART_Init(&huart4) != HAL_OK) {
////		app_error_handler(__FILE__, __LINE__, ERR_UART);
////	}
    if (m_signal_lcd_uart == NULL)
        m_signal_lcd_uart = xSemaphoreCreateBinary();
    
    if (m_signal_lcd_uart == NULL)
        app_error_handler(__FILE__, __LINE__, 0);
    
    lcd_uart_rx_enable();
}

void Lcd_Send(uint8_t *data, uint32_t size) {
	HAL_UART_Transmit_IT(&huart4, data, size);
    LCD_WAIT_UART_TX_DONE();
}
