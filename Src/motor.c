#include "main.h"
#include "constant.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "motor.h"

#define MOTOR_WAIT_UART_DONE()    xSemaphoreTake(m_signal_uart, portMAX_DELAY)      

static SemaphoreHandle_t m_signal_uart;
extern void app_error_handler(char * file, int line, int error_code);
static uint8_t m_tmp_uart[1];

extern UART_HandleTypeDef huart2;

void motor_uart_rx_enable(void){
    HAL_UART_Receive_IT(&huart2, m_tmp_uart, 1);
}

void motor_cmd_done_cb(void){
    BaseType_t ctx_sw;
    xSemaphoreGiveFromISR(m_signal_uart, &ctx_sw);
    portYIELD_FROM_ISR(ctx_sw);
}

void motor_power(GPIO_PinState onoff) {
	HAL_GPIO_WritePin(PW_MOTOR_GPIO_Port, PW_MOTOR_Pin, onoff);
}

void motor_init_communication(void) {    
    if (m_signal_uart == NULL)
        m_signal_uart = xSemaphoreCreateBinary();
    
    if (m_signal_uart == NULL)
        app_error_handler(__FILE__, __LINE__, 0);
    
    motor_uart_rx_enable();
}

void motor_send(uint8_t *data, uint32_t size) {
//	HAL_UART_Transmit(&huart2, data, size, 0xFFFF);
    if (HAL_UART_Transmit_IT(&huart2, data, size) != HAL_OK)
    {
        app_error_handler(__FILE__, __LINE__, ERR_UART);
    }
    MOTOR_WAIT_UART_DONE();
}
