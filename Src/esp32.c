#include <esp32.h>
#include "stm32f429xx.h"
#include "log.h"
#include "string.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "semphr.h"
#include "ringbuff/ringbuff.h"

#define ESP32_UART_RX_BUFFER_SIZE   (96U)
#define ESP32_WAIT_UART_TX_DONE()      xSemaphoreTake(m_signal_uart_esp32, portMAX_DELAY)
#define LOCK_ESP32_UART_TX()                xSemaphoreTake(m_lock_uart_tx_esp32, portMAX_DELAY)      
#define UNLOCK_ESP32_UART_TX()              xSemaphoreGive(m_lock_uart_tx_esp32) 

static const char *TAG = "ESP32";
extern SPI_HandleTypeDef hspi2;
extern void Error_Handler(void);
extern void Assert_Is_NULL(void* value);
extern UART_HandleTypeDef huart7;

uint8_t* TxBuffer = NULL;
uint8_t* RxBuffer = NULL;
static SemaphoreHandle_t m_spi_tx_done;
static SemaphoreHandle_t m_spi_slave_ready;
static SemaphoreHandle_t m_signal_uart_esp32, m_protect_ring_buffer;
static SemaphoreHandle_t m_lock_uart_tx_esp32;

static uint8_t m_tmp_uart_buffer[1];
static ringbuff_t m_esp32_uart_rx_ring_buffer;
static uint8_t m_uart_rx_raw_buffer[ESP32_UART_RX_BUFFER_SIZE];

void Esp32_Power(GPIO_PinState status) {
	HAL_GPIO_WritePin(PW_WIFI_GPIO_Port, PW_WIFI_Pin, status);
}

void Esp32_Change_Spi_Cs(GPIO_PinState State) {
	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, State);
}

void esp32_spi_communication_initialize(void) {
	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

	TxBuffer = (uint8_t*)pvPortMalloc((size_t)RESPONSE_BUF_SIZE);
	Assert_Is_NULL(TxBuffer);
	RxBuffer = (uint8_t*)pvPortMalloc((size_t)RESPONSE_BUF_SIZE);
	Assert_Is_NULL(RxBuffer);
    
    m_spi_tx_done = xSemaphoreCreateBinary();
    Assert_Is_NULL(m_spi_tx_done);
    
    m_spi_slave_ready = xSemaphoreCreateBinary();
}

uint8_t Esp32_Is_Available() {
	memset(TxBuffer, 0, RESPONSE_BUF_SIZE);
	memset(RxBuffer, 0, RESPONSE_BUF_SIZE);
	*(TxBuffer + HEADER_OFFSET_REQUEST_TYPE) = CMD_PING;

	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	//Ignore the first data because the data could be dummy
	HAL_SPI_TransmitReceive(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE , 100);
	HAL_Delay(1);

	//Get a pong message
	HAL_SPI_TransmitReceive(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE , 100);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

	uint8_t result = *RxBuffer == CMD_PONG;
	return result;
}

void Esp32_Send(uint8_t *cmd, size_t size) {
	memset(TxBuffer, 0, RESPONSE_BUF_SIZE);
	memset(RxBuffer, 0, RESPONSE_BUF_SIZE);
	uint8_t *pTrack = cmd;
	RK_LOGI(TAG, "Start sending audio \n");

	*(TxBuffer + HEADER_OFFSET_REQUEST_TYPE) = CMD_CHATBOX_AUDIO;
	*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_TOTAL_SIZE)) = (uint32_t) size;

	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	while (pTrack - cmd + HEADER_REQUEST_MAX_DATA_SIZE <= size) {
		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_CURRENT_OFFSET)) =	(uint32_t) (pTrack - cmd);
		memcpy(TxBuffer + HEADER_OFFSET_REQUEST_DATA, pTrack,	HEADER_REQUEST_MAX_DATA_SIZE);
		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_DATA_SIZE)) =	(uint32_t) (HEADER_REQUEST_MAX_DATA_SIZE);
		HAL_SPI_TransmitReceive(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE , 4000);
		HAL_Delay(50);
		//Move the pointer to next position of audio
		pTrack += HEADER_REQUEST_MAX_DATA_SIZE;
	}

	int remainChar = cmd + size - pTrack;
	if (remainChar > 0) {
		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_CURRENT_OFFSET)) =	(uint32_t) (pTrack - cmd);
		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_DATA_SIZE)) =	(uint32_t) remainChar;
		memset(TxBuffer + HEADER_OFFSET_REQUEST_DATA, 0, HEADER_REQUEST_MAX_DATA_SIZE);
		memcpy(TxBuffer + HEADER_OFFSET_REQUEST_DATA, pTrack, remainChar);
		HAL_SPI_TransmitReceive(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE , 4000);
	}
	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

	RK_LOGI(TAG, "Total bytes %d \n ", pTrack - cmd + remainChar);
	RK_LOGI(TAG, "Send audio: Done. \n");
}

void Esp32_Send2(Audio_Node* audio){
	memset(TxBuffer, 0, RESPONSE_BUF_SIZE);
	memset(RxBuffer, 0, RESPONSE_BUF_SIZE);
	Audio_Node* audio_pointer = audio;
	uint32_t current_offset = 0;

	*(TxBuffer + HEADER_OFFSET_REQUEST_TYPE) = CMD_CHATBOX_AUDIO;
	*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_TOTAL_SIZE)) = 96000;

	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	do {
		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_CURRENT_OFFSET)) =	current_offset;
		memcpy(TxBuffer + HEADER_OFFSET_REQUEST_DATA, audio_pointer->data,	audio_pointer->data_size);
		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_DATA_SIZE)) =	audio_pointer->data_size;
		HAL_SPI_TransmitReceive(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE , 4000);
		current_offset += audio_pointer->data_size;
		vTaskDelay(5);
		//Move the pointer to next position of audio

		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_CURRENT_OFFSET)) =	current_offset;
		memcpy(TxBuffer + HEADER_OFFSET_REQUEST_DATA, audio_pointer->data + audio_pointer->data_size/2,	audio_pointer->data_size);
		*((uint32_t*) (TxBuffer + HEADER_OFFSET_REQUEST_DATA_SIZE)) =	audio_pointer->data_size;
		HAL_SPI_TransmitReceive(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE , 4000);
		current_offset += audio_pointer->data_size;
		vTaskDelay(5);
		audio_pointer = audio_pointer->next;
	}while(audio_pointer != audio);
	HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);
}


Esp_ResponseTypeDef* Esp32_Get_Response() {
	Esp_ResponseTypeDef* result = (Esp_ResponseTypeDef*)calloc(1, sizeof(Esp_ResponseTypeDef));

	*(TxBuffer + HEADER_OFFSET_REQUEST_TYPE) = CMD_GET_DATA;
	memset(RxBuffer, 0, RESPONSE_BUF_SIZE);

	Esp32_Change_Spi_Cs(GPIO_PIN_RESET);
    uint32_t current_tick = xTaskGetTickCount();
	vTaskDelayUntil(&current_tick, 1);
	HAL_SPI_TransmitReceive_DMA(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE);
	if(*RxBuffer != 0 && *RxBuffer != CMD_PONG)
	{
		//The first times sending spi, PONG msg could be sent by esp32
		goto FINISH_RECEIVING;
	}
	vTaskDelay(10);

	HAL_SPI_TransmitReceive(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE, 4000);
	if(*RxBuffer == 0 || *RxBuffer == CMD_PONG) //No more data to be receive
	{
		free(result);
		return NULL;
	}

FINISH_RECEIVING:
    RK_LOGI(TAG, "FINISH_RECEIVING\n");
	Esp32_Change_Spi_Cs(GPIO_PIN_SET);
	result->data = calloc(RESPONSE_BUF_SIZE, sizeof(uint8_t));
	memcpy(result->data, RxBuffer, RESPONSE_BUF_SIZE);
	return result;
}

bool esp32_signal_spi_done(bool in_isr)
{
    if (in_isr)
    {
        BaseType_t ctx_sw;
        if (!xSemaphoreGiveFromISR(m_spi_tx_done, &ctx_sw))
            return false;
        portYIELD_FROM_ISR(ctx_sw);
        return true;
    }
    return xSemaphoreGive(m_spi_tx_done);
}

static bool esp32_wait_spi_done(uint32_t wait_ms)
{
    return xSemaphoreTake(m_spi_tx_done, wait_ms);
}

void esp32_notify_slave_ready(bool isr)
{
    if (isr)
    {
         //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
        //looking at the time between interrupts and refusing any interrupt too close to another one.
        static uint32_t lasthandshaketime = 0;
        uint32_t currtime = xTaskGetTickCountFromISR();
        
        uint32_t diff = currtime - lasthandshaketime;
        if (diff < 4) 
            return; //ignore everything <1ms after an earlier irq
        
        lasthandshaketime = currtime;

        //Give the semaphore.
        BaseType_t mustYield = false;
        xSemaphoreGiveFromISR(m_spi_slave_ready, &mustYield);
        if (mustYield) 
            portYIELD_FROM_ISR(mustYield);   
    }
    else
    {
        xSemaphoreGive(m_spi_slave_ready);
    }
}

void esp32_notify_slave_busy(void)
{
    if (m_spi_slave_ready == NULL)
        return;
        
    BaseType_t mustYield = false;
    xSemaphoreTakeFromISR(m_spi_slave_ready, &mustYield);
    if (mustYield) 
        portYIELD_FROM_ISR(mustYield);  
}

uint8_t* esp32_get_audio_data(uint32_t wait_time) {
    *(TxBuffer + HEADER_OFFSET_REQUEST_TYPE) = CMD_GET_DATA;
    memset(RxBuffer, 0, RESPONSE_BUF_SIZE);
    
    //xSemaphoreTake(m_spi_slave_ready, portMAX_DELAY);

    Esp32_Change_Spi_Cs(GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive_DMA(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE) != HAL_OK)
    {
        app_error_handler(__FILE__, __LINE__, 1);
    }
   
    esp32_wait_spi_done(wait_time);

    Esp32_Change_Spi_Cs(GPIO_PIN_SET);
    return RxBuffer;
}

void Esp32_Resend_Last_Command_DMA(){
	memset(RxBuffer, 0, RESPONSE_BUF_SIZE);
	*(TxBuffer + HEADER_OFFSET_REQUEST_TYPE) = CMD_GET_DATA;
	Esp32_Change_Spi_Cs(GPIO_PIN_RESET);
	vTaskDelay(50);
	if(HAL_SPI_TransmitReceive_DMA(&hspi2, TxBuffer, RxBuffer, RESPONSE_BUF_SIZE) != HAL_OK){
		Error_Handler(); //TODO: change error value
	}
}

void esp32_uart_rx_enable(void)
{
    HAL_UART_Receive_IT(&huart7, m_tmp_uart_buffer, 1);
}

uint8_t esp32_communication_get_last_in_data(void)
{
    return m_tmp_uart_buffer[0];
}

void esp32_communication_rx_data(bool in_isr)
{
    BaseType_t ctx_sw;
    if (in_isr)
    {
        if (xSemaphoreTakeFromISR(m_protect_ring_buffer, &ctx_sw) == pdFAIL)
            return;
    }
    
    if (ringbuff_write(&m_esp32_uart_rx_ring_buffer, m_tmp_uart_buffer, 1) != 1)
    {
        RK_LOGI(TAG, "ESP32 RX : Ring buffer full\r\n");
    }
    
    xSemaphoreGiveFromISR(m_protect_ring_buffer, &ctx_sw);
    
    portYIELD_FROM_ISR(ctx_sw);
}

uint8_t esp32_communication_get_bytes_availble()
{
    if (xSemaphoreTake(m_protect_ring_buffer, portMAX_DELAY) == pdFAIL)
            return 0;
    
    uint8_t size = (uint8_t)ringbuff_get_full(&m_esp32_uart_rx_ring_buffer);
    
    xSemaphoreGive(m_protect_ring_buffer);
    
    return size;
}

uint8_t esp32_communication_get_bytes(void)
{
    if (xSemaphoreTake(m_protect_ring_buffer, portMAX_DELAY) == pdFAIL)
            return 0;
    
    uint8_t data;
    ringbuff_read(&m_esp32_uart_rx_ring_buffer, &data, 1);
    
    xSemaphoreGive(m_protect_ring_buffer);
    
    return data;
}

void esp32_cmd_uart_tx_done(void)
{
    BaseType_t ctx_sw;
    xSemaphoreGiveFromISR(m_signal_uart_esp32, &ctx_sw);
    portYIELD_FROM_ISR(ctx_sw);
}

void esp32_communication_uart_tx(uint8_t * data, uint16_t len)
{
    LOCK_ESP32_UART_TX();
    HAL_UART_Transmit_IT(&huart7, data, len);
    ESP32_WAIT_UART_TX_DONE();
    UNLOCK_ESP32_UART_TX();
}

void esp32_uart_communication_initialize(void) 
{
    
    ringbuff_init(&m_esp32_uart_rx_ring_buffer, m_uart_rx_raw_buffer, ESP32_UART_RX_BUFFER_SIZE);
    
    if (m_signal_uart_esp32 == NULL)
        m_signal_uart_esp32 = xSemaphoreCreateBinary();
    
    if (m_signal_uart_esp32 == NULL)
        app_error_handler(__FILE__, __LINE__, 0);
    
    if (m_protect_ring_buffer == NULL)
        m_protect_ring_buffer = xSemaphoreCreateMutex();
    
    if (m_lock_uart_tx_esp32 == NULL)
        m_lock_uart_tx_esp32 = xSemaphoreCreateMutex();
    xSemaphoreGive(m_lock_uart_tx_esp32);
    
    if (m_protect_ring_buffer == NULL)
        app_error_handler(__FILE__, __LINE__, 0);
    
    xSemaphoreGive(m_protect_ring_buffer);
      
    esp32_uart_rx_enable();
}


