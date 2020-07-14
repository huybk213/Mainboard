#include "direction.h"
#include "constant.h"

extern void app_error_handler(char * file, int line, int error_code);
extern SPI_HandleTypeDef hspi4;

void Direction_Power(GPIO_PinState onoff) {
	HAL_GPIO_WritePin(SOUND_CS_GPIO_Port, SOUND_CS_Pin, onoff);
}

void Direction_Init_Communication() {
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi4) != HAL_OK) {
		app_error_handler(__FILE__, __LINE__, ERR_SPI_INIT);
	}
}

void Direction_Send(uint8_t *data, uint32_t size) {

}
