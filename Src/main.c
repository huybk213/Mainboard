/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "command.h"
#include "constant.h"
#include "esp32.h"
#include "direction.h"
#include "log.h"
#include "lcd.h"
#include "audio_play.h"
#include "motor.h"
#include "voice_command.h"
#include "mfcc.h"
#include "gru.h"
#include "dnn.h"

#ifdef RK_CODEC_ENABLE
#include "es8311.h"
#include "stm32429i_mainboard_audio.h"
#include "Resource/audio_please_repeat.h"
#endif /* RK_CODEC_ENABLE */

#include "SEGGER_RTT.h"
#include "app_cli.h"
#include "vsm_shell.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "cpu_utils.h"
#include "motor.h"
#include "esp32.h"
#include "voice_command.h"
#include "rk_serial_protocol.h"
#include "rk_serial_protocol_id.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TAG    "main"
#define BUFFER_SIZE         ((uint32_t)0x0100)
#define WRITE_READ_ADDR     ((uint32_t)0x000)
//#define WRITE_READ_ADDR     ((uint32_t)0x004)
#define REFRESH_COUNT       ((uint32_t)0x056A)   /* SDRAM refresh counter (90MHz SDRAM clock) */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUFFER_SIZE           16000
#define MAX_AUDIO_LENGTH            64000
#define AUDIO_SPI_RX_BUFFER_SIZE    (16000)
#define DMA_SPI_RX_BUFFER_SIZE      (4092)
#define PING_PONG_TIMEOUT           (10000)
#define ESP32_KEEP_ALIVE_TIMEOUT    (60000)

#define SDRAM_SIZE                  (8*1024*1024)

typedef uint32_t sram_addr_t;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_i2s3_ext_rx;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart7_tx;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* USER CODE BEGIN PV */
osThreadId_t task_handle_log;
const osThreadAttr_t voice_task_attributes = {
  .name = "voiceCMD",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 512 * 4
};
extern int16_t *audio_buffer_in;

/* Read/Write Buffers */
sram_addr_t aTxBuffer[BUFFER_SIZE];
sram_addr_t aRxBuffer[BUFFER_SIZE];
__IO uint32_t uwWriteReadStatus = 0;
static void Fill_Buffer(sram_addr_t *pBuffer, uint32_t uwBufferLenght, sram_addr_t value)
{
    uint32_t tmpIndex = 0;

    /* Put in global buffer different values */
    for (tmpIndex = 0; tmpIndex < uwBufferLenght; tmpIndex++ )
    {
        pBuffer[tmpIndex] = value;
    }
}
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);
FMC_SDRAM_CommandTypeDef command;

/* Counter index */
uint32_t uwIndex = 0;
/**
 * @brief the buffer stores audio stream
*/
int16_t *audio_buffer_1s;
extern VAD_ResultTypeDef vad_result;


void Assert_Is_NULL(void* value);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S3_Init(void);
static void MX_UART7_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void task_uart(void * argument);
static void esp32_on_data_callback(uint8_t id, uint8_t len, uint8_t * data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief The buffer used to send a command to motor's module
 */
uint8_t Motor_Command_TX[CMD_UART_MAX_LENGTH];

/**
 * @brief The buffer used to send a command to eye's module
 */
uint8_t Eye_Command_TX[CMD_UART_MAX_LENGTH];


static volatile uint32_t success = 0;
static void sdram_test_loop()
{
    vTaskDelay(2000);
    //#define SDRAM_BANK_ADDR 0xD0000000
    #define SDRAM_BANK_ADDR 0xC0000000
    Fill_Buffer(aTxBuffer, BUFFER_SIZE, 0x1234);   
    RK_LOGI(TAG, "RAM size %d, test buffer size %d, filter count %d\r\n", SDRAM_SIZE, BUFFER_SIZE*sizeof(sram_addr_t), (SDRAM_SIZE/(BUFFER_SIZE*sizeof(sram_addr_t)) - 1));
    while(1)
    {
        sram_addr_t * pTest = (sram_addr_t*)(SDRAM_BANK_ADDR + WRITE_READ_ADDR);
        for (uint32_t idx = 0; idx < (SDRAM_SIZE/(BUFFER_SIZE*sizeof(sram_addr_t)) - 1); idx++)
        {          
          /* Write data to the SDRAM memory */
          for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
          {
            *(__IO sram_addr_t*) (pTest + uwIndex*sizeof(sram_addr_t) + idx*BUFFER_SIZE*sizeof(sram_addr_t)) = aTxBuffer[uwIndex];
          }    
          
          /* Read back data from the SDRAM memory */
          for (uwIndex = 0; uwIndex < BUFFER_SIZE; uwIndex++)
          {
            aRxBuffer[uwIndex] = *(__IO sram_addr_t*) (pTest + uwIndex*sizeof(sram_addr_t) + idx*BUFFER_SIZE*sizeof(sram_addr_t));
           } 

          /*##-3- Checking data integrity ############################################*/    

    //      for (uwIndex = 0; (uwIndex < BUFFER_SIZE) && (uwWriteReadStatus == 0); uwIndex++)
          for (uwIndex = 0; (uwIndex < BUFFER_SIZE); uwIndex++)
          {
            if (aRxBuffer[uwIndex] != aTxBuffer[uwIndex])
            {
              uwWriteReadStatus++;
            }
          }	

          if (uwWriteReadStatus)
          {
            /* KO */
            /* Turn on LED4 */
            RK_LOGI(TAG, "SDRAM verify error count %d\r\n", uwWriteReadStatus);
            assert_failed(__FILE__, __LINE__);     
          }
          else
          { 
            /* OK */
            /* Turn on LED3 */
              success++;
              static uint32_t j = 0;
              if (j++ == 2000)
              {
                  j = 0;
                  RK_LOGI(TAG, "SDRAM loop : %d\r\n", success);
              }
          }
        }   
        vTaskDelay(1);
    }
}

void cli_get_char(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    uwTickPrio = TICK_INT_PRIORITY;     // Workaround for stm32 bug
    /* https://community.st.com/s/question/0D50X0000C6duTo/halnvicsetpriority-assert-issue-and-incorrect-preempt-priority-value-for-the-stm32f4-stm32g4- */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2S3_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  


	audio_buffer_1s = pvPortMalloc(SAMPLE_1_SEC *2);
  Assert_Is_NULL(audio_buffer_1s);
    
	esp32_spi_communication_initialize();
  KWS_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  task_handle_log = osThreadNew(task_uart, NULL, &voice_task_attributes);
  
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
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
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */
#if 1
  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_3;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_2;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 7;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  /* Program the SDRAM external device */
  SDRAM_Initialization_Sequence(&hsdram1, &command);   
#endif /* SDRAM_ENABLE */
  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SOUND_CS_GPIO_Port, SOUND_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, PW_SOUND_Pin|PW_LCD_Pin|PW_MOTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PW_WIFI_Pin|PW_AMP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : SOUND_CS_Pin */
  GPIO_InitStruct.Pin = SOUND_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SOUND_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PW_SOUND_Pin PW_LCD_Pin PW_MOTOR_Pin */
  GPIO_InitStruct.Pin = PW_SOUND_Pin|PW_LCD_Pin|PW_MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PW_WIFI_Pin PW_AMP_Pin */
  GPIO_InitStruct.Pin = PW_WIFI_Pin|PW_AMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP32_HAS_DATA_Pin */
  GPIO_InitStruct.Pin = ESP32_HAS_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ESP32_HAS_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AMP_EN_Pin */
  GPIO_InitStruct.Pin = AMP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMP_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP32_CS_Pin */
  GPIO_InitStruct.Pin = ESP32_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ESP32_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

void cli_get_char()
{
    int key = SEGGER_RTT_GetKey();
    if (key < 0)
    {
        return;
    }
    uint8_t val = (uint8_t)key;
    cli_input_insert(&val, 1);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4)
    {
//        RK_LOGI(TAG, "Handshake level %s\r\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) ? "high" : "low");
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
            esp32_notify_slave_ready(true);
        else
            esp32_notify_slave_busy();
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
//    __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
    
    if (huart == &huart7)
    {
        esp32_uart_rx_enable(); //TODO:
    }
    else if (huart == &huart2)
    {
        motor_uart_rx_enable();
    }
    else if (huart == &huart4) 
    {
        lcd_uart_rx_enable();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 
{
	if (huart == &huart2) 
    {
//		free(huart->pTxBuffPtr);
        motor_cmd_done_cb();
	}
    else if (huart == &huart7)
    {
        esp32_cmd_uart_tx_done();
    }
    else if (huart == &huart4)
    {
        lcd_cmd_done_cb();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7)
    {
//        uint8_t recv = esp32_communication_get_last_in_data();
        esp32_communication_rx_data(true);
        esp32_uart_rx_enable();
    }
    else 
        if (huart == &huart2) 
    {
        motor_uart_rx_enable();
	}
    else if (huart == &huart4) 
    {
        uint8_t recv = lcd_uart_get_last_in_data();
        lcd_uart_rx_enable();
    }
}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
//    RK_LOGE(TAG, "HAL_SPI_RxCpltCallback\r\n");
    if (hspi == &hspi2)
    {
        esp32_signal_spi_done(true);
    }
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
{
//    RK_LOGE(TAG, "HAL_SPI_RxHalfCpltCallback\r\n");
//    if (hspi == &hspi2)
//    {
//        BaseType_t ctx_sw;
//        if (xStreamBufferSendFromISR(m_spi_rx_stream_buffer, g_spi_rx_buffer, DMA_SPI_RX_BUFFER_SIZE/2, &ctx_sw) == 0)
//        {
//            RK_LOGE(TAG, "SPI RX queue full\r\n");  
//            app_error_handler(__FILE__, __LINE__, 1);            
//        }
//        portYIELD_FROM_ISR(ctx_sw);
//    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  	if(hspi->Instance == SPI2){
  		app_error_handler(__FILE__, __LINE__, 1);
        
//        #define HAL_SPI_ERROR_NONE              (0x00000000U)   /*!< No error                               */
//        #define HAL_SPI_ERROR_MODF              (0x00000001U)   /*!< MODF error                             */
//        #define HAL_SPI_ERROR_CRC               (0x00000002U)   /*!< CRC error                              */
//        #define HAL_SPI_ERROR_OVR               (0x00000004U)   /*!< OVR error                              */
//        #define HAL_SPI_ERROR_FRE               (0x00000008U)   /*!< FRE error                              */
//        #define HAL_SPI_ERROR_DMA               (0x00000010U)   /*!< DMA transfer error                     */
//        #define HAL_SPI_ERROR_FLAG              (0x00000020U)   /*!< Error on RXNE/TXE/BSY Flag             */
//        #define HAL_SPI_ERROR_ABORT             (0x00000040U)   /*!< Error during SPI Abort procedure       */
        volatile uint32_t spi_data_register =  hspi2.Instance->DR; // To clear over run
        RK_LOGI(TAG, "HAL_SPI_ErrorCallback : error code [0x%02X], CS %s\r\n", 
                hspi->ErrorCode, 
                HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) ? "high" : "low"); // CS signal);
//        
//        while(!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12));
//        __disable_irq();
//        HAL_SPI_Abort(&hspi2);
//        __enable_irq();    
//        while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) != GPIO_PIN_SET) // CS signal
//        {
//        }
        
//    __disable_irq(); 
////    /* might not be necessary */

////    hspi2.hdmarx->XferCpltCallback = NULL;
//    HAL_SPI_Abort(&hspi2);
//    __enable_irq(); 
        

	}
}


void Assert_Is_NULL(void* value){
	if(value == NULL){
		app_error_handler(__FILE__, __LINE__, (int)ERR_ALLOCATE_MEMORY);
	}
}


/**
  * @brief  Perform the SDRAM exernal memory inialization sequence
  * @param  hsdram: SDRAM handle
  * @param  Command: Pointer to SDRAM command structure
  * @retval None
  */
static void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
  __IO uint32_t tmpmrd =0;
  /* Step 3:  Configure a clock configuration enable command */
  Command->CommandMode 			 = FMC_SDRAM_CMD_CLK_ENABLE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

  /* Step 4: Insert 100 ms delay */
  HAL_Delay(100);
    
  /* Step 5: Configure a PALL (precharge all) command */ 
  Command->CommandMode 			 = FMC_SDRAM_CMD_PALL;
  Command->CommandTarget 	     = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);  
  
  /* Step 6 : Configure a Auto-Refresh command */ 
  Command->CommandMode 			 = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 4;
  Command->ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);
  
  /* Step 7: Program the external memory mode register */
    #define SDRAM_MODEREG_BURST_LENGTH_1             ((uint16_t)0x0000)
    #define SDRAM_MODEREG_BURST_LENGTH_2             ((uint16_t)0x0001)
    #define SDRAM_MODEREG_BURST_LENGTH_4             ((uint16_t)0x0002)
    #define SDRAM_MODEREG_BURST_LENGTH_8             ((uint16_t)0x0004)
    #define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL      ((uint16_t)0x0000)
    #define SDRAM_MODEREG_BURST_TYPE_INTERLEAVED     ((uint16_t)0x0008)
    #define SDRAM_MODEREG_CAS_LATENCY_2              ((uint16_t)0x0020)
    #define SDRAM_MODEREG_CAS_LATENCY_3              ((uint16_t)0x0030)
    #define SDRAM_MODEREG_OPERATING_MODE_STANDARD    ((uint16_t)0x0000)
    #define SDRAM_MODEREG_WRITEBURST_MODE_PROGRAMMED ((uint16_t)0x0000) 
    #define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE     ((uint16_t)0x0200) 
    
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_2          |
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
                     SDRAM_MODEREG_CAS_LATENCY_3           |
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
                     
//    Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  Command->CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command->CommandTarget 		 = FMC_SDRAM_CMD_TARGET_BANK1;
  Command->AutoRefreshNumber 	 = 1;
  Command->ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);
  
  /* Step 8: Set the refresh rate counter */
  /* (15.62 us x Freq) - 20 */
  /* Set the device refresh counter */
  HAL_SDRAM_ProgramRefreshRate(hsdram, REFRESH_COUNT); 
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void app_error_handler(char * file, int line, int error_code) {
    RK_LOGE(TAG, "Assert [%d] at file %s, line %d\r\n", error_code, file, line);
    __disable_irq();
    __ASM volatile("BKPT #01");   // insert breakpoint here
	while(true){
	};
}

void vApplicationMallocFailedHook( void )
{
    RK_LOGE(TAG, "Malloc failed\r\n");
    __disable_irq();
    __ASM volatile("BKPT #01");   // insert breakpoint here
	while(true){
	};
}

static uint32_t m_last_seen_esp32 = 0;
static void on_esp32_dead()
{
    // TODO : Long time no see esp32
    // May be esp32 could not boot up
}

static void esp32_on_data_callback(uint8_t id, uint8_t len, uint8_t* data)
{
    RK_LOGI(TAG, "Recv id %d, size %d bytes\r\n", id, len);
    switch (id)
    {
        case RK_ID_SEND_AUDIO_TO_CLOUD:
            break;
        
        case RK_ID_PING: 
            break;

        case RK_ID_PONG: 
             RK_LOGI(TAG, "Pong\r\n");
            break;
        
        case RK_ID_MOTOR_CTRL:
					{  
						memset(Motor_Command_TX, 0, CMD_UART_MAX_LENGTH);
						*Motor_Command_TX = *data;
						motor_send(Motor_Command_TX, CMD_UART_MAX_LENGTH);
					}
            break;
        
        default:
            break;
    }
}

static void task_uart(void * argument)
{
    vTaskDelay(1);
    sdram_test_loop();
    
    rk_serial_protocol_config_t conf;
    conf.tx_byte = NULL;   
    conf.tx_frame = esp32_communication_uart_tx;
    conf.callback = esp32_on_data_callback;
    rk_serial_protocol_init(&conf);
    
    esp32_uart_communication_initialize();
    uint32_t ping_cnt = xTaskGetTickCount();
//    uint32_t m_last_seen_esp32 = xTaskGetTickCount();
    for(;;)
    {
        bool seen_esp32 = false;
        while(esp32_communication_get_bytes_availble())
        {
            rk_serial_protocol_rx_byte(esp32_communication_get_bytes());
            seen_esp32 = true;
        }
        
        if (seen_esp32)
        {
            m_last_seen_esp32 = xTaskGetTickCount();
        }
        else
        {
            if (xTaskGetTickCount() - m_last_seen_esp32 > ESP32_KEEP_ALIVE_TIMEOUT)
            {
                on_esp32_dead();
                m_last_seen_esp32 = xTaskGetTickCount();
            }
        }
				
        if(vad_result == VAD_DETECT_VOICE){
            vTaskDelay(100);
            rk_serial_tx_frame(RK_ID_START_RECORDING, 0, NULL);
            vad_result = VAD_NO_CHANGE;
        }else if(vad_result == VAD_DETECT_SILIENT){
            vTaskDelay(100);
            rk_serial_tx_frame(RK_ID_START_SENDING_AUDIO, 0, NULL);
            vad_result = VAD_NO_CHANGE;
        }
        
//        if (xTaskGetTickCount() - ping_cnt > PING_PONG_TIMEOUT)
//        {
//            // Do ping
//            // @Note : max frame size is 115 bytes
//            rk_serial_tx_frame(RK_ID_PING, 0, NULL);
//            ping_cnt = xTaskGetTickCount();
//            RK_LOGI(TAG, "Ping\r\n");
//        }

        vTaskDelay(1);
    }
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
    #define SPI_DMA_RX_WAIT_TIME 50
    RK_LOG_INIT();
    RK_LOGI(TAG, "Build %s\r\n", __DATE__);

    esp32_spi_communication_initialize();
    Direction_Init_Communication();
    motor_init_communication();
    Lcd_Init_Communication();

    /* Infinite loop */
    uint32_t audio_size;
		vTaskDelay(3000);
    uint32_t default_task_tick = xTaskGetTickCount();
    
    Esp32_Change_Spi_Cs(GPIO_PIN_SET);
    vTaskDelay(30);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4))
    {
        RK_LOGI(TAG, "Slave ready\r\n");
        esp32_notify_slave_ready(false);
    }
            
     uint32_t start_time = xTaskGetTickCount();
    for(;;)
    {
        vTaskDelayUntil(&default_task_tick, SPI_DMA_RX_WAIT_TIME);
        uint8_t* audio_data = esp32_get_audio_data(10000);
        if(*audio_data == CMD_AUDIO_DATA){
            audio_size = *((uint32_t*)(audio_data + HEADER_OFFSET_REQUEST_DATA_SIZE));
            if(audio_size < SAMPLE_1_SEC*2){
                memmove(audio_buffer_1s, ((uint8_t*)audio_buffer_1s) + audio_size, SAMPLE_1_SEC * 2 - audio_size);
                memcpy(((uint8_t*)audio_buffer_1s) + SAMPLE_1_SEC * 2 - audio_size, audio_data + HEADER_OFFSET_REQUEST_DATA, audio_size);
            }
        }
//        else{
//            RK_LOGI(TAG, "no data\r\n");
//        }
        if(xTaskGetTickCount() - start_time > 200){
            voice_command_fill_buffer(audio_buffer_1s, SAMPLE_1_SEC *2);
            Run_KWS();
            start_time = xTaskGetTickCount();
        }
    }
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    RK_LOGE(TAG, "Error_Handler\r\n");
    __disable_irq();
    __ASM volatile("BKPT #01");   // insert breakpoint here
    while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
    RK_LOGE(TAG, "Assert failed in file %s, line %d\r\n", (char*)file, line);
    __disable_irq();
    __ASM volatile("BKPT #01");   // insert breakpoint here
    while(1);
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
