/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myFlash.h"
#include "myEncoder.h"
#include "myUart.h"

#include "queue/queue.h"
#include "modbus_master/modbus_master.h"
#include "modbus_slave/modbus_slave.h"
#include <stdlib.h>
#include "wifi_command/wifi_command.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
Encoder_t enc2;
UartReceiver_t uart_rx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define M_LENGTH
//#define MODBUS_MASTER 	//uncomment để bật chế đô modbus slave


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t PPR 	= 1000;
float DIA	 	= 0.25f;
uint32_t TIME 	= 100;
int64_t pulse_t = 0;
float rpm;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
// ------------------- Printf --------------------------
#ifdef __GNUC__
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif
{
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

#ifndef M_LENGTH
void update_RPM() {
	static uint32_t last_log_time = 0;
	uint32_t now = HAL_GetTick();

	if (now - last_log_time >= TIME) {
		rpm = Encoder_GetRPM(&enc2);
//			printf("RPM: %.2f\r\n", rpm);
		last_log_time = now;
	}
}
#endif

void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart) {
	printf("HAL_UART_IDLE_Callback!\n");
	if (huart->Instance == USART3) {
		uint16_t len = UART_RX_BUFFER_SIZE
				- __HAL_DMA_GET_COUNTER(huart->hdmarx);
		if (len > 0 && len <= MODBUS_FRAME_MAX_LEN) {
			queue_frame_t frame;
			memcpy(frame.data, uart_rx_buffer, len);
			frame.len = len;

			if (!queue_push(&frame)) {
				printf("⚠️ Queue full- Remove frame!\r\n");
			}
		}

		// Reset DMA
		HAL_UART_DMAStop(huart);
		HAL_UART_Receive_DMA(huart, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	}
}


////////////////////// Dùng cái này nếu stm32 là MODBUS SLAVE /////////////
#define SLAVE_ID 0x01
uint16_t holding_regs[10];
volatile uint32_t encoder_pulses = 0;
volatile uint32_t distance_mm = 0;
void on_write_single_register(uint16_t addr, uint16_t value) {
	switch (addr) {
	case 0:
		holding_regs[0] = value;
		PPR = value;
		myFlash_Write(FLASH_PAGE_PPR, PPR);
		break; // số xung
	case 1:
		holding_regs[1] = value;
		DIA = (float)value / 1000.0;
		myFlash_Write(FLASH_PAGE_DIA, *(uint32_t*)&DIA);
		break; // đường kính (mm)
	case 2:
		holding_regs[2] = value;
		TIME = value;
		myFlash_Write(FLASH_PAGE_TIME, TIME);
		break; // thời gian lấy mẫu (ms)
	default:
		break;
	}
	Encoder_Init(&enc2, &htim2, (PPR), DIA, TIME);
}
void on_write_multiple_registers(uint16_t addr, const uint16_t *values,
		uint16_t quantity) {
	printf("Master write multi registers!\n");
}
void modbus_slave_setup() {
	modbus_slave_config_t slave_cfg = { .id = 1, .holding_registers =
			holding_regs, .holding_register_count = 10,
			.on_write_single_register = on_write_single_register,
			.on_write_multiple_registers = on_write_multiple_registers };

	modbus_slave_init(&huart3, &slave_cfg);
	holding_regs[0] = PPR;  // số xung
	holding_regs[1] = DIA*1000;  // đường kính (mm)
	holding_regs[2] = TIME; // thời gian lấy mẫu(ms)
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t len_val;
float dia_val;
#ifdef M_LENGTH
float length = 0;
#endif
uint8_t test = 0;
uint8_t uart_byte;
uint32_t parity = 0;
uint32_t c_uart = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
//    	HAL_UART_Transmit(&huart1, "hello", 8, 100);

        HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
        MyUart_OnRx(&uart_rx, uart_byte);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART3) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_TIM2_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // ----------------- UART3 -----------------------------
  HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  modbus_slave_setup();

  // ----------------- UART1 -----------------------------
  MyUart_Init(&uart_rx, &huart1);
  HAL_UART_Receive_IT(&huart1, &uart_byte, 1);

  // ----------------- IWDG -------------------------------
  HAL_IWDG_Init(&hiwdg);

  // ------------------ FLASH ----------------------------
  if(myFlash_Read(FLASH_PAGE_PPR) != 0xFFFFFFFF)
	  PPR = myFlash_Read(FLASH_PAGE_PPR);
  if(myFlash_Read(FLASH_PAGE_DIA) != 0xFFFFFFFF)
	  DIA = *(float*)&(uint32_t){ myFlash_Read(FLASH_PAGE_DIA)};
  if(myFlash_Read(FLASH_PAGE_TIME) != 0xFFFFFFFF)
	  TIME = myFlash_Read(FLASH_PAGE_TIME);
  if(myFlash_Read(FLASH_PAGE_BAUD) != 0xFFFFFFFF)
  {
	  huart1.Init.BaudRate = myFlash_Read(FLASH_PAGE_BAUD);
	  huart3.Init.BaudRate = myFlash_Read(FLASH_PAGE_BAUD);
	  HAL_UART_DeInit(&huart1);
	  HAL_UART_Init(&huart1);
	  HAL_UART_DeInit(&huart3);
	  HAL_UART_Init(&huart3);
	  // ----------------- UART3 -----------------------------
	  HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	  modbus_slave_setup();

	  // ----------------- UART1 -----------------------------
	  MyUart_Init(&uart_rx, &huart1);
	  HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
  }
  if(myFlash_Read(FLASH_PAGE_PARITY) != 0xFFFFFFFF)
  {
	  parity = myFlash_Read(FLASH_PAGE_PARITY);
	  switch(parity) {
	  case 0:
		  huart1.Init.Parity = UART_PARITY_NONE;
		  huart1.Init.WordLength = UART_WORDLENGTH_8B;
		  huart3.Init.Parity = UART_PARITY_NONE;
  		  huart3.Init.WordLength = UART_WORDLENGTH_8B;
		  break;
	  case 1:
		  huart1.Init.Parity = UART_PARITY_ODD;
		  huart1.Init.WordLength = UART_WORDLENGTH_9B;
		  huart3.Init.Parity = UART_PARITY_ODD;
  		  huart3.Init.WordLength = UART_WORDLENGTH_9B;
		  break;
	  case 2:
		  huart1.Init.Parity = UART_PARITY_EVEN;
		  huart1.Init.WordLength = UART_WORDLENGTH_9B;
		  huart3.Init.Parity = UART_PARITY_EVEN;
  		  huart3.Init.WordLength = UART_WORDLENGTH_9B;
		  break;
	  default:
		  break;
	  }
	  HAL_UART_DeInit(&huart1);
	  HAL_UART_Init(&huart1);
	  HAL_UART_DeInit(&huart3);
	  HAL_UART_Init(&huart3);
	  // ----------------- UART3 -----------------------------
	  HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	  modbus_slave_setup();

	  // ----------------- UART1 -----------------------------
	  MyUart_Init(&uart_rx, &huart1);
	  HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
  }
  len_val = myFlash_Read(FLASH_PAGE_LENGTH);

  Encoder_Init(&enc2, &htim2, PPR, DIA, TIME);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
// ------------------------------- Refresh Watchdog --------------------------
	  HAL_IWDG_Refresh(&hiwdg);

// ------------------------------- Receive UART ------------------------------
	  if (uart_rx.has_command) {
#ifdef M_LENGTH
	      myFlash_Write(FLASH_PAGE_LENGTH, *(uint32_t*)&length);
	      len_val = myFlash_Read(FLASH_PAGE_LENGTH);
#endif
	      if (strcmp(uart_rx.command, "PPR") == 0) {
	    	  myFlash_Write(FLASH_PAGE_PPR, atoi(uart_rx.value));
	    	  PPR = myFlash_Read(FLASH_PAGE_PPR);
	      } else if (strcmp(uart_rx.command, "DIA") == 0) {
	    	  dia_val = atof(uart_rx.value);  // chuyển chuỗi thành float
	    	  myFlash_Write(FLASH_PAGE_DIA, *(uint32_t*)&dia_val);
	    	  DIA = *(float*)&(uint32_t){ myFlash_Read(FLASH_PAGE_DIA) };
	      } else if (strcmp(uart_rx.command, "TIME") == 0) {
	    	  myFlash_Write(FLASH_PAGE_TIME, atoi(uart_rx.value));
	    	  TIME = myFlash_Read(FLASH_PAGE_TIME);
	      } else if(strcmp(uart_rx.command, "BAUD") == 0) {
	    	  myFlash_Write(FLASH_PAGE_BAUD, atoi(uart_rx.value));
	    	  huart1.Init.BaudRate = atoi(uart_rx.value);
	    	  huart3.Init.BaudRate = atoi(uart_rx.value);

//	    	  huart1.Init.BaudRate = myFlash_Read(FLASH_PAGE_BAUD);
//	    	  huart3.Init.BaudRate = myFlash_Read(FLASH_PAGE_BAUD);
	    	  HAL_UART_DeInit(&huart1);
	    	  HAL_UART_Init(&huart1);
	    	  HAL_UART_DeInit(&huart3);
	    	  HAL_UART_Init(&huart3);
	    	  // ----------------- UART3 -----------------------------
	    	  HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	    	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	    	  modbus_slave_setup();

	    	  // ----------------- UART1 -----------------------------
	    	  MyUart_Init(&uart_rx, &huart1);
	    	  HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
	      } else if(strcmp(uart_rx.command, "PARITY") == 0) {
	    	  myFlash_Write(FLASH_PAGE_PARITY, atoi(uart_rx.value));
	    	  parity = myFlash_Read(FLASH_PAGE_PARITY);
	    	  switch(parity) {
	    	  case 0:
	    		  huart1.Init.Parity = UART_PARITY_NONE;
	    		  huart1.Init.WordLength = UART_WORDLENGTH_8B;
	    		  huart3.Init.Parity = UART_PARITY_NONE;
  	    		  huart3.Init.WordLength = UART_WORDLENGTH_8B;
	    		  break;
	    	  case 1:
	    		  huart1.Init.Parity = UART_PARITY_ODD;
	    		  huart1.Init.WordLength = UART_WORDLENGTH_9B;
	    		  huart3.Init.Parity = UART_PARITY_ODD;
  	    		  huart3.Init.WordLength = UART_WORDLENGTH_9B;
	    		  break;
	    	  case 2:
	    		  huart1.Init.Parity = UART_PARITY_EVEN;
	    		  huart1.Init.WordLength = UART_WORDLENGTH_9B;
	    		  huart3.Init.Parity = UART_PARITY_EVEN;
  	    		  huart3.Init.WordLength = UART_WORDLENGTH_9B;
	    		  break;
	    	  }
	    	  HAL_UART_DeInit(&huart1);
	    	  HAL_UART_Init(&huart1);
	    	  HAL_UART_DeInit(&huart3);
	    	  HAL_UART_Init(&huart3);
	    	  // ----------------- UART3 -----------------------------
	    	  HAL_UART_Receive_DMA(&huart3, uart_rx_buffer, UART_RX_BUFFER_SIZE);
	    	  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	    	  modbus_slave_setup();

	    	  // ----------------- UART1 -----------------------------
	    	  MyUart_Init(&uart_rx, &huart1);
	    	  HAL_UART_Receive_IT(&huart1, &uart_byte, 1);
	      }
	      Encoder_Init(&enc2, &htim2, (PPR), DIA, TIME);
	      uart_rx.has_command = 0;
	      uart_rx.command[0] = '\0';
	      uart_rx.value[0] = '\0';
	  }
		holding_regs[0] = PPR;  // số xung
		holding_regs[1] = DIA*1000;  // đường kính (mm)
		holding_regs[2] = TIME; // thời gian lấy mẫu(ms)

	  pulse_t = Encoder_GetPulse(&enc2);
// -------------------------- Update Encoder ---------------------
#ifndef M_LENGTH
	  update_RPM();
	  uint16_t *value_rpm = (uint16_t*) &rpm;
	  holding_regs[5] = value_rpm[1];
	  holding_regs[6] = value_rpm[0];
#else
	  if(len_val == 0xFFFFFFFF)
	  {
		  test = 1;
		  length = Encoder_GetLengthMeter(&enc2);
//		  uint256_from_float_mult(&myEncoder, &len_256, &sign);
	  }
	  else
	  {
		  test = 2;
//		  float signed_val = *(float*)&len_val;
//		  uint256_from_float_mult(&myEncoder, &len_256, &sign);

		  length = Encoder_GetLengthMeter(&enc2) + *(float*)&len_val;
	  }
	  uint16_t *value_length = (uint16_t*) &length;
	  holding_regs[3] = value_length[1];
	  holding_regs[4] = value_length[0];
#endif
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET)
	  {
#ifdef M_LENGTH
		  myFlash_Write(FLASH_PAGE_LENGTH, *(uint32_t*)&length);
#endif
		  test = 3;
		  while(1);
	  }
	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) // Erase all stored data in Flash memory
	  {
		  myFlash_ErasePage(FLASH_PAGE_LENGTH);
		  myFlash_ErasePage(FLASH_PAGE_DIA);
		  myFlash_ErasePage(FLASH_PAGE_PPR);
		  myFlash_ErasePage(FLASH_PAGE_TIME);
		  myFlash_ErasePage(FLASH_PAGE_BAUD);
		  myFlash_ErasePage(FLASH_PAGE_PARITY);
		  while(1);
	  }
	  queue_frame_t frame;
	  if (queue_pop(&frame)) {
		  printf("📥 Processing %d byte\n", frame.len);
		  modbus_slave_handle_frame(frame.data, frame.len);
	  }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 2499;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
