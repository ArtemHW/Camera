/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAMERA_ADDRESS_WRITE (0x60)
#define CAMERA_ADDRESS_READ (0x61)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

osThreadId Camera_SCCBHandle;
/* USER CODE BEGIN PV */
const unsigned char OV2640_JPEG_INIT[][2] = { { 0xff, 0x00 }, { 0x2c, 0xff }, {
		0x2e, 0xdf }, { 0xff, 0x01 }, { 0x3c, 0x32 }, { 0x11, 0x00 }, { 0x09,
		0x02 }, { 0x04, 0x28 }, { 0x13, 0xe5 }, { 0x14, 0x48 }, { 0x2c, 0x0c },
		{ 0x33, 0x78 }, { 0x3a, 0x33 }, { 0x3b, 0xfB }, { 0x3e, 0x00 }, { 0x43,
				0x11 }, { 0x16, 0x10 }, { 0x39, 0x92 }, { 0x35, 0xda }, { 0x22,
				0x1a }, { 0x37, 0xc3 }, { 0x23, 0x00 }, { 0x34, 0xc0 }, { 0x36,
				0x1a }, { 0x06, 0x88 }, { 0x07, 0xc0 }, { 0x0d, 0x87 }, { 0x0e,
				0x41 }, { 0x4c, 0x00 }, { 0x48, 0x00 }, { 0x5B, 0x00 }, { 0x42,
				0x03 }, { 0x4a, 0x81 }, { 0x21, 0x99 }, { 0x24, 0x40 }, { 0x25,
				0x38 }, { 0x26, 0x82 }, { 0x5c, 0x00 }, { 0x63, 0x00 }, { 0x61,
				0x70 }, { 0x62, 0x80 }, { 0x7c, 0x05 }, { 0x20, 0x80 }, { 0x28,
				0x30 }, { 0x6c, 0x00 }, { 0x6d, 0x80 }, { 0x6e, 0x00 }, { 0x70,
				0x02 }, { 0x71, 0x94 }, { 0x73, 0xc1 }, { 0x12, 0x40 }, { 0x17,
				0x11 }, { 0x18, 0x43 }, { 0x19, 0x00 }, { 0x1a, 0x4b }, { 0x32,
				0x09 }, { 0x37, 0xc0 }, { 0x4f, 0x60 }, { 0x50, 0xa8 }, { 0x6d,
				0x00 }, { 0x3d, 0x38 }, { 0x46, 0x3f }, { 0x4f, 0x60 }, { 0x0c,
				0x3c }, { 0xff, 0x00 }, { 0xe5, 0x7f }, { 0xf9, 0xc0 }, { 0x41,
				0x24 }, { 0xe0, 0x14 }, { 0x76, 0xff }, { 0x33, 0xa0 }, { 0x42,
				0x20 }, { 0x43, 0x18 }, { 0x4c, 0x00 }, { 0x87, 0xd5 }, { 0x88,
				0x3f }, { 0xd7, 0x03 }, { 0xd9, 0x10 }, { 0xd3, 0x82 }, { 0xc8,
				0x08 }, { 0xc9, 0x80 }, { 0x7c, 0x00 }, { 0x7d, 0x00 }, { 0x7c,
				0x03 }, { 0x7d, 0x48 }, { 0x7d, 0x48 }, { 0x7c, 0x08 }, { 0x7d,
				0x20 }, { 0x7d, 0x10 }, { 0x7d, 0x0e }, { 0x90, 0x00 }, { 0x91,
				0x0e }, { 0x91, 0x1a }, { 0x91, 0x31 }, { 0x91, 0x5a }, { 0x91,
				0x69 }, { 0x91, 0x75 }, { 0x91, 0x7e }, { 0x91, 0x88 }, { 0x91,
				0x8f }, { 0x91, 0x96 }, { 0x91, 0xa3 }, { 0x91, 0xaf }, { 0x91,
				0xc4 }, { 0x91, 0xd7 }, { 0x91, 0xe8 }, { 0x91, 0x20 }, { 0x92,
				0x00 }, { 0x93, 0x06 }, { 0x93, 0xe3 }, { 0x93, 0x05 }, { 0x93,
				0x05 }, { 0x93, 0x00 }, { 0x93, 0x04 }, { 0x93, 0x00 }, { 0x93,
				0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93,
				0x00 }, { 0x93, 0x00 }, { 0x96, 0x00 }, { 0x97, 0x08 }, { 0x97,
				0x19 }, { 0x97, 0x02 }, { 0x97, 0x0c }, { 0x97, 0x24 }, { 0x97,
				0x30 }, { 0x97, 0x28 }, { 0x97, 0x26 }, { 0x97, 0x02 }, { 0x97,
				0x98 }, { 0x97, 0x80 }, { 0x97, 0x00 }, { 0x97, 0x00 }, { 0xc3,
				0xed }, { 0xa4, 0x00 }, { 0xa8, 0x00 }, { 0xc5, 0x11 }, { 0xc6,
				0x51 }, { 0xbf, 0x80 }, { 0xc7, 0x10 }, { 0xb6, 0x66 }, { 0xb8,
				0xA5 }, { 0xb7, 0x64 }, { 0xb9, 0x7C }, { 0xb3, 0xaf }, { 0xb4,
				0x97 }, { 0xb5, 0xFF }, { 0xb0, 0xC5 }, { 0xb1, 0x94 }, { 0xb2,
				0x0f }, { 0xc4, 0x5c }, { 0xc0, 0x64 }, { 0xc1, 0x4B }, { 0x8c,
				0x00 }, { 0x86, 0x3D }, { 0x50, 0x00 }, { 0x51, 0xC8 }, { 0x52,
				0x96 }, { 0x53, 0x00 }, { 0x54, 0x00 }, { 0x55, 0x00 }, { 0x5a,
				0xC8 }, { 0x5b, 0x96 }, { 0x5c, 0x00 }, { 0xd3, 0x00 }, { 0xc3,
				0xed }, { 0x7f, 0x00 }, { 0xda, 0x00 }, { 0xe5, 0x1f }, { 0xe1,
				0x67 }, { 0xe0, 0x00 }, { 0xdd, 0x7f }, { 0x05, 0x00 }, { 0x12,
				0x40 }, { 0xd3, 0x04 }, { 0xc0, 0x16 }, { 0xC1, 0x12 }, { 0x8c,
				0x00 }, { 0x86, 0x3d }, { 0x50, 0x00 }, { 0x51, 0x2C }, { 0x52,
				0x24 }, { 0x53, 0x00 }, { 0x54, 0x00 }, { 0x55, 0x00 }, { 0x5A,
				0x2c }, { 0x5b, 0x24 }, { 0x5c, 0x00 }, { 0xff, 0xff }, };

const unsigned char OV2640_YUV422[][2] = { { 0xFF, 0x00 }, { 0x05, 0x00 }, {
		0xDA, 0x10 }, { 0xD7, 0x03 }, { 0xDF, 0x00 }, { 0x33, 0x80 }, { 0x3C,
		0x40 }, { 0xe1, 0x77 }, { 0x00, 0x00 }, { 0xff, 0xff }, };

const unsigned char OV2640_JPEG[][2] = { { 0xe0, 0x14 }, { 0xe1, 0x77 }, { 0xe5,
		0x1f }, { 0xd7, 0x03 }, { 0xda, 0x10 }, { 0xe0, 0x00 }, { 0xFF, 0x01 },
		{ 0x04, 0x08 }, { 0xff, 0xff }, };

const unsigned char OV2640_160x120_JPEG[][2] = { { 0xFF, 0x01 }, { 0x12, 0x40 },
		{ 0x17, 0x11 }, { 0x18, 0x43 }, { 0x19, 0x00 }, { 0x1a, 0x4b }, { 0x32,
				0x09 }, { 0x4f, 0xca }, { 0x50, 0xa8 }, { 0x5a, 0x23 }, { 0x6d,
				0x00 }, { 0x39, 0x12 }, { 0x35, 0xda }, { 0x22, 0x1a }, { 0x37,
				0xc3 }, { 0x23, 0x00 }, { 0x34, 0xc0 }, { 0x36, 0x1a }, { 0x06,
				0x88 }, { 0x07, 0xc0 }, { 0x0d, 0x87 }, { 0x0e, 0x41 }, { 0x4c,
				0x00 }, { 0xFF, 0x00 }, { 0xe0, 0x04 }, { 0xc0, 0x64 }, { 0xc1,
				0x4b }, { 0x86, 0x35 }, { 0x50, 0x92 }, { 0x51, 0xc8 }, { 0x52,
				0x96 }, { 0x53, 0x00 }, { 0x54, 0x00 }, { 0x55, 0x00 }, { 0x57,
				0x00 }, { 0x5a, 0x2c }, { 0x5b, 0x24 }, { 0x5c, 0x00 }, { 0xe0,
				0x00 }, { 0xff, 0xff } };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void cameraSCCB(void const * argument);

/* USER CODE BEGIN PFP */
void OV2640_ResolutionConfiguration(short opt) {

	uint8_t data_to_send = 0;
	OV2640_Configuration(OV2640_JPEG_INIT);
	OV2640_Configuration(OV2640_YUV422);
	OV2640_Configuration(OV2640_JPEG);
	HAL_Delay(10);
	//SCCB_Write(0xff, 0x01);
	data_to_send = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, CAMERA_ADDRESS_WRITE, 0xFF, 1, &data_to_send, 1, 1000);
	HAL_Delay(10);
	//SCCB_Write(0x15, 0x00);
	data_to_send = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, CAMERA_ADDRESS_WRITE, 0x15, 1, &data_to_send, 1, 1000);

	switch (opt) {
	case 0:
		OV2640_Configuration(OV2640_160x120_JPEG);
		break;
	}

}

void OV2640_Configuration(const unsigned char arr[][2]) {
	uint16_t i = 0;
	uint8_t reg_addr, data, data_read;
	uint8_t data_to_send = 0;
	while (1) {
		reg_addr = arr[i][0];
		data = arr[i][1];
		if (reg_addr == 0xff && data == 0xff) {
			break;
		}
		HAL_StatusTypeDef result = 4;
		HAL_I2C_DeInit(&hi2c1);
		HAL_Delay(10);
		HAL_I2C_Init(&hi2c1);
		HAL_Delay(10);
		//SCCB_Read(reg_addr, &data_read);
//		data_to_send = reg_addr;
//		__disable_irq();
//		HAL_I2C_Master_Transmit(&hi2c1, CAMERA_ADDRESS_WRITE, &data_to_send, 1, 1000);
//		HAL_I2C_Master_Receive(&hi2c1, CAMERA_ADDRESS_READ, &data_read, 1, 1000);
//		__enable_irq();
//		HAL_Delay(10);
		//SCCB_Write(reg_addr, data);
		//data_to_send = data;
		//__disable_irq();
		//HAL_I2C_Mem_Write(&hi2c1, CAMERA_ADDRESS_WRITE, reg_addr, 1, &data_to_send, 1, 1000);
		uint8_t data_to_send2[] = {reg_addr,  data};
		result = 4;
		do{
			result = HAL_I2C_Master_Transmit(&hi2c1, CAMERA_ADDRESS_WRITE, &data_to_send2, 2, 1000);
			if(result != HAL_OK){
				HAL_I2C_DeInit(&hi2c1);
				HAL_Delay(10);
				HAL_I2C_Init(&hi2c1);
				HAL_Delay(10);
			}
		} while(result != HAL_OK);
		//__enable_irq();
		HAL_Delay(10);
		//SCCB_Read(reg_addr, &data_read);
		data_to_send = reg_addr;
		result = 4;
		//__disable_irq();
		do{
			result = HAL_I2C_Master_Transmit(&hi2c1, CAMERA_ADDRESS_WRITE, &data_to_send, 1, 1000);
			if(result != HAL_OK){
				HAL_I2C_DeInit(&hi2c1);
				HAL_Delay(10);
				HAL_I2C_Init(&hi2c1);
				HAL_Delay(10);
			}
		}while(result != HAL_OK);
		result = 4;
		do{
			result = HAL_I2C_Master_Receive(&hi2c1, CAMERA_ADDRESS_READ, &data_read, 1, 1000);
			if(result != HAL_OK){
				HAL_I2C_DeInit(&hi2c1);
				HAL_Delay(10);
				HAL_I2C_Init(&hi2c1);
				HAL_Delay(10);
			}
		}while(result != HAL_OK);
		//__enable_irq();
		HAL_Delay(10);
		if (data != data_read) {
			continue;
		}
		i++;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	uint8_t data_to_send = 0;

	// Hardware reset
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(100);
	// Software reset: reset all registers to default values
//	SCCB_Write(0xff, 0x01);
	data_to_send = 0x1;
	HAL_I2C_Mem_Write(&hi2c1, CAMERA_ADDRESS_WRITE, 0xFF, 1, &data_to_send, 1, 1000);
	data_to_send = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, CAMERA_ADDRESS_WRITE, 0x12, 1, &data_to_send, 1, 1000);
//	SCCB_Write(0x12, 0x80);
	HAL_Delay(100);

	data_to_send = 1;
	HAL_I2C_Mem_Write(&hi2c1, CAMERA_ADDRESS_WRITE, 0xFF, 1, &data_to_send, 1, 1000);

	data_to_send = 0x1D; //Manufacturer ID Byte – Low
	HAL_I2C_Master_Transmit(&hi2c1, CAMERA_ADDRESS_WRITE, &data_to_send, 1, 1000);
	uint8_t read_data = 0;
	HAL_I2C_Master_Receive(&hi2c1, CAMERA_ADDRESS_READ, &read_data, 1, 1000);

	HAL_Delay(10);
	OV2640_ResolutionConfiguration(0);
	HAL_Delay(10);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
//	vTaskDelay(1);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
//	vTaskDelay(1);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
//	vTaskDelay(500);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);

//	GPIOC->ODR |= GPIO_ODR_OD11;
//	HAL_Delay(1);
//	GPIOC->ODR &= ~GPIO_ODR_OD11;
//	HAL_Delay(1);
//	GPIOC->ODR |= GPIO_ODR_OD12;
//	HAL_Delay(500);
//	GPIOC->ODR &= ~GPIO_ODR_OD12;
//	HAL_Delay(1);

	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

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
  /* definition and creation of Camera_SCCB */
  osThreadDef(Camera_SCCB, cameraSCCB, osPriorityNormal, 0, 256);
  Camera_SCCBHandle = osThreadCreate(osThread(Camera_SCCB), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 64;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_cameraSCCB */
/**
  * @brief  Function implementing the Camera_SCCB thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_cameraSCCB */
void cameraSCCB(void const * argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  __disable_irq();
  while (1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
