/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "mpu9250.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */ 
#define SIZE_GYRO 6
#define SIZE_ACCEL 6
#define MPU_ADDR (0x68 << 1)
#define MPU_PWR_MGMT_1 (0x6B)
#define MPU_SMPLRT_DIV (0x19)
#define MPU_CONFIG (0x1A)
#define MPU_GYRO_CONFIG (0x1B)
#define MPU_ACCEL_CONFIG (0x1C)
#define ACCEL_XOUT_H (0x3B)
#define GYRO_XOUT_H (0x43)
#define TEMP_OUT_H (0x41)

uint16_t acc_x = 0;
uint8_t buff[14] = {0};

#define MPU_IS_CONNECT(hi2c, addr) HAL_I2C_IsDeviceReady(&hi2c,addr, 5, 200)
void mpu_init(){
	// check connect imu
	if(MPU_IS_CONNECT(hi2c1, MPU_ADDR) != HAL_OK){
		uint8_t cnt = 50;
		while(cnt--){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			HAL_Delay(20);
		}
	}
	
	// 
	uint8_t data[2];
	data[0] = MPU_PWR_MGMT_1;
	data[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR, data, sizeof(data), 1000);

	//
	data[0] = MPU_SMPLRT_DIV;
	data[1] = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR, data, sizeof(data), 1000);
	
	// Set Gyroscopic configuration in GYRO_CONFIG Register
	data[0] = MPU_GYRO_CONFIG;
	data[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR, data, sizeof(data), 1000);
	
  // Set accelerometer configuration in ACCEL_CONFIG Register
	data[0] = MPU_ACCEL_CONFIG;
	data[1] = 0; // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
	HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR, data, sizeof(data), 1000);
}


uint16_t accel_x_raw, accel_y_raw, accel_z_raw;
float ax, ay, az;
void mpu_read_accel(){
	uint8_t data_accel[6] = {0};
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, ACCEL_XOUT_H, 1, data_accel, SIZE_ACCEL, 100);
//	HAL_I2C_Master_Transmit(&hi2c1, MPU_ADDR, (uint8_t *)ACCEL_XOUT_H, 1, 100);
//	HAL_Delay(1);
//	HAL_I2C_Master_Receive(&hi2c1, MPU_ADDR, data_accel, 6, 100);
	
	// combine data high and low to form 16 bit accelerometer data for x, y, z axis
	accel_x_raw = (data_accel[0] << 8) | data_accel[1];
	accel_y_raw = (data_accel[2] << 8) | data_accel[3];
	accel_z_raw = (data_accel[4] << 8) | data_accel[5];
	
	// convert data into proper ‘g’ format
	ax = accel_x_raw / 16384.0;
	ay = accel_y_raw / 16384.0;
	az = accel_z_raw / 16384.0;
}


uint16_t gryo_x_raw, gryo_y_raw, gryo_z_raw;
float gx, gy, gz;

void mpu_read_gyro(){
	uint8_t data_gyro[6] = {0};
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, GYRO_XOUT_H, 1, data_gyro, SIZE_GYRO, 100);
	gryo_x_raw = (data_gyro[0] << 8) | data_gyro[1];
	gryo_y_raw = (data_gyro[2] << 8) | data_gyro[3];
	gryo_z_raw = (data_gyro[4] << 8) | data_gyro[5];
	
	gx = gryo_x_raw / 131.0;
	gy = gryo_y_raw / 131.0;
	gz = gryo_z_raw / 131.0;
}

float temp;
void mpu_read_temp(){
	uint8_t data_temp[2] = {0};
	HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, TEMP_OUT_H, 1, data_temp, 2, 100); 
	
	uint16_t data = (data_temp[0] << 8) | data_temp[1];
	temp = data / 340.0 + 36.53;
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	mpu_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		mpu_read_accel();
		mpu_read_gyro();
		mpu_read_temp();
		HAL_Delay(1000);

		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		//HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
