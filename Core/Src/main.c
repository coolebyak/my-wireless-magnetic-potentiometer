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
#include "dma.h"
#include "i2c.h"
#include "ipcc.h"
#include "usart.h"
#include "memorymap.h"
#include "rf.h"
#include "rng.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "usbd_hid.h"
#include "app_ble.h"
#include "hids_app.h"
#include "stm32_seq.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile HID_MouseReport_t mr = {0};
AS5600_TypeDef *ams = NULL;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline int32_t filter(int32_t in, int32_t prev, int32_t alpha1000){
	const uint16_t DIVIDER = 1000;
	int32_t result = 0;
	if (alpha1000 > 1000) return result;
	result = (alpha1000*in + (DIVIDER-alpha1000)*prev)/DIVIDER;
	return result;
}
void foo(void){
	static int8_t angle_report = 0;
	static HID_ConnStatus_t conn_status = HID_IDLE;
	if (ams->InitStatus != 0){
		AS5600_GetRawAngle(ams, &ams->angle_var);
		angle_report = 0;
		ams->angle_diff = ams->angle_var - ams->angle_prev;
		if (abs(ams->angle_diff) > 1){
			ams->angle_prev = ams->angle_var;

			if (ams->angle_diff >  2048) {
				ams->angle_diff -= 4095;
			}
			if (ams->angle_diff < -2048) {
				ams->angle_diff += 4095;
			}

			static const uint16_t divider = 8;
			if (ams->angle_diff > divider) ams->angle_diff /= divider;


			if (abs(ams->angle_diff) > 0) HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

			if (ams->angle_diff >  127) ams->angle_diff =  127;
			if (ams->angle_diff < -127) ams->angle_diff = -127;

			//ams->angle_filt = filter(ams->angle_diff, ams->angle_filt, 900);

			angle_report = ams->angle_diff;

		} else {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		}

		mr.wheel = angle_report;


		conn_status = get_conn_status(0);
		if (conn_status == HID_CONNECTED_SERVER || conn_status == HID_CONNECTED_CLIENT){
			UTIL_SEQ_SetTask(1<<CFG_TASK_HID_UPDATE_REQ_ID, CFG_SCH_PRIO_0);
		} else {
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mr, sizeof(mr));
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim16) {
		//foo();
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
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USB_Device_Init();
  MX_TIM16_Init();
  MX_RNG_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */
  LL_HSEM_1StepLock( HSEM, CFG_HW_CLK48_CONFIG_SEMID);

  ams = AS5600_New_Static();
  ams->i2cHandle = &hi2c1;
  ams->i2cAddr = AS5600_SLAVE_ADDRESS<<1;
  ams->DirPin = Dir_pin_Pin;
  ams->DirPort = Dir_pin_GPIO_Port;
  AS5600_Init(ams);
  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	  static uint32_t tick = 0;
	  static uint32_t tick_now = 0;
//	  uint16_t angle;
//	  HAL_Delay(500);
//	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    tick_now = HAL_GetTick();
    if(tick_now >= tick){
    	tick = tick_now + 10;
//    	foo();
//    	//usb-hid report
//    	HID_ConnStatus_t tmp = get_conn_status(0);
//    	if (tmp == HID_CONNECTED_SERVER || tmp == HID_CONNECTED_CLIENT){
//    		UTIL_SEQ_SetTask(1<<CFG_TASK_HID_UPDATE_REQ_ID, CFG_SCH_PRIO_0);
//    	} else {
//    		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&mr, sizeof(mr));
//    	}
//
//    	if(0){
//    		HIDSAPP_Profile_UpdateChar();
//    	}
    }
    /* USER CODE END WHILE */
    MX_APPE_Process();

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
