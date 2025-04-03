/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_ble.h
  * @author  MCD Application Team
  * @brief   Header for ble application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BLE_H
#define APP_BLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hci_tl.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/

typedef enum
{
  APP_BLE_Limited,
  APP_BLE_Full,
} APP_BLE_InitMode_t;

/* USER CODE BEGIN ET */
typedef enum
{
  HID_IDLE,
  HID_FAST_ADV,
  HID_LP_ADV,
  HID_LP_CONNECTING,
  HID_CONNECTED_SERVER,
  HID_CONNECTED_CLIENT
} HID_ConnStatus_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ---------------------------------------------*/
void APP_BLE_Init(void);


/* USER CODE BEGIN EF */
void APP_BLE_Key_Button1_Action(void);
void APP_BLE_Key_Button2_Action(void);
void APP_BLE_Key_Button3_Action(void);

HID_ConnStatus_t get_conn_status(uint32_t index);
/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /* APP_BLE_H */
