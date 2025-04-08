/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* HumanInterfaceDevice */
  uint8_t               Inputrep_Notification_Status;
  /* Device_Information */
  /* Battery */
  uint8_t               Bal_Notification_Status;
  /* Scan_Parameters */
  uint8_t               Scr_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* HumanInterfaceDevice */
static void Custom_Inputrep_Update_Char(void);
static void Custom_Inputrep_Send_Notification(void);
/* Device_Information */
/* Battery */
static void Custom_Bal_Update_Char(void);
static void Custom_Bal_Send_Notification(void);
/* Scan_Parameters */
static void Custom_Scr_Update_Char(void);
static void Custom_Scr_Send_Notification(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* HumanInterfaceDevice */
    case CUSTOM_STM_HII_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HII_READ_EVT */

      /* USER CODE END CUSTOM_STM_HII_READ_EVT */
      break;

    case CUSTOM_STM_HCP_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HCP_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_HCP_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_REM_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_REM_READ_EVT */

      /* USER CODE END CUSTOM_STM_REM_READ_EVT */
      break;

    case CUSTOM_STM_INPUTREP_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_INPUTREP_READ_EVT */

      /* USER CODE END CUSTOM_STM_INPUTREP_READ_EVT */
      break;

    case CUSTOM_STM_INPUTREP_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_INPUTREP_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_INPUTREP_WRITE_EVT */
      break;

    case CUSTOM_STM_INPUTREP_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_INPUTREP_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_INPUTREP_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_INPUTREP_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_INPUTREP_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_INPUTREP_NOTIFY_DISABLED_EVT */
      break;

    /* Device_Information */
    case CUSTOM_STM_MANS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MANS_READ_EVT */

      /* USER CODE END CUSTOM_STM_MANS_READ_EVT */
      break;

    case CUSTOM_STM_MONS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_MONS_READ_EVT */

      /* USER CODE END CUSTOM_STM_MONS_READ_EVT */
      break;

    case CUSTOM_STM_FRS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_FRS_READ_EVT */

      /* USER CODE END CUSTOM_STM_FRS_READ_EVT */
      break;

    case CUSTOM_STM_SRS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SRS_READ_EVT */

      /* USER CODE END CUSTOM_STM_SRS_READ_EVT */
      break;

    case CUSTOM_STM_PNI_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_PNI_READ_EVT */

      /* USER CODE END CUSTOM_STM_PNI_READ_EVT */
      break;

    /* Battery */
    case CUSTOM_STM_BAL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BAL_READ_EVT */

      /* USER CODE END CUSTOM_STM_BAL_READ_EVT */
      break;

    case CUSTOM_STM_BAL_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BAL_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_BAL_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_BAL_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_BAL_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_BAL_NOTIFY_DISABLED_EVT */
      break;

    /* Scan_Parameters */
    case CUSTOM_STM_SIW_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SIW_WRITE_NO_RESP_EVT */

      /* USER CODE END CUSTOM_STM_SIW_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_SCR_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SCR_NOTIFY_ENABLED_EVT */

      /* USER CODE END CUSTOM_STM_SCR_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_SCR_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_SCR_NOTIFY_DISABLED_EVT */

      /* USER CODE END CUSTOM_STM_SCR_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* HumanInterfaceDevice */
__USED void Custom_Inputrep_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Inputrep_UC_1*/

  /* USER CODE END Inputrep_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_INPUTREP, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Inputrep_UC_Last*/

  /* USER CODE END Inputrep_UC_Last*/
  return;
}

void Custom_Inputrep_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Inputrep_NS_1*/

  /* USER CODE END Inputrep_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_INPUTREP, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Inputrep_NS_Last*/

  /* USER CODE END Inputrep_NS_Last*/

  return;
}

/* Device_Information */
/* Battery */
__USED void Custom_Bal_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Bal_UC_1*/

  /* USER CODE END Bal_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BAL, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Bal_UC_Last*/

  /* USER CODE END Bal_UC_Last*/
  return;
}

void Custom_Bal_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Bal_NS_1*/

  /* USER CODE END Bal_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_BAL, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Bal_NS_Last*/

  /* USER CODE END Bal_NS_Last*/

  return;
}

/* Scan_Parameters */
__USED void Custom_Scr_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Scr_UC_1*/

  /* USER CODE END Scr_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SCR, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Scr_UC_Last*/

  /* USER CODE END Scr_UC_Last*/
  return;
}

void Custom_Scr_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Scr_NS_1*/

  /* USER CODE END Scr_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_SCR, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Scr_NS_Last*/

  /* USER CODE END Scr_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
