/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* HumanInterfaceDevice */
  CUSTOM_STM_HII,
  CUSTOM_STM_HCP,
  CUSTOM_STM_REM,
  CUSTOM_STM_INPUTREP,
  /* Device_Information */
  CUSTOM_STM_MANS,
  CUSTOM_STM_MONS,
  CUSTOM_STM_FRS,
  CUSTOM_STM_SRS,
  CUSTOM_STM_PNI,
  /* Battery */
  CUSTOM_STM_BAL,
  /* Scan_Parameters */
  CUSTOM_STM_SIW,
  CUSTOM_STM_SCR,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* HID_Information */
  CUSTOM_STM_HII_READ_EVT,
  /* HID_Control_Point */
  CUSTOM_STM_HCP_WRITE_NO_RESP_EVT,
  /* Report_Map */
  CUSTOM_STM_REM_READ_EVT,
  /* InputReport */
  CUSTOM_STM_INPUTREP_READ_EVT,
  CUSTOM_STM_INPUTREP_WRITE_EVT,
  CUSTOM_STM_INPUTREP_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_INPUTREP_NOTIFY_DISABLED_EVT,
  /* Manufacturer_Name */
  CUSTOM_STM_MANS_READ_EVT,
  /* Model_Number */
  CUSTOM_STM_MONS_READ_EVT,
  /* Firmware_Revision */
  CUSTOM_STM_FRS_READ_EVT,
  /* Software_Revision */
  CUSTOM_STM_SRS_READ_EVT,
  /* PnPID */
  CUSTOM_STM_PNI_READ_EVT,
  /* Battery_Level */
  CUSTOM_STM_BAL_READ_EVT,
  CUSTOM_STM_BAL_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_BAL_NOTIFY_DISABLED_EVT,
  /* Scan_Interval_Window */
  CUSTOM_STM_SIW_WRITE_NO_RESP_EVT,
  /* Scan_Refresh */
  CUSTOM_STM_SCR_NOTIFY_ENABLED_EVT,
  CUSTOM_STM_SCR_NOTIFY_DISABLED_EVT,
  CUSTOM_STM_NOTIFICATION_COMPLETE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
  uint16_t                      AttrHandle;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint16_t SizeHii;
extern uint16_t SizeHcp;
extern uint16_t SizeRem;
extern uint16_t SizeInputrep;
extern uint16_t SizeMans;
extern uint16_t SizeMons;
extern uint16_t SizeFrs;
extern uint16_t SizeSrs;
extern uint16_t SizePni;
extern uint16_t SizeBal;
extern uint16_t SizeSiw;
extern uint16_t SizeScr;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
tBleStatus Custom_STM_App_Update_Char_Variable_Length(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload, uint8_t size);
tBleStatus Custom_STM_App_Update_Char_Ext(uint16_t Connection_Handle, Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
