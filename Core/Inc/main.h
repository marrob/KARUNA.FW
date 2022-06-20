/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DSD_PCM_ISO_Pin GPIO_PIN_0
#define DSD_PCM_ISO_GPIO_Port GPIOA
#define CODEC_RST_ISO_Pin GPIO_PIN_1
#define CODEC_RST_ISO_GPIO_Port GPIOA
#define SEL_3_ISO_Pin GPIO_PIN_2
#define SEL_3_ISO_GPIO_Port GPIOA
#define SEL_0_ISO_Pin GPIO_PIN_3
#define SEL_0_ISO_GPIO_Port GPIOA
#define H5_3_ISO_Pin GPIO_PIN_4
#define H5_3_ISO_GPIO_Port GPIOA
#define H5_1_ISO_Pin GPIO_PIN_5
#define H5_1_ISO_GPIO_Port GPIOA
#define FS_352_384_ISO_Pin GPIO_PIN_7
#define FS_352_384_ISO_GPIO_Port GPIOA
#define SEL_1_ISO_Pin GPIO_PIN_0
#define SEL_1_ISO_GPIO_Port GPIOB
#define EN_SPDIF_1_Pin GPIO_PIN_13
#define EN_SPDIF_1_GPIO_Port GPIOB
#define SEL_2_ISO_Pin GPIO_PIN_14
#define SEL_2_ISO_GPIO_Port GPIOB
#define EN_MCLK_Pin GPIO_PIN_15
#define EN_MCLK_GPIO_Port GPIOB
#define EN_I2S_Pin GPIO_PIN_8
#define EN_I2S_GPIO_Port GPIOA
#define USART1_DIR_Pin GPIO_PIN_11
#define USART1_DIR_GPIO_Port GPIOA
#define EN_SPDIF_0_Pin GPIO_PIN_12
#define EN_SPDIF_0_GPIO_Port GPIOA
#define EN_AES_Pin GPIO_PIN_15
#define EN_AES_GPIO_Port GPIOA
#define FREQ_MUX_SEL_Pin GPIO_PIN_3
#define FREQ_MUX_SEL_GPIO_Port GPIOB
#define GP_LED_Pin GPIO_PIN_5
#define GP_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define   DEVICE_OK   0
#define   DEVICE_FAIL 1

#define DEVICE_NAME             "KARUNA"
#define DEVICE_NAME_SIZE        sizeof(DEVICE_NAME)
#define DEVICE_FW               "220620_1716"
#define DEVICE_FW_SIZE          sizeof(DEVICE_FW)
#define DEVICE_PCB              "V00"
#define DEVICE_PCB_SIZE         sizeof(DEVICE_PCB)
#define DEVICE_MNF              "github.com/marrob"
#define DEVICE_MNF_SIZE         sizeof(DEVICE_MNF)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
