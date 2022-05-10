/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liveLed.h"
#include "common.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum _CtrlStatesTypeDef
{
  SDEV_START,                   //0
  SDEV_WAIT,                    //1
}CtrlStatesTypeDef;

typedef struct _AppTypeDef
{
  uint16_t FailCnt;
  struct
  {
    CtrlStatesTypeDef Next;
    CtrlStatesTypeDef Curr;
    CtrlStatesTypeDef Pre;
  }State;

  struct _status
  {
    uint32_t MainCycleTime;
    uint32_t UartTaskCnt;
    uint32_t SuccessParsedCmdCnt;
    uint32_t Seconds;
    uint32_t QueryCnt;
    uint32_t UART_Receive_IT_ErrorCounter;
    uint32_t UartErrorCounter;

  }Diag;


}DeviceTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE        40
#define CMDLINE_UNKNOWN_PARAMETER_ERROR    "!UNKNOWN PARAMETER ERROR '%2s'"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
DeviceTypeDef Device;
LiveLED_HnadleTypeDef hLiveLed;

char UartRxBuffer[UART_BUFFER_SIZE];
char UartTxBuffer[UART_BUFFER_SIZE];
char        UartCharacter;
uint8_t     UartRxBufferPtr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/*** LiveLed ***/
void LiveLedOff(void);
void LiveLedOn(void);

/*** UART/RS485 ***/
char* CmdParser(char *line);

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
  /*** LiveLed ***/
  hLiveLed.LedOffFnPtr = &LiveLedOff;
  hLiveLed.LedOnFnPtr = &LiveLedOn;
  hLiveLed.HalfPeriodTimeMs = 500;
  LiveLedInit(&hLiveLed);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t timestamp;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(HAL_GetTick() - timestamp > 100)
    {
      timestamp = HAL_GetTick();
    }
    LiveLedTask(&hLiveLed);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  if(HAL_UART_Receive_IT(&huart1, (uint8_t *)&UartCharacter, 1) != HAL_OK)
  {
    Device.Diag.UART_Receive_IT_ErrorCounter++;
  }
  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_SPDIF_1_Pin|EN_MCLK_Pin|FREQ_MUX_SEL_Pin|GP_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_I2S_Pin|USART1_DIR_Pin|EN_SPDIF_0_Pin|EN_AES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DSD_PCM_ISO_Pin CODEC_RST_ISO_Pin SEL_3_ISO_Pin SEL_0_ISO_Pin
                           H5_3_ISO_Pin H5_1_ISO_Pin FS_352_384_ISO_Pin */
  GPIO_InitStruct.Pin = DSD_PCM_ISO_Pin|CODEC_RST_ISO_Pin|SEL_3_ISO_Pin|SEL_0_ISO_Pin
                          |H5_3_ISO_Pin|H5_1_ISO_Pin|FS_352_384_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEL_1_ISO_Pin SEL_2_ISO_Pin */
  GPIO_InitStruct.Pin = SEL_1_ISO_Pin|SEL_2_ISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_SPDIF_1_Pin EN_MCLK_Pin FREQ_MUX_SEL_Pin GP_LED_Pin */
  GPIO_InitStruct.Pin = EN_SPDIF_1_Pin|EN_MCLK_Pin|FREQ_MUX_SEL_Pin|GP_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_I2S_Pin USART1_DIR_Pin EN_SPDIF_0_Pin EN_AES_Pin */
  GPIO_InitStruct.Pin = EN_I2S_Pin|USART1_DIR_Pin|EN_SPDIF_0_Pin|EN_AES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* LEDs -----------------------------------------------------------------------*/
void LiveLedOn(void)
{
  HAL_GPIO_WritePin(GP_LED_GPIO_Port, GP_LED_Pin, GPIO_PIN_SET);
}

void LiveLedOff(void)
{
  HAL_GPIO_WritePin(GP_LED_GPIO_Port, GP_LED_Pin, GPIO_PIN_RESET);
}


/* UART ----------------------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *context)
{
  if(HAL_UART_Receive_IT(context, (uint8_t *)&UartCharacter, 1) != HAL_OK)
    Device.Diag.UART_Receive_IT_ErrorCounter++;
  else
  {
    if(UartCharacter == '\n')
    {
      UartRxBuffer[UartRxBufferPtr] = '\0';
      strcpy(UartTxBuffer, CmdParser(UartRxBuffer));
      UartRxBufferPtr = 0;
    }
    else
      UartRxBuffer[UartRxBufferPtr++] = UartCharacter;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Device.Diag.UartErrorCounter++;
  __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_PE);
  __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_FE);
  __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_NE);
  __HAL_UART_CLEAR_FLAG(huart,UART_FLAG_ORE);
  UartRxBufferPtr = 0;
  if(HAL_UART_Receive_IT(huart, (uint8_t *)&UartCharacter, 1) != HAL_OK)
    Device.Diag.UART_Receive_IT_ErrorCounter++;
}

char* CmdParser(char *line)
{
  static char resp[UART_BUFFER_SIZE];
  char cmd[20];
  char arg1[10];
  char arg2[10];

  memset(resp,0x00,UART_BUFFER_SIZE );
  uint8_t params = sscanf(line, "%s %s %s", cmd, arg1, arg2);

  if(params == 1)
  {/*** parméter mentes utasitások ***/
    if(!strcmp(cmd, "*OPC?"))
    {
      strcpy(resp, "*OPC");
    }
    else if(!strcmp(cmd, "*RDY?"))
    {
      strcpy(resp, "*RDY");
    }
    else if(!strcmp(cmd, "*WHOIS?"))
    {
      strcpy(resp, DEVICE_NAME);
    }
    else if(!strcmp(cmd, "*VER?"))
    {
      strcpy(resp, DEVICE_FW);
    }
    else if(!strcmp(cmd, "*UID?"))
    {
      sprintf(resp, "%4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
    }
    else if(!strcmp(cmd,"UPTIME?"))
    {
  //    sprintf(resp, "%ld", Device.Diag.UpTimeSec);
    }
    else if(!strcmp(cmd,"FRQ:LRCK?"))
    {
  //   sprintf(resp, "%ld", Device.Meas.FreqLRCK_MHz);
    }
    else if(!strcmp(cmd,"FRQ:BCLK?"))
    {
   //   sprintf(resp, "%ld", Device.Meas.FreqBCLK_MHz);
    }
    else if(!strcmp(cmd,"XMOS:STATUS?"))
    {
  //   sprintf(resp, "%02X", Device.XMOS.Status);
    }
    else if(!strcmp(cmd,"XMOS:MUTE?"))
    {
  //    sprintf(resp, "%01d", Device.XMOS.IsMute);
    }
    else if(!strcmp(cmd,"MODE?"))
    {
  //    sprintf(resp, "%02d", (uint8_t)Device.Mode.Curr);
    }
    else if (!strcmp(cmd,"SRC:BYPAS?"))
    {
  //    sprintf(resp, "%01d", (uint8_t)Device.SRC.System.BYASS);
    }
    else if (!strcmp(cmd,"SRC:MUTE?"))
    {
    //  sprintf(resp, "%01d", (uint8_t)Device.SRC.System.MUTE);
    }
    else if (!strcmp(cmd,"SRC:PDN?"))
    {
  //    sprintf(resp, "%01d", (uint8_t)Device.SRC.System.PDN);
    }
    else if (!strcmp(cmd,"DAC:VOL1?"))
    {
  //    sprintf(resp, "%01d", (uint8_t)Device.BD34301.Volume1);
    }
    else if (!strcmp(cmd,"DAC:VOL2?"))
    {
 //     sprintf(resp, "%01d", (uint8_t)Device.BD34301.Volume2);
    }
    else
    {
      strcpy(resp, "!UNKNOWN");
    }
  }
  if(params == 2)
  {/*** Paraméteres utasitások ***/
    if(!strcmp(cmd,"MODE"))
    {
 //     Device.Mode.Curr = (Modes_t) strtol(arg1, NULL, 0);
      strcpy(resp, "RDY");
    }
    else if(!strcmp(cmd,"SRC:BYPAS"))
    {
  //    Device.SRC.System.BYASS = strtol(arg1, NULL, 0);
      strcpy(resp, "RDY");
    }
    else if(!strcmp(cmd,"SRC:MUTE"))
    {
  //    Device.SRC.System.MUTE = strtol(arg1, NULL, 0);
      strcpy(resp, "RDY");
    }
    else if(!strcmp(cmd,"SRC:PDN"))
    {
 //     Device.SRC.System.PDN = strtol(arg1, NULL, 0);
      strcpy(resp, "RDY");
    }
    else if(!strcmp(cmd,"DAC:VOL1"))
    {
  //    Device.BD34301.Volume1 = strtol(arg1, NULL, 0);
  //    BD34301SettingsUpdate(&Device.BD34301);
      strcpy(resp, "RDY");
    }
    else if(!strcmp(cmd,"DAC:VOL2"))
    {
   //   Device.BD34301.Volume2 = strtol(arg1, NULL, 0);
   //   BD34301SettingsUpdate(&Device.BD34301);
      strcpy(resp, "RDY");
    }
    else
    {
      strcpy(resp, "!UNKNOWN");
    }
  }
  uint8_t length = strlen(resp);
  resp[length] = '\n';
  resp[length + 1] = '\0';
  return resp;
}

/* RS485-----------------------------------------------------------------------*/

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
