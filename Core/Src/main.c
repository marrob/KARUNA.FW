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
#include <stdlib.h>

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

  struct
  {
    uint8_t Status;
    uint8_t Outputs;
  }Karuna;

  struct _status
  {
    uint32_t MainCycleTime;
    uint32_t UartTaskCnt;
    uint32_t SuccessParsedCmdCnt;
    uint32_t QueryCnt;
    uint32_t UART_Receive_IT_ErrorCounter;
    uint32_t UartErrorCounter;
    uint32_t UpTimeSec;
  }Diag;


}DeviceTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_BUFFER_SIZE        40
#define CMDLINE_UNKNOWN_PARAMETER_ERROR    "!UNKNOWN PARAMETER ERROR '%2s'"

#define KRN_STAT_A0             (uint8_t)1<<0
#define KRN_STAT_A1             (uint8_t)1<<1
#define KRN_STAT_A2             (uint8_t)1<<2
#define KRN_STAT_A3             (uint8_t)1<<3
#define KRN_STAT_DSD_PCM        (uint8_t)1<<4
#define KRN_STAT_H51            (uint8_t)1<<5
#define KRN_STAT_H53            (uint8_t)1<<6

#define KRN_CTRL_RCA            (uint8_t)1<<0
#define KRN_CTRL_BNC            (uint8_t)1<<1
#define KRN_CTRL_XLR            (uint8_t)1<<2
#define KRN_CTRL_I2S            (uint8_t)1<<3

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
char* RS485Parser(char *line);
void RS485DirRx(void);
void RS485DirTx(void);
void RS485TxTask(void);

/*** Karuna ***/
uint8_t GetKarunaStatus(void);
void SetKarunaOutputs(uint8_t state);
void KarunaFerq22M5792(void);
void KarunaFreq24M5760(void);
void KarunaClockSelectTask(void);

/*** Tools ***/
void UpTimeTask(void);

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

  /*** RS485 ***/
  RS485DirRx();

  /*** Karuna ***/
  Device.Karuna.Outputs = KRN_CTRL_RCA | KRN_CTRL_BNC | KRN_CTRL_XLR | KRN_CTRL_I2S;


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
      Device.Karuna.Status = GetKarunaStatus();
      SetKarunaOutputs(Device.Karuna.Outputs);
      KarunaClockSelectTask();
    }
    LiveLedTask(&hLiveLed);
    RS485TxTask();
    UpTimeTask();
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


/* UART-RS485-----------------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *context)
{
  if(HAL_UART_Receive_IT(context, (uint8_t *)&UartCharacter, 1) != HAL_OK)
    Device.Diag.UART_Receive_IT_ErrorCounter++;
  else
  {
    if(UartCharacter == '\n')
    {
      UartRxBuffer[UartRxBufferPtr] = '\0';
      strcpy(UartTxBuffer, RS485Parser(UartRxBuffer));
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

/* RS485-----------------------------------------------------------------------*/
char* RS485Parser(char *line)
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
       sprintf(resp, "%ld", Device.Diag.UpTimeSec);
    }
    else if(!strcmp(cmd,"STATUS?"))
    {
       sprintf(resp, "%02X", Device.Karuna.Status);
    }
    else if(!strcmp(cmd,"OUTS?"))
    {
       sprintf(resp, "%02X", Device.Karuna.Outputs);
    }
    else
      strcpy(resp, "!UNKNOWN");
  }
  if(params == 2)
  {/*** Paraméteres utasitások ***/
    if(!strcmp(cmd,"OUTS"))
    {
      Device.Karuna.Outputs = strtol(arg1, NULL, 0);
      strcpy(resp, "RDY");
    }
    else
      strcpy(resp, "!UNKNOWN");
  }
  uint8_t length = strlen(resp);
  resp[length] = '\n';
  resp[length + 1] = '\0';
  return resp;
}

void RS485TxTask(void)
{
  uint8_t txn=strlen(UartTxBuffer);
  if( txn != 0)
  {
    RS485DirTx();
    huart1.Instance->CR1 &= (~0x04);
    DelayMs(1);
    HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, txn, 100);
    UartTxBuffer[0] = 0;
    RS485DirRx();
    huart1.Instance->CR1 |= 0x04;
  }
}

void RS485DirTx(void)
{
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_SET);
}

void RS485DirRx(void)
{
  HAL_GPIO_WritePin(USART1_DIR_GPIO_Port, USART1_DIR_Pin, GPIO_PIN_RESET);
}

/* Karuna----------------------------------------------------------------------*/
uint8_t GetKarunaStatus(void)
{
  uint8_t status = 0;

  if(HAL_GPIO_ReadPin(SEL_0_ISO_GPIO_Port, SEL_0_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_A0;

  if(HAL_GPIO_ReadPin(SEL_1_ISO_GPIO_Port, SEL_1_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_A1;

  if(HAL_GPIO_ReadPin(SEL_2_ISO_GPIO_Port, SEL_2_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_A2;

  if(HAL_GPIO_ReadPin(SEL_3_ISO_GPIO_Port, SEL_3_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_A3;

  if(HAL_GPIO_ReadPin(DSD_PCM_ISO_GPIO_Port, DSD_PCM_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_DSD_PCM;

  if(HAL_GPIO_ReadPin(DSD_PCM_ISO_GPIO_Port, DSD_PCM_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_DSD_PCM;

  if(HAL_GPIO_ReadPin(H5_3_ISO_GPIO_Port, H5_3_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_H53;

  if(HAL_GPIO_ReadPin(H5_1_ISO_GPIO_Port, H5_1_ISO_Pin) == GPIO_PIN_RESET)
    status |= KRN_STAT_H51;

  return status;
}

void SetKarunaOutputs(uint8_t state)
{
  if(state & KRN_CTRL_RCA)
    HAL_GPIO_WritePin(EN_SPDIF_0_GPIO_Port, EN_SPDIF_0_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_SPDIF_0_GPIO_Port, EN_SPDIF_0_Pin, GPIO_PIN_RESET);

  if(state & KRN_CTRL_BNC)
    HAL_GPIO_WritePin(EN_SPDIF_1_GPIO_Port, EN_SPDIF_1_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_SPDIF_1_GPIO_Port, EN_SPDIF_1_Pin, GPIO_PIN_RESET);

  if(state & KRN_CTRL_XLR)
    HAL_GPIO_WritePin(EN_AES_GPIO_Port, EN_AES_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_AES_GPIO_Port, EN_AES_Pin, GPIO_PIN_RESET);

  if(state & KRN_CTRL_I2S)
    HAL_GPIO_WritePin(EN_I2S_GPIO_Port, EN_I2S_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_I2S_GPIO_Port, EN_I2S_Pin, GPIO_PIN_RESET);
};

void KarunaFerq22M5792(void)
{
  HAL_GPIO_WritePin(FREQ_MUX_SEL_GPIO_Port, FREQ_MUX_SEL_Pin, GPIO_PIN_SET);
}

void KarunaFreq24M5760(void)
{
  HAL_GPIO_WritePin(FREQ_MUX_SEL_GPIO_Port, FREQ_MUX_SEL_Pin, GPIO_PIN_RESET);
}

void KarunaClockSelectTask(void)
{

  uint8_t a0 = (Device.Karuna.Status & KRN_STAT_A0) == KRN_STAT_A0;
  uint8_t dsd = (Device.Karuna.Status & KRN_STAT_DSD_PCM) == KRN_STAT_DSD_PCM;

  if(!a0 && !dsd)
  {
    KarunaFerq22M5792();
  }
  if(!a0 && dsd)
  {
    KarunaFerq22M5792();
  }
  if(a0 && !dsd)
  {
    KarunaFreq24M5760();
  }
  if(a0 && dsd)
  {
    KarunaFerq22M5792();
  }
}
/* Tools----------------------------------------------------------------------*/
void UpTimeTask(void)
{
  static uint32_t timestamp;

  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.Diag.UpTimeSec++;
  }
}



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
