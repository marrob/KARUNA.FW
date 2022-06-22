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

typedef struct _AppTypeDef
{
  struct _Karuna
  {
    uint32_t DI;
    uint32_t DO;
    uint32_t UpTimeSec;
  }Karuna;

  struct _Diag
  {
    uint32_t RS485ResponseCnt;
    uint32_t RS485RequestCnt;
    uint32_t RS485UnknwonCnt;
    uint32_t RS485NotMyCmdCnt;
    uint32_t UartErrorCnt;
  }Diag;

}DeviceTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*** RS485 ***/
#define RS485_BUFFER_SIZE     40
#define RS485_TX_HOLD_MS      1
#define RS485_CMD_LENGTH      35
#define RS485_ARG1_LENGTH     35
#define RS485_ARG2_LENGTH     35

/*** Address ***/
#define CLIENT_TX_ADDR        0x10
#define CLIENT_RX_ADDR        0x01

/*** Karuna ***/
#define KRN_DI_A0             ((uint32_t)1<<0)
#define KRN_DI_A1             ((uint32_t)1<<1)
#define KRN_DI_A2             ((uint32_t)1<<2)
#define KRN_DI_A3             ((uint32_t)1<<3)
#define KRN_DI_DSD_PCM        ((uint32_t)1<<4)
#define KRN_DI_H51            ((uint32_t)1<<5)
#define KRN_DI_H53            ((uint32_t)1<<6)
#define KRN_DI_RCA            ((uint32_t)1<<7)
#define KRN_DI_BNC            ((uint32_t)1<<8)
#define KRN_DI_XLR            ((uint32_t)1<<9)
#define KRN_DI_I2S            ((uint32_t)1<<10)
#define KRN_DI_MCLK_I2S       ((uint32_t)1<<11)

#define KRN_DO_RCA_EN         ((uint32_t)1<<0)
#define KRN_DO_BNC_EN         ((uint32_t)1<<1)
#define KRN_DO_XLR_EN         ((uint32_t)1<<2)
#define KRN_DO_I2S_EN         ((uint32_t)1<<3)
#define KRN_DO_MCLK_I2S_EN    ((uint32_t)1<<4)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
DeviceTypeDef Device;
LiveLED_HnadleTypeDef hLiveLed;

/*** RS485 ***/
char    UartRxBuffer[RS485_BUFFER_SIZE];
char    UartTxBuffer[RS485_BUFFER_SIZE];
char    UartCharacter;
uint8_t UartRxBufferPtr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_WWDG_Init(void);
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
uint8_t ReadDI(void);
void WriteDO(uint8_t state);
void KarunaFerq22M5792(void);
void KarunaFreq24M5760(void);
void KarunaClockSelectTask(void);
void KarunaMclkOnI2sDisable(void);
void KarunaMclkOnI2sEnable(void);

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
  Device.Karuna.DO = KRN_DO_RCA_EN | KRN_DO_BNC_EN | KRN_DO_XLR_EN | KRN_DO_I2S_EN;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_WWDG_Init();
  /* USER CODE BEGIN 2 */

  /*** Check if the system has resumed from WWDG reset ***/
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    //ToDo
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t timestamp;

  while (1)
  {
    HAL_WWDG_Refresh(&hwwdg);
    if(HAL_GetTick() - timestamp > 100)
    {
      timestamp = HAL_GetTick();
      Device.Karuna.DI = ReadDI();
      WriteDO(Device.Karuna.DO);
      KarunaClockSelectTask();

      if(Device.Karuna.DO & KRN_DO_RCA_EN)
        Device.Karuna.DI |= KRN_DI_RCA;
      else
        Device.Karuna.DI &= ~KRN_DI_RCA;

      if(Device.Karuna.DO & KRN_DO_BNC_EN)
        Device.Karuna.DI |= KRN_DI_BNC;
      else
        Device.Karuna.DI &= ~KRN_DI_BNC;

      if(Device.Karuna.DO & KRN_DO_XLR_EN)
        Device.Karuna.DI |= KRN_DI_XLR;
      else
        Device.Karuna.DI &= ~KRN_DI_XLR;

      if(Device.Karuna.DO & KRN_DO_I2S_EN)
        Device.Karuna.DI |= KRN_DI_I2S;
      else
        Device.Karuna.DI &= ~KRN_DI_I2S;

      if(Device.Karuna.DO & KRN_DO_MCLK_I2S_EN)
        Device.Karuna.DI |= KRN_DI_MCLK_I2S;
      else
        Device.Karuna.DI &= ~KRN_DI_MCLK_I2S;
    }
    LiveLedTask(&hLiveLed);
    RS485TxTask();
    UpTimeTask();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  huart1.Init.BaudRate = 230400;
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
    Device.Diag.UartErrorCnt++;
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief WWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_WWDG_Init(void)
{

  /* USER CODE BEGIN WWDG_Init 0 */

  /* USER CODE END WWDG_Init 0 */

  /* USER CODE BEGIN WWDG_Init 1 */

  /* USER CODE END WWDG_Init 1 */
  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN WWDG_Init 2 */

  /* USER CODE END WWDG_Init 2 */

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
    Device.Diag.UartErrorCnt++;
  else
  {
    if(UartRxBufferPtr < RS485_BUFFER_SIZE - 1)
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
    else
    {
      UartRxBufferPtr = 0;
      memset(UartRxBuffer,0x00, RS485_BUFFER_SIZE);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Device.Diag.UartErrorCnt++;
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);

  if(HAL_UART_Receive_IT(huart, (uint8_t *)&UartCharacter, 1) != HAL_OK)
    Device.Diag.UartErrorCnt++;
}

char* RS485Parser(char *line)
{
  unsigned int addr = 0;
  char buffer[RS485_BUFFER_SIZE];
  char cmd[RS485_CMD_LENGTH];
  char arg1[RS485_ARG1_LENGTH];
  char arg2[RS485_ARG2_LENGTH];

  memset(buffer, 0x00, RS485_BUFFER_SIZE);
  memset(cmd,0x00, RS485_CMD_LENGTH);
  memset(arg1,0x00, RS485_ARG1_LENGTH);
  memset(arg2,0x00, RS485_ARG2_LENGTH);

  uint8_t params = sscanf(line, "#%x %s %s %s",&addr, cmd, arg1, arg2);
  if(addr != CLIENT_RX_ADDR)
  {
    Device.Diag.RS485NotMyCmdCnt++;
    return NULL;
  }
  Device.Diag.RS485RequestCnt++;
  if(params == 2)
  {
    if(!strcmp(cmd, "*OPC?"))
      strcpy(buffer, "OK");
    else if(!strcmp(cmd, "FW?"))
      sprintf(buffer, "FW %s", DEVICE_FW);
    else if(!strcmp(cmd, "UID?"))
      sprintf(buffer, "UID %4lX%4lX%4lX",HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
    else if(!strcmp(cmd, "PCB?"))
      sprintf(buffer, "PCB %s", DEVICE_PCB);
    else if(!strcmp(cmd,"UPTIME?"))
       sprintf(buffer, "UPTIME %08lX", Device.Karuna.UpTimeSec);
    else if(!strcmp(cmd,"DI?"))
       sprintf(buffer, "DI %08lX", Device.Karuna.DI);
    else if(!strcmp(cmd,"DO?"))
       sprintf(buffer, "DO %08lX", Device.Karuna.DO);
    else if(!strcmp(cmd,"UE?"))
      sprintf(buffer, "UE %08lX", Device.Diag.UartErrorCnt);
    else
      Device.Diag.RS485UnknwonCnt++;
  }
  if(params == 3)
  {
    if(!strcmp(cmd,"DO"))
    {
      Device.Karuna.DO = strtol(arg1, NULL, 16);
      strcpy(buffer, "OK");
    }
    else
      Device.Diag.RS485UnknwonCnt++;
  }
  static char resp[RS485_BUFFER_SIZE + 5];
  memset(resp, 0x00, RS485_BUFFER_SIZE);
  sprintf(resp, "#%02X %s", CLIENT_TX_ADDR, buffer);
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
    Device.Diag.RS485ResponseCnt++;
    RS485DirTx();
    DelayMs(RS485_TX_HOLD_MS);
    HAL_UART_Transmit(&huart1, (uint8_t*) UartTxBuffer, txn, 100);
    UartTxBuffer[0] = 0;
    RS485DirRx();
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

/* Karuna---------------------------------------------------------------------*/
uint8_t ReadDI(void)
{
  uint8_t state = 0;

  if(HAL_GPIO_ReadPin(SEL_0_ISO_GPIO_Port, SEL_0_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_A0;

  if(HAL_GPIO_ReadPin(SEL_1_ISO_GPIO_Port, SEL_1_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_A1;

  if(HAL_GPIO_ReadPin(SEL_2_ISO_GPIO_Port, SEL_2_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_A2;

  if(HAL_GPIO_ReadPin(SEL_3_ISO_GPIO_Port, SEL_3_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_A3;

  if(HAL_GPIO_ReadPin(DSD_PCM_ISO_GPIO_Port, DSD_PCM_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_DSD_PCM;

  if(HAL_GPIO_ReadPin(DSD_PCM_ISO_GPIO_Port, DSD_PCM_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_DSD_PCM;

  if(HAL_GPIO_ReadPin(H5_3_ISO_GPIO_Port, H5_3_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_H53;

  if(HAL_GPIO_ReadPin(H5_1_ISO_GPIO_Port, H5_1_ISO_Pin) == GPIO_PIN_RESET)
    state |= KRN_DI_H51;

  return state;
}

void WriteDO(uint8_t state)
{
  if(state & KRN_DO_RCA_EN)
    HAL_GPIO_WritePin(EN_SPDIF_0_GPIO_Port, EN_SPDIF_0_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_SPDIF_0_GPIO_Port, EN_SPDIF_0_Pin, GPIO_PIN_RESET);

  if(state & KRN_DO_BNC_EN)
    HAL_GPIO_WritePin(EN_SPDIF_1_GPIO_Port, EN_SPDIF_1_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_SPDIF_1_GPIO_Port, EN_SPDIF_1_Pin, GPIO_PIN_RESET);

  if(state & KRN_DO_XLR_EN)
    HAL_GPIO_WritePin(EN_AES_GPIO_Port, EN_AES_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_AES_GPIO_Port, EN_AES_Pin, GPIO_PIN_RESET);

  if(state & KRN_DO_I2S_EN)
    HAL_GPIO_WritePin(EN_I2S_GPIO_Port, EN_I2S_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(EN_I2S_GPIO_Port, EN_I2S_Pin, GPIO_PIN_RESET);

  if(state & KRN_DO_MCLK_I2S_EN)
    HAL_GPIO_WritePin(EN_MCLK_GPIO_Port, EN_I2S_Pin, GPIO_PIN_RESET);
  else
    HAL_GPIO_WritePin(EN_MCLK_GPIO_Port, EN_I2S_Pin, GPIO_PIN_SET);
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
  uint8_t a0 = (Device.Karuna.DI & KRN_DI_A0) == KRN_DI_A0;
  uint8_t dsd = (Device.Karuna.DI & KRN_DI_DSD_PCM) == KRN_DI_DSD_PCM;
  if(!a0 && !dsd)
    KarunaFerq22M5792();
  if(!a0 && dsd)
    KarunaFerq22M5792();
  if(a0 && !dsd)
    KarunaFreq24M5760();
  if(a0 && dsd)
    KarunaFerq22M5792();
}
/* Tools----------------------------------------------------------------------*/
void UpTimeTask(void)
{
  static uint32_t timestamp;
  if(HAL_GetTick() - timestamp > 1000)
  {
    timestamp = HAL_GetTick();
    Device.Karuna.UpTimeSec++;
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
