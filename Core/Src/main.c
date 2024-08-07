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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

#include "../icode//Led/led.h"
#include "../icode//Key/key.h"
#include "../icode//buzzer/buzzer.h"
#include "../icode//delay/delay.h"
#include "../icode//relay/relay.h"
#include "../icode//printf/retarget.h"
#include "../icode//usart/usart.h"
#include "../icode//adc/adc.h"
#include "../icode//rtc/rtc.h"
#include "../icode//dht11/dht11.h"
#include "../icode//can/can1.h"
#include "../icode//bsp/usart_bsp.h"
#include "../../USB_DEVICE/App/usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef enum
{
  VOFA_RAWDATA = 0x00U,
  VOFA_JUSTFLOAT = 0x02U,
  VOFA_FIREWATER = 0x04U,
}VOFA_MODE_TypeDef;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_buffer[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t VOFA_Send_Message_VC (uint16_t len,VOFA_MODE_TypeDef Mode,...)
{
  if(Mode == VOFA_FIREWATER)//FiREWATER数据帧模式，尾帧规律
  {
    va_list valist;
    /* 为 num 个参数初始化 valist */
    va_start(valist,Mode);
    /* 访问所有赋给 valist 的参数 */
    float *temp;
    temp = va_arg(valist, float *);
    /* 清理为 valist 保留的内存 */
    va_end(valist);
    // CDC_Transmit_FS((char*)temp,len*sizeof(float));
    // while (CDC_Transmit_FS((char*)temp,len*sizeof(float)) != USBD_OK){};
    //查询串口状态是否正确，延时一段时间等待恢复
    uint8_t data[4] = {0x00, 0x00, 0x80, 0x7f};
    CDC_Transmit_FS(data,4);
    // while (CDC_Transmit_FS(data,4) != USBD_OK){};
    return HAL_OK;
  }
  if (Mode == VOFA_RAWDATA)
  {
    // RAWData 无需处理
    va_list valist;
    /* 为 num 个参数初始化 valist */
    va_start(valist,Mode);
    /* 访问所有赋给 valist 的参数 */
    uint8_t *temp;
    temp = va_arg(valist, uint8_t *);
    /* 清理为 valist 保留的内存 */
    va_end(valist);
    CDC_Transmit_FS(temp,len);
    // while (CDC_Transmit_FS(temp,len) != USBD_OK){};
    return HAL_OK;
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
  if (rx_buffer[0] == 0)
  {
    LED_2(0);
  }
  else if (rx_buffer[0] == 1)
  {
    LED_2(1);
  }
  else if (rx_buffer[0] == 2)
  {
    LED_2_Contrary();
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
  u_int16_t a = 0;
  uint8_t buff[1];
  RTC_DateTypeDef RtcDate;
  RTC_TimeTypeDef RtcTime;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);
  HAL_CAN_MspDeInit(&hcan);
  // CAN_User_Init(&hcan);
  HAL_UART_Receive_IT(&huart1,(uint8_t *)&USART1_NewData,1);
  RTC_Init();
  // Uart_Init(&huart1, rx_buffer, 8, Serialplot_Call_Back);
  // LED_1(1);
  // HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  // HAL_ADCEx_Calibration_Start(&hadc1);
  // HAL_ADCEx_Calibration_Start(&hadc2);
  // HAL_ADC_Start_DMA(&hadc1,(uint32_t *)a1,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float t = 0;
  while (1)
  {
    if(USB_RX_STA!=0)
    {
      // uint8_t tmp[1];
      // tmp[0] = 1;
      CDC_Transmit_FS(USB_RX_BUF,USB_RX_STA);
      USB_RX_STA = 0;
      memset(USB_RX_BUF,0,sizeof(USB_RX_BUF));
    }
    //   float tmep[256];
    //   static uint32_t flag;
    //   if (flag == 2500)
    //   {
    //     flag = 0;
    //   }
    //   float tmp_data;
    //   tmp_data = ((float)flag / 1000.0f) * ((float)flag / 1000.0f);
    //   tmep[0] = tmp_data;
    //   float led_status;
    //   led_status = HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin);
    //   tmep[1] = led_status;
    //   HAL_Delay(0);
    //   flag++;
    //   VOFA_Send_Message_VC(2,VOFA_FIREWATER,tmep);
    //
    // }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_USB;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
