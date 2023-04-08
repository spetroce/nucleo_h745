/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "eth.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint16_t MIN_CYCLE_ON_TIME = 5;  // Expressed in 100 ns intervals.
const uint16_t MAX_CYCLE_ON_TIME = 1000;  // Expressed in 100 ns intervals.
const uint32_t MIN_CYCLE_OFF_TIME = 5;  // Expressed in 100 ns intervals.
const uint32_t MAX_CYCLE_OFF_TIME = 100000;  // Expressed in 100 ns intervals.
const uint8_t MIN_NUM_CYCLE_PER_BURST = 1;
const uint8_t MAX_NUM_CYCLE_PER_BURST = 100;

#define MAX_CCR_VALUE_BUFFER_LEN 800
ALIGN_32BYTES (uint32_t g_ccr_value_buffer[MAX_CCR_VALUE_BUFFER_LEN]);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * \brief Fill the cycle burst CCR value buffer.
 *
 * \param cycle_on_time The cycle on time in 100 ns intervals. Valid values are
 * from MIN_CYCLE_ON_TIME to MAX_CYCLE_ON_TIME.
 * \param cycle_off_time The cycle off time in 100 ns intervals. Valid values
 * are from MIN_CYCLE_OFF_TIME to MAX_CYCLE_OFF_TIME.
 * \param num_cycle_per_burst The number of cycles per burst (ie. The number of
 * times a cycle should be repeated). Valid values are from
 * MIN_NUM_CYCLE_PER_BURST to MAX_NUM_CYCLE_PER_BURST.
 * \return Returns the number of values added to the DMA CCR value buffer. If an
 * error occured, zero is returned.
 */
uint16_t SetupCycleBurstBuffer(const uint16_t cycle_on_time,
                               const uint32_t cycle_off_time,
                               const uint8_t num_cycle_per_burst) {
  if (cycle_on_time < MIN_CYCLE_ON_TIME ||
      cycle_on_time > MAX_CYCLE_ON_TIME ||
      cycle_off_time < MIN_CYCLE_OFF_TIME ||
      cycle_off_time > MAX_CYCLE_OFF_TIME ||
      num_cycle_per_burst < MIN_NUM_CYCLE_PER_BURST ||
      num_cycle_per_burst > MAX_NUM_CYCLE_PER_BURST) {
    return 0;
  }
  uint32_t tim_cnt = 0;
  uint16_t i = 0, j;
  g_ccr_value_buffer[i++] = tim_cnt;
  for (j = 0; j < num_cycle_per_burst; ++j) {
    tim_cnt += cycle_on_time;
    g_ccr_value_buffer[i++] = tim_cnt;
    if (j+1 < num_cycle_per_burst) {
      tim_cnt += cycle_off_time;
      g_ccr_value_buffer[i++] = tim_cnt;
    }
  }
  return i;
}

// void TimOcStartDmaLowLevel() {
//   /* Set the DMA compare callbacks */
//   // htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
//   // htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;
//   /* Set the DMA error callback */
//   // htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError ;

//   /* Enable the Output compare channel */
//   // TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
//   LL_TIM_CC_DisableChannel(TIM2, LL_TIM_CHANNEL_CH1);  // HAL does this, might not be necessary.
//   LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);

//   /* Enable the DMA stream */
//   HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1, Length);

//   LL_TIM_EnableDMAReq_CC1(TIM2);
  
//   LL_TIM_GenerateEvent_CC1(TIM2);

//   LL_TIM_GenerateEvent_UPDATE(TIM2);

//   LL_TIM_EnableCounter(TIM2);
// }

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;

HAL_StatusTypeDef HAL_DMA_Start_(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
  // #define DMA_SxCR_EN_Pos          (0U)
  // #define DMA_SxCR_EN_Msk          (0x1UL << DMA_SxCR_EN_Pos)                    /*!< 0x00000001 */
  // #define DMA_SxCR_EN              DMA_SxCR_EN_Msk                               /*!< Stream enable / flag stream ready when read low */
  /* Disable the peripheral */
  // __HAL_DMA_DISABLE(hdma);
  ((DMA_Stream_TypeDef *)(hdma)->Instance)->CR &= ~DMA_SxCR_EN;

  /* calculate DMA base and stream number */
  DMA_Base_Registers  *regs_dma  = (DMA_Base_Registers *)hdma->StreamBaseAddress;

  /* Clear the DMAMUX synchro overrun flag */
  hdma->DMAmuxChannelStatus->CFR = hdma->DMAmuxChannelStatusMask;

  /* Clear all interrupt flags at correct offset within the register */
  regs_dma->IFCR = 0x3FUL << (hdma->StreamIndex & 0x1FU);

  /* Clear DBM bit */
  ((DMA_Stream_TypeDef *)hdma->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);

  /* Configure DMA Stream data length */
  ((DMA_Stream_TypeDef *)hdma->Instance)->NDTR = DataLength;

  /* Configure DMA Stream destination address */
  ((DMA_Stream_TypeDef *)hdma->Instance)->PAR = DstAddress;

  /* Configure DMA Stream source address */
  ((DMA_Stream_TypeDef *)hdma->Instance)->M0AR = SrcAddress;

  /* Enable the Peripheral */
  // __HAL_DMA_ENABLE(hdma);
  ((DMA_Stream_TypeDef *)(hdma)->Instance)->CR |=  DMA_SxCR_EN;

  return HAL_OK;
}

void TimOcStartDma(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData, uint16_t Length) {
  /* Set the DMA compare callbacks */
  // htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = TIM_DMADelayPulseCplt;
  // htim->hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = TIM_DMADelayPulseHalfCplt;
  /* Set the DMA error callback */
  // htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = TIM_DMAError ;

  /* Enable the Output compare channel */
  TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);

  /* Enable the DMA stream */
  HAL_DMA_Start_(htim->hdma[TIM_DMA_ID_CC1], (uint32_t)pData, (uint32_t)&htim->Instance->CCR1, Length);

  /* Enable the TIM Capture/Compare 1 DMA request */
  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
  
  htim2.Instance->EGR  |= TIM_EGR_CC1G;

  htim2.Instance->EGR  |= TIM_EGR_UG;

  // if (IS_TIM_BREAK_INSTANCE(htim->Instance)) {  // returns false for TIM2
  //   /* Enable the main output */
  //   __HAL_TIM_MOE_ENABLE(htim);
  // }

  __HAL_TIM_ENABLE(htim);
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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* Clean Data Cache to update the content of the SRAM to be used by the DMA */
  // SCB_CleanDCache_by_Addr((uint32_t*)g_ccr_value_buffer, MAX_CCR_VALUE_BUFFER_LEN);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ETH_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (g_gpio_user_button) {
      g_gpio_user_button = false;
      int i;
      uint16_t ccr_value_buffer_len = SetupCycleBurstBuffer(10, 5, 4);
      if (ccr_value_buffer_len) {
        for (i = 0; i < 4; ++i) {
          HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
          HAL_Delay(50);
        }
      }
      // __HAL_TIM_SET_COUNTER(&htim2, 0);
      SCB_CleanDCache_by_Addr(g_ccr_value_buffer, ccr_value_buffer_len);
      // htim2.Instance->EGR  |= TIM_EGR_UG;
      // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, g_ccr_value_buffer[0]);
      // HAL_TIM_OC_Start_DMA(&htim2, TIM_CHANNEL_1, g_ccr_value_buffer, ccr_value_buffer_len);
      TimOcStartDma(&htim2, TIM_CHANNEL_1, g_ccr_value_buffer, ccr_value_buffer_len);
      // HAL_TIM_PWM_PulseFinishedCallback()
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
