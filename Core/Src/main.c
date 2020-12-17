/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static void TIM3_deinit(void);
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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 );
 LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
 LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
 LL_TIM_EnableCounter(TIM3);
 LL_TIM_OC_SetCompareCH1(TIM3, 65538);
 LL_TIM_OC_SetCompareCH2(TIM3, 4000);
unsigned long freq_arr[]={120,200,300,400,500,600,700,800,900}; //Hz
//uint32_t main_clk_freq=72000000;
int cnt=0;
const uint16_t presc=72;

const  uint16_t freq[280]={369,329,369,391,369,329,293,293,329,369,391,391,391,391,391,369,329,329,220,293,
329,369,369,329,369,329,293,293,293,493,493,493,554,587,554,493,440,440,391,369,
391,391,391,440,493,440,391,369,369,293,293,493,493,493,554,587,554,493,440,440,
391,369,391,391,329,369,391,369,329,293,369,369,329,369,369,329,293,293,329,369,
391,391,391,391,391,369,329,329,220,293,329,369,369,329,369,329,293,293,293,493,
493,493,554,587,554,493,440,440,391,369,391,391,391,440,493,440,391,369,369,293,
293,493,493,493,554,587,554,493,440,440,391,369,391,391,329,369,391,369,329,293,
369,329,369,391,369,329,293,293,329,369,391,391,391,391,391,369,329,329,220,293,
329,369,369,329,369,329,293,293,293,493,493,493,554,587,554,493,440,440,391,369,
391,391,391,440,493,440,391,369,369,293,293,493,493,493,554,587,554,493,440,440,
391,369,391,391,329,369,391,369,329,293,369,329,369,391,369,329,293,293,329,369,
391,391,391,391,391,369,329,329,220,293,329,369,369,329,369,329,293,293,293,493,
493,493,554,587,554,493,440,440,391,369,391,391,391,440,493,440,391,369,369,293,
293,493,493,493,554,587,554,493,440,440,391,369,391,391,329,369,391,369,329,293};

const  uint16_t duration[280]={1950,236,236,236,236,236,696,1671,473,236,696,1393,236,236,236,236,696,1393,236,473,
236,835,1393,236,473,236,835,1393,835,835,1114,236,236,236,236,236,835,1393,557,236,
835,1114,236,236,236,236,236,557,1114,473,236,557,1114,236,236,236,236,236,557,1114,
473,236,557,835,236,236,236,236,236,835,696,975,236,236,236,236,696,1671,473,236,
696,1393,236,236,236,236,696,1393,236,473,236,835,1393,236,473,236,835,1393,835,835,
1114,236,236,236,236,236,835,1393,557,236,835,1114,236,236,236,236,236,557,1114,473,
236,557,1114,236,236,236,236,236,557,1114,473,236,557,835,236,236,236,236,236,835,
1950,236,236,236,236,236,696,1671,473,236,696,1393,236,236,236,236,696,1393,236,473,
236,835,1393,236,473,236,835,1393,835,835,1114,236,236,236,236,236,835,1393,557,236,
835,1114,236,236,236,236,236,557,1114,473,236,557,1114,236,236,236,236,236,557,1114,
473,236,557,835,236,236,236,236,236,696,1950,236,236,236,236,236,696,1671,473,236,
696,1393,236,236,236,236,696,1393,236,473,236,835,1393,236,473,236,835,1393,835,835,
1114,236,236,236,236,236,835,1393,557,236,835,1114,236,236,236,236,236,557,1114,473,
236,557,1114,236,236,236,236,236,557,1114,473,236,557,835,236,236,236,236,236,835};

const  uint16_t del_time[280]={2167,309,309,309,309,309,927,1857,618,309,927,1547,309,309,309,309,927,1547,309,618,
309,928,1547,309,618,309,928,1856,928,928,1238,309,309,309,309,309,928,1856,619,309,
928,1238,309,309,309,309,309,928,1857,618,309,928,1238,309,309,309,309,309,928,1857,
618,309,928,1237,309,309,309,309,309,3714,927,1547,309,309,309,309,927,1857,618,309,
927,1547,309,309,309,309,927,1547,309,618,309,928,1547,309,618,309,928,1856,928,928,
1238,309,309,309,309,309,928,1856,619,309,928,1238,309,309,309,309,309,928,1857,618,
309,928,1238,309,309,309,309,309,928,1857,618,309,928,1237,309,309,309,309,309,3714,
2167,309,309,309,309,309,927,1857,618,309,927,1547,309,309,309,309,927,1547,309,618,
309,928,1547,309,618,309,928,1856,928,928,1238,309,309,309,309,309,928,1856,619,309,
928,1238,309,309,309,309,309,928,1857,618,309,928,1238,309,309,309,309,309,928,1857,
618,309,928,1237,309,309,309,309,309,3714,2167,309,309,309,309,309,927,1857,618,309,
927,1547,309,309,309,309,927,1547,309,618,309,928,1547,309,618,309,928,1856,928,928,
1238,309,309,309,309,309,928,1856,619,309,928,1238,309,309,309,309,309,928,1857,618,
309,928,1238,309,309,309,309,309,928,1857,618,309,928,1237,309,309,309,309,309,928};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(cnt<280)
	  {
		  MX_TIM3_Init();
		  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 );
		 LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
		 LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
		 LL_TIM_EnableCounter(TIM3);
	  unsigned long ARR_val=(1.0/freq[cnt])*72*1000000/(double)presc;
	  TIM3->ARR =ARR_val;
	  LL_TIM_OC_SetCompareCH1(TIM3, (uint16_t)(ARR_val/2.0));
	//  cnt=(cnt==8)? 0: cnt+1;
	  LL_mDelay(duration[cnt]/2);
	  TIM3_deinit();
	  LL_mDelay((del_time[cnt]-duration[cnt])/2);
	  cnt++;
	  }else{
		  TIM3_deinit();
	  }

    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 72;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM3 GPIO Configuration
  PA6   ------> TIM3_CH1
  PA7   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE0);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_FLOATING);

}

/* USER CODE BEGIN 4 */
static void TIM3_deinit(void)
{
	LL_GPIO_DeInit(GPIOA);
	LL_GPIO_DeInit(GPIOA);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
