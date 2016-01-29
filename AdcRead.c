

/* Includes ------------------------------------------------------------------*/
#include "AdcRead.h"
//#include "syringepump.h"
#include "stm32f1xx_hal_tim.h"


/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup ADC_AnalogWatchdog
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define RANGE_12BITS                   ((uint32_t) 4095)    /* Max value with a full range of 12 bits */
#define USERBUTTON_CLICK_COUNT_MAX     ((uint32_t)    4)    /* Maximum value of variable "UserButtonClickCount" */

//#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t) 256)     /* Size of array containing ADC converted values */

#if defined(ADC_TRIGGER_FROM_TIMER)
#define TIMER_FREQUENCY                ((uint32_t) 1000)    /* Timer frequency (unit: Hz). With a timer 16 bits and time base freq min 1Hz, range is min=1Hz, max=32kHz. */
#define TIMER_FREQUENCY_RANGE_MIN      ((uint32_t)    1)    /* Timer minimum frequency (unit: Hz). With a timer 16 bits, maximum frequency will be 32000 times this value. */
#define TIMER_PRESCALER_MAX_VALUE      (0xFFFF-1)           /* Timer prescaler maximum value (0xFFFF for a timer 16 bits) */
#endif /* ADC_TRIGGER_FROM_TIMER */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ADC handler declaration */
ADC_HandleTypeDef    AdcHandle;

#if defined(ADC_TRIGGER_FROM_TIMER)
/* TIM handler declaration */
TIM_HandleTypeDef    TimHandle;
#endif /* ADC_TRIGGER_FROM_TIMER */

/* Note: This example, on some other STM32 boards, is performing              */
/*       DAC handler declaration here.                                        */
/*       On STM32F103RB-Nucleo, the device has no DAC available,              */
/*       therefore analog signal must be supplied externally.                 */

/* Variable containing ADC conversions results */
//volatile __IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* Variable to report ADC analog watchdog status:   */
/*   RESET <=> voltage into AWD window   */
/*   SET   <=> voltage out of AWD window */
uint8_t         ubAnalogWatchdogStatus = RESET;  /* Set into analog watchdog interrupt callback */

/* Variables to manage push button on board: interface between ExtLine interruption and main program */
//uint8_t         ubUserButtonClickCount = 0;      /* Count number of clicks: Incremented after User Button interrupt */
//__IO uint8_t    ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */






void ADC_Config_CH1(void)
{
  ADC_ChannelConfTypeDef   sConfig;
 // ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
  
  /* Configuration of ADCx init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADCx;
  
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
//#if defined ADC_TRIGGER_FROM_TIMER
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
//#else
//  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode to have maximum conversion speed (no delay between conversions) */
//#endif
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
#if defined ADC_TRIGGER_FROM_TIMER
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_Tx_TRGO;  /* Trig of conversion start done by external event */
#else
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
#endif

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion if ADC          */
  /*       conversion is out of the analog watchdog window selected (ADC IT   */
  /*       enabled), select sampling time and ADC clock with sufficient       */
  /*       duration to not create an overhead situation in IRQHandler.        */
  sConfig.Channel      = ADCx_CHANNELa;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	//sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
  /* Set analog watchdog thresholds in order to be between steps of DAC       */
  /* voltage.                                                                 */
  /*  - High threshold: between DAC steps 1/2 and 3/4 of full range:          */
  /*                    5/8 of full range (4095 <=> Vdda=3.3V): 2559<=> 2.06V */
  /*  - Low threshold:  between DAC steps 0 and 1/4 of full range:            */
  /*                    1/8 of full range (4095 <=> Vdda=3.3V): 512 <=> 0.41V */

  /* Analog watchdog 1 configuration */
//   AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
//   AnalogWDGConfig.Channel = ADCx_CHANNELa;
//   AnalogWDGConfig.ITMode = ENABLE;
//   AnalogWDGConfig.HighThreshold = (RANGE_12BITS * 5/8);
//   AnalogWDGConfig.LowThreshold = (RANGE_12BITS * 1/8);
//   if (HAL_ADC_AnalogWDGConfig(&AdcHandle, &AnalogWDGConfig) != HAL_OK)
//   {
//     /* Channel Configuration Error */
//     Error_Handler();
//   }
//   
}




void ADC_Config_CH2(void)
{
  ADC_ChannelConfTypeDef   sConfig;
 // ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
  
  /* Configuration of ADCx init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADCx;
  
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
//#if defined ADC_TRIGGER_FROM_TIMER
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
//#else
//  AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode to have maximum conversion speed (no delay between conversions) */
//#endif
  AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
  AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
#if defined ADC_TRIGGER_FROM_TIMER
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_Tx_TRGO;  /* Trig of conversion start done by external event */
#else
  AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
#endif

  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }
  
  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion if ADC          */
  /*       conversion is out of the analog watchdog window selected (ADC IT   */
  /*       enabled), select sampling time and ADC clock with sufficient       */
  /*       duration to not create an overhead situation in IRQHandler.        */
  sConfig.Channel      = ADCx_CHANNELb;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	//sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  
  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
  
  /* Set analog watchdog thresholds in order to be between steps of DAC       */
  /* voltage.                                                                 */
  /*  - High threshold: between DAC steps 1/2 and 3/4 of full range:          */
  /*                    5/8 of full range (4095 <=> Vdda=3.3V): 2559<=> 2.06V */
  /*  - Low threshold:  between DAC steps 0 and 1/4 of full range:            */
  /*                    1/8 of full range (4095 <=> Vdda=3.3V): 512 <=> 0.41V */

  /* Analog watchdog 1 configuration */
//   AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
//   AnalogWDGConfig.Channel = ADCx_CHANNELa;
//   AnalogWDGConfig.ITMode = ENABLE;
//   AnalogWDGConfig.HighThreshold = (RANGE_12BITS * 5/8);
//   AnalogWDGConfig.LowThreshold = (RANGE_12BITS * 1/8);
//   if (HAL_ADC_AnalogWDGConfig(&AdcHandle, &AnalogWDGConfig) != HAL_OK)
//   {
//     /* Channel Configuration Error */
//     Error_Handler();
//   }
//   
}






#if defined(ADC_TRIGGER_FROM_TIMER)
/**
  * @brief  TIM configuration
  * @param  None
  * @retval None
  */
static void TIM_Config(void)
{
  TIM_MasterConfigTypeDef master_timer_config;
  RCC_ClkInitTypeDef clk_init_struct = {0};       /* Temporary variable to retrieve RCC clock configuration */
  uint32_t latency;                               /* Temporary variable to retrieve Flash Latency */
  
  uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
 // uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
  uint32_t timer_prescaler = 10;
  /* Configuration of timer as time base:                                     */ 
  /* Caution: Computation of frequency is done for a timer instance on APB1   */
  /*          (clocked by PCLK1)                                              */
  /* Timer period can be adjusted by modifying the following constants:       */
  /* - TIMER_FREQUENCY: timer frequency (unit: Hz).                           */
  /* - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency (unit: Hz).         */
  
  /* Retrieve timer clock source frequency */
  HAL_RCC_GetClockConfig(&clk_init_struct, &latency);
  /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
  /* clock source.                                                            */
  if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
  }
  
  /* Timer prescaler calculation */
  /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
  timer_prescaler = (timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;
  
  /* Set timer instance */
  TimHandle.Instance = TIMx;
  
  /* Configure timer parameters */
  TimHandle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * TIMER_FREQUENCY)) - 1);
  TimHandle.Init.Prescaler         = (timer_prescaler - 1);
  TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0x0;
  
  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Timer initialization Error */
    Error_Handler();
  }

  /* Timer TRGO selection */
  master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
  master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &master_timer_config) != HAL_OK)
  {
    /* Timer TRGO selection Error */
    Error_Handler();
  }
  
}
#endif /* ADC_TRIGGER_FROM_TIMER */


/* Note: This example, on some other STM32 boards, is performing              */
/*       DAC configuration here.                                              */
/*       On STM32F103RB-Nucleo, the device has no DAC available,              */
/*       therefore analog signal must be supplied externally.                 */

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//   if (GPIO_Pin == USER_BUTTON_PIN)
//   {
//     /* Set variable to report push button event to main program */
//     ubUserButtonClickEvent = SET;
//   
//     /* Manage ubUserButtonClickCount to increment it circularly from 0 to     */
//     /* maximum value defined                                                  */
//     if (ubUserButtonClickCount < USERBUTTON_CLICK_COUNT_MAX)
//     {
//       ubUserButtonClickCount++;
//     }      
//     else
//     {
//       ubUserButtonClickCount=0;
//     }
//     
//   }
// }

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{

}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode 
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

/**
  * @brief  Analog watchdog callback in non blocking mode. 
  * @param  hadc: ADC handle
  * @retval None
  */
  void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  /* Set variable to report analog watchdog out of window status to main      */
  /* program.                                                                 */
  ubAnalogWatchdogStatus = SET;
}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with a potential error */
  
  /* In case of error, LED2 is toggling at a frequency of 1Hz */
 // while(1)
  {
    /* Toggle LED2 */
//    BSP_LED_Toggle(LED2);
    HAL_Delay(500);
  }
}


uint16_t ADC_Get_Pressure(void)
{
	uint16_t adc_value;
	uint16_t adc_valuetest=0;
 ADC_Config_CH1();
	
	
	  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
	
	#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Configure the TIM peripheral */
  TIM_Config();
#endif /* ADC_TRIGGER_FROM_TIMER */
	
	 /*## Enable peripherals ####################################################*/
#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Timer enable */
  if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK)
  {
    /* Counter Enable Error */
    Error_Handler();
  }
#endif /* ADC_TRIGGER_FROM_TIMER */
  /* Note: This example, on some other STM32 boards, is performing            */
  /*       DAC signal generation here.                                        */
  /*       On STM32F103RB-Nucleo, the device has no DAC available,            */
  /*       therefore analog signal must be supplied externally.               */

  /*## Start ADC conversions #################################################*/
  
  /* Start ADC conversion on regular group with transfer by DMA */
//   if (HAL_ADC_Start_DMA(&AdcHandle,
//                         (uint32_t *)aADCxConvertedValues,
//                         ADCCONVERTEDVALUES_BUFFER_SIZE
//                        ) != HAL_OK)
//   {
//     /* Start Error */
//     Error_Handler();
//   }
   if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	if (HAL_ADC_PollForConversion(&AdcHandle,10) != HAL_OK)
		  {
    /* Start Error */
    Error_Handler();
			}
	
	adc_value= HAL_ADC_GetValue(&AdcHandle);

	return(adc_value);

}



uint16_t ADC_Get_Pulse(void)
{
	uint16_t adc_value;
	uint16_t adc_valuetest=0;
 ADC_Config_CH2();
	
	
	  if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
	
	#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Configure the TIM peripheral */
  TIM_Config();
#endif /* ADC_TRIGGER_FROM_TIMER */
	
	 /*## Enable peripherals ####################################################*/
#if defined(ADC_TRIGGER_FROM_TIMER)
  /* Timer enable */
  if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK)
  {
    /* Counter Enable Error */
    Error_Handler();
  }
#endif /* ADC_TRIGGER_FROM_TIMER */
  /* Note: This example, on some other STM32 boards, is performing            */
  /*       DAC signal generation here.                                        */
  /*       On STM32F103RB-Nucleo, the device has no DAC available,            */
  /*       therefore analog signal must be supplied externally.               */

  /*## Start ADC conversions #################################################*/
  
  /* Start ADC conversion on regular group with transfer by DMA */
//   if (HAL_ADC_Start_DMA(&AdcHandle,
//                         (uint32_t *)aADCxConvertedValues,
//                         ADCCONVERTEDVALUES_BUFFER_SIZE
//                        ) != HAL_OK)
//   {
//     /* Start Error */
//     Error_Handler();
//   }
   if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }
	
	if (HAL_ADC_PollForConversion(&AdcHandle,10) != HAL_OK)
		  {
    /* Start Error */
    Error_Handler();
			}
	
	adc_value= HAL_ADC_GetValue(&AdcHandle);

	return(adc_value);

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif


