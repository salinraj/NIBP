/**
  ******************************************************************************
  * @file    ADC/ADC_AnalogWatchdog/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-December-2014
  * @brief   This example provides a short description of how to use the ADC
  *          peripheral to perform conversions with analog watchdog and 
  *          interruptions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup ADC_AnalogWatchdog
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Note: This example, on some other STM32 boards, is performing              */
/*       DAC handler declaration here.                                        */
/*       On STM32F103RB-Nucleo, the device has no DAC available,              */
/*       therefore analog signal must be supplied externally.                 */

uint16_t  PressureSensValue,PressureSensValue2,CurrentPos=0, PressureOldValue, Last_Position, MAP_Position, SysPulse ,DiaPulse, Pulse_Difference, Pulse_Difference_Old;
volatile __IO uint16_t   PressureAmplitude[ADCCONVERTEDVALUES_BUFFER_SIZE];
volatile __IO uint16_t   PulseAmplitude[ADCCONVERTEDVALUES_BUFFER_SIZE];
/* Variable to report ADC analog watchdog status:   */
/*   RESET <=> voltage into AWD window   */
/*   SET   <=> voltage out of AWD window */
//uint8_t         ubAnalogWatchdogStatus = RESET;  /* Set into analog watchdog interrupt callback */

/* Variables to manage push button on board: interface between ExtLine interruption and main program */
uint8_t         ubUserButtonClickCount = 0;      /* Count number of clicks: Incremented after User Button interrupt */
__IO uint8_t    ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */
#define USERBUTTON_CLICK_COUNT_MAX     ((uint32_t)    4)    /* Maximum value of variable "UserButtonClickCount" */


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
//static void ADC_Config(void);
void SendDecimal(uint16_t value);
#if defined(ADC_TRIGGER_FROM_TIMER)
static void TIM_Config(void);


#endif /* ADC_TRIGGER_FROM_TIMER */
uint8_t aTxBuffer[] = " ****UART_TwoBoards_ComIT**** ";
uint8_t aRxBuffer[RXBUFFERSIZE];
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);



UART_HandleTypeDef UartHandle;
__IO ITStatus UartReady = RESET;
uint8_t CommandToStart[RXBUFFERSIZE];
uint16_t MaxPulseAmplitude=0;
uint16_t MeanPressure=0;
uint16_t SysPressure=0;
uint16_t DiaPressure=0;


void Sendnewline(void);
void Sendspace(void);
void Sendstring_mean(void);
void Sendstring_Sys(void);
void Sendstring_Dia(void);

void Motor_IO_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 64 MHz */
  SystemClock_Config();
  
  /*## Configure peripherals #################################################*/
  
  /* Initialize LED on board */
  BSP_LED_Init(LED2);
  
  /* Configure User push-button in Interrupt mode */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  
  /* Configure the ADC peripheral */
  //ADC_Config();
  /*uart configuration*/
	
	
	 UartHandle.Instance        = USARTx;

  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
     Error_Handler();
  }
	
// 	
// 	
//   /* Run the ADC calibration */  
//   if (HAL_ADCEx_Calibration_Start(&AdcHandle) != HAL_OK)
//   {
//     /* Calibration Error */
//     Error_Handler();
//   }

// #if defined(ADC_TRIGGER_FROM_TIMER)
//   /* Configure the TIM peripheral */
//   TIM_Config();
// #endif /* ADC_TRIGGER_FROM_TIMER */

//   /* Note: This example, on some other STM32 boards, is performing            */
//   /*       DAC configuration here.                                            */
//   /*       On STM32F103RB-Nucleo, the device has no DAC available,            */
//   /*       therefore analog signal must be supplied externally.               */

//   /*## Enable peripherals ####################################################*/
// #if defined(ADC_TRIGGER_FROM_TIMER)
//   /* Timer enable */
//   if (HAL_TIM_Base_Start(&TimHandle) != HAL_OK)
//   {
//     /* Counter Enable Error */
//     Error_Handler();
//   }
// #endif /* ADC_TRIGGER_FROM_TIMER */

//   /* Note: This example, on some other STM32 boards, is performing            */
//   /*       DAC signal generation here.                                        */
//   /*       On STM32F103RB-Nucleo, the device has no DAC available,            */
//   /*       therefore analog signal must be supplied externally.               */

//   /*## Start ADC conversions #################################################*/
//   
//   /* Start ADC conversion on regular group with transfer by DMA */
//   if (HAL_ADC_Start_DMA(&AdcHandle,
//                         (uint32_t *)aADCxConvertedValues,
//                         ADCCONVERTEDVALUES_BUFFER_SIZE
//                        ) != HAL_OK)
//   {
//     /* Start Error */
//     Error_Handler();
//   }
//   
//     if(HAL_UART_Receive_IT(&UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   

Motor_IO_Init();
MOTOR_OFF();
Valve_Open();
  /* Infinite loop */
  while (1)
  {
		
	  MOTOR_ON();
		Valve_Close();
//  		HAL_Delay(4100);
//  		Valve_Open();
		memset(PressureAmplitude , 0, ADCCONVERTEDVALUES_BUFFER_SIZE);
		memset(PulseAmplitude , 0, ADCCONVERTEDVALUES_BUFFER_SIZE);
		while((PressureSensValue=ADC_Get_Pressure()* 0.052)<140)
		{
	PressureSensValue2=ADC_Get_Pulse();
	
	PressureAmplitude[CurrentPos] = PressureSensValue2 * 0.052; 
	PulseAmplitude[CurrentPos] = PressureSensValue;
	SendDecimal(PressureSensValue2);
			Sendspace();
	SendDecimal(PressureSensValue);		
			Sendnewline();
			
	
		
	HAL_Delay(100);
	MOTOR_OFF();
		HAL_Delay(20);
	MOTOR_ON();
		}
		PressureOldValue=PressureSensValue;
		MOTOR_OFF();
		
		
		
		////////////////////////////////////////reducing pressure 5mmhg per second
		
		
		while((PressureSensValue=ADC_Get_Pressure()* 0.052)>50)
		{
		if(PressureSensValue>(PressureOldValue-4))
		{
		Valve_Open();
		}
		else
		{
			Valve_Close();
			PressureOldValue=PressureSensValue;
			PressureAmplitude[CurrentPos]= PressureSensValue;                //storing pressure amplitude to the buffer
			PressureSensValue2=ADC_Get_Pulse();
			PulseAmplitude[CurrentPos] = PressureSensValue2;
			
// 			SendDecimal(PressureSensValue2);
// 			Sendspace();
// 			SendDecimal(PressureSensValue);		
// 			Sendnewline();
			HAL_Delay(200);
						PressureSensValue2=ADC_Get_Pulse();
					if((PulseAmplitude[CurrentPos])<PressureSensValue2)
						{
							PulseAmplitude[CurrentPos] = PressureSensValue2;
						}
// 			SendDecimal(PressureSensValue2);
// 			Sendspace();
// 			SendDecimal(PressureSensValue);		
// 			Sendnewline();
			HAL_Delay(200);
						PressureSensValue2=ADC_Get_Pulse();
					if((PulseAmplitude[CurrentPos])<PressureSensValue2)
						{
							PulseAmplitude[CurrentPos] = PressureSensValue2;
						}
// 			SendDecimal(PressureSensValue2);
// 			Sendspace();
// 			SendDecimal(PressureSensValue);		
// 			Sendnewline();
						
						
// 			HAL_Delay(200);
// 						PressureSensValue2=ADC_Get_Pulse();
// 					if((PulseAmplitude[CurrentPos])<PressureSensValue2)
// 						{
// 							PulseAmplitude[CurrentPos] = PressureSensValue2;
// 						}
						
						
// 			SendDecimal(PressureSensValue2);
// 			Sendspace();
// 	 		SendDecimal(PressureSensValue);		
// 			Sendnewline();
			HAL_Delay(200);
						PressureSensValue2=ADC_Get_Pulse();
					if((PulseAmplitude[CurrentPos])<PressureSensValue2)
						{
							PulseAmplitude[CurrentPos] = PressureSensValue2;
						}
// 			SendDecimal(PressureSensValue2);
// 			Sendspace();
// 			SendDecimal(PressureSensValue);		
// 			Sendnewline();
			HAL_Delay(200);
						PressureSensValue2=ADC_Get_Pulse();
					if((PulseAmplitude[CurrentPos])<PressureSensValue2)
						{
							PulseAmplitude[CurrentPos] = PressureSensValue2;
						}
				SendDecimal(PulseAmplitude[CurrentPos]);
				Sendspace();
				SendDecimal(PressureSensValue);		
				Sendnewline();
			HAL_Delay(200);
				CurrentPos++;
				if(CurrentPos>ADCCONVERTEDVALUES_BUFFER_SIZE)
				{
					CurrentPos=0;
				
				}


						
						
						
		}
	}
		Last_Position = CurrentPos;
	MaxPulseAmplitude=0;
	
		Sendnewline();
		Sendnewline();
	
// 	 	for(CurrentPos=0 ; CurrentPos<Last_Position ; CurrentPos++)
// 	{
// 		SendDecimal(PulseAmplitude[CurrentPos]);
// 		Sendspace();
// 		SendDecimal(PressureAmplitude[CurrentPos]);	
// 		Sendnewline();
// 	}
// 	
// 	
	
	
	
	
	
	
	
	///**********   Finding Mean Pressure    ********************
	
 		for(CurrentPos=2 ; CurrentPos<Last_Position ; CurrentPos++)
	{
		
		if(MaxPulseAmplitude<PulseAmplitude[CurrentPos])
		{	
			MaxPulseAmplitude=PulseAmplitude[CurrentPos];
			MeanPressure=PressureAmplitude[CurrentPos];
			MAP_Position = CurrentPos;
		}
	}
	
	CurrentPos=0;	

	Sendstring_mean();
					Sendspace();
				SendDecimal(MeanPressure);		
				Sendnewline();
	
	
	
	///********      Finding Systolic Pressure    ************
	SysPulse = MaxPulseAmplitude *0.54;
	Pulse_Difference_Old=0xffff;
		for(CurrentPos=2 ; CurrentPos < MAP_Position ; CurrentPos++)	
			{
			if(PulseAmplitude[CurrentPos]<SysPulse)
			{
				Pulse_Difference= SysPulse - PulseAmplitude[CurrentPos];
			}
			else
			{
				Pulse_Difference= PulseAmplitude[CurrentPos] - SysPulse;
			}
			
			if(Pulse_Difference_Old>=Pulse_Difference)
			{
				Pulse_Difference_Old=Pulse_Difference;
				SysPressure = PressureAmplitude[CurrentPos];
				
			}
			
			}	
			Sendstring_Sys();
					Sendspace();
				SendDecimal(SysPressure);		
				Sendnewline();
		
			
			
		///********      Finding Diastolic Pressure    ************		
			
				DiaPulse = MaxPulseAmplitude *0.72;
	Pulse_Difference_Old=0xffff;
		for(CurrentPos=MAP_Position ; CurrentPos < Last_Position ; CurrentPos++)	
			{
			if(PulseAmplitude[CurrentPos]<DiaPulse)
			{
				Pulse_Difference= DiaPulse - PulseAmplitude[CurrentPos];
			}
			else
			{
				Pulse_Difference= PulseAmplitude[CurrentPos] - DiaPulse;
			}
			
			if(Pulse_Difference_Old>=Pulse_Difference)
			{
				Pulse_Difference_Old=Pulse_Difference;
				DiaPressure = PressureAmplitude[CurrentPos];
				
			}
			
			}
			
				Sendstring_Dia();
					Sendspace();
				SendDecimal(DiaPressure);		
				Sendnewline();
		

			
			
			
			
		
		Valve_Open();
	HAL_Delay(10000);
		
		
		
    /* Turn-on/off LED2 in function of ADC conversion result */
    /*  - Turn-off if voltage is into AWD window */ 
    /*  - Turn-on if voltage is out of AWD window */

    /* Variable of analog watchdog status is set into analog watchdog         */
    /* interrupt callback         
		*/
memset(CommandToStart,0,10);
// while(1)
// {
// uint8_t i;
// //	uint16_t PressureSensValuefinal=0;
// HAL_Delay(1000);
// PressureSensValue=0;	
// if(CommandToStart[0]=='s' && CommandToStart[1]=='t' && CommandToStart[2]=='a' && CommandToStart[3]=='r' && CommandToStart[4]=='t')
// {
// memset(CommandToStart,0,10);
// 	for( i=0;i<25;i++)
// {
//  PressureSensValue += aADCxConvertedValues[i];
// }	
// PressureSensValue=PressureSensValue/25;	
// SendDecimal(PressureSensValue);
// //while(1);
// }

// }

    /* Note: This example, on some other STM32 boards, is performing          */
    /*       DAC signal generation here.                                      */
    /*       On STM32F103RB-Nucleo, the device has no DAC available,          */
    /*       therefore analog signal must be supplied externally.             */
// 		memset(PressureAmplitude , 0, ADCCONVERTEDVALUES_BUFFER_SIZE);
// 		memset(PulseAmplitude , 0, ADCCONVERTEDVALUES_BUFFER_SIZE);
// 		

// 	for(CurrentPos=0 ; CurrentPos<ADCCONVERTEDVALUES_BUFFER_SIZE ; CurrentPos++)
// 	{
// 	PressureSensValue=ADC_Get_Pressure();	
// 	PressureSensValue2=ADC_Get_Pulse();
// 	
// 	PressureAmplitude[CurrentPos] = PressureSensValue2 * 0.052; 
// 	PulseAmplitude[CurrentPos] = PressureSensValue;
// 	
// 	HAL_Delay(100);
// 		
// 	} 
// 	
// 		for(CurrentPos=120 ; CurrentPos<ADCCONVERTEDVALUES_BUFFER_SIZE ; CurrentPos++)
// 	{
// 		
// 		if(MaxPulseAmplitude<PulseAmplitude[CurrentPos])
// 		{	
// 			MaxPulseAmplitude=PulseAmplitude[CurrentPos];
// 			MeanPressure=PressureAmplitude[CurrentPos];
// 		}
// 	}

// 		
// 	for(CurrentPos=0 ; CurrentPos<ADCCONVERTEDVALUES_BUFFER_SIZE ; CurrentPos++)
// 	{
// 		SendDecimal(PulseAmplitude[CurrentPos]);
// 		Sendspace();
// 		SendDecimal(PressureAmplitude[CurrentPos]);	
// 		Sendnewline();
// 	}
// 	
// 	Sendstring_mean();
// 	Sendspace();
// 	SendDecimal(MeanPressure);
// 	
// 	
	
	
	
// 	//  calibration starts
// 	while(1)
// 	{
// 		PressureSensValue=ADC_Get_Pressure();	
// 	PressureSensValue2=ADC_Get_Pulse();
// 		
// 	PressureSensValue2= PressureSensValue2 * 0.056 ;
// 	SendDecimal(PressureSensValue);	
// 	Sendspace();
// 	SendDecimal(PressureSensValue2);	
// 	Sendnewline();
// 	
// 		HAL_Delay(100);
// 		
// 	}
// 	
// 		
// 	//  calibration ends
	
	
	
	
// 		for(CurrentPos=0 ; CurrentPos<ADCCONVERTEDVALUES_BUFFER_SIZE ; CurrentPos++)
// 	
// 	{
// 	SendDecimal(PulseAmplitude[CurrentPos]);		
// 	}
		
//HAL_Delay(3000);	
		
  }
}

void SendDecimal(uint16_t value)
{
 static char digit[4];
		digit[0] = (unsigned int)(value/1000);									 // Calculate digit1 of ADC_value
		digit[1] = (unsigned int)((value - digit[0]*1000)/100);						 // Calculate digit2 of ADC_value
		digit[2] = (unsigned int)((value - (digit[0]*1000+digit[1]*100))/10);			 // Calculate digit3 of ADC_value
		digit[3] = (unsigned int)((value - (digit[0]*1000+digit[1]*100+digit[2]*10))/1);	 // Calculate digit4 of ADC_value
		digit[0]=digit[0]+0x30;
		digit[1]=digit[1]+0x30;
		digit[2]=digit[2]+0x30;
		digit[3]=digit[3]+0x30;

		
		//LCD_Printf(digit2);
//LCD_Printf(digit3);
//LCD_Printf(digit4);
	
	  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)digit, 4)!= HAL_OK)
  {
    Error_Handler();
	}
	
		  while (UartReady != SET)
  {
  } 
	UartReady = RESET;
	
// 		  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)digit+1, 1)!= HAL_OK)
//   {
//     Error_Handler();
//   }
// 		  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)digit+2, 1)!= HAL_OK)
//   {
//     Error_Handler();
//   }
// 		  if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)digit+3, 1)!= HAL_OK)
//   {
//     Error_Handler();
//   }
	
}

void Sendspace(void)
{
	  
	uint8_t value[]="    ";
	
	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)value, 4)!= HAL_OK)
  {
    Error_Handler();
	}
			  while (UartReady != SET)
  {
  } 
	UartReady = RESET;

}

void Sendnewline(void)
{
	  
	uint8_t value[]="\n\r";
	
	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)value, 2)!= HAL_OK)
  {
    Error_Handler();
	}
			  while (UartReady != SET)
  {
  } 
	UartReady = RESET;

}

void Sendstring_mean(void)
{
	  
	uint8_t value[]="mean";
	
	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)value, 4)!= HAL_OK)
  {
    Error_Handler();
	}
			  while (UartReady != SET)
  {
  } 
	UartReady = RESET;

}


void Sendstring_Sys(void)
{
	  
	uint8_t value[]="Sys";
	
	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)value, 3)!= HAL_OK)
  {
    Error_Handler();
	}
			  while (UartReady != SET)
  {
  } 
	UartReady = RESET;

}

void Sendstring_Dia(void)
{
	  
	uint8_t value[]="Dia";
	
	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t*)value, 3)!= HAL_OK)
  {
    Error_Handler();
	}
			  while (UartReady != SET)
  {
  } 
	UartReady = RESET;

}















void Motor_IO_Init(void)
{ 

	GPIO_InitTypeDef  gpioinitstruct = {0};

  /* LCD_CS_GPIO and LCD_DC_GPIO Periph clock enable */
__HAL_RCC_GPIOB_CLK_ENABLE();
  
  /* Configure LCD_CS_PIN pin: LCD Card CS pin */
  gpioinitstruct.Pin    = Motor_PIN ;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Speed  = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(Motor_PORT, &gpioinitstruct);
  gpioinitstruct.Pin    = Valve_PIN ;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Speed  = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(Motor_PORT, &gpioinitstruct);
      
}

















/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}


/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
// static void ADC_Config(void)
// {
//   ADC_ChannelConfTypeDef   sConfig;
//   ADC_AnalogWDGConfTypeDef AnalogWDGConfig;
//   
//   /* Configuration of ADCx init structure: ADC parameters and regular group */
//   AdcHandle.Instance = ADCx;
//   
//   AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
//   AdcHandle.Init.ScanConvMode          = ADC_SCAN_DISABLE;              /* Sequencer disabled (ADC conversion on only 1 channel: channel set on rank 1) */
// #if defined ADC_TRIGGER_FROM_TIMER
//   AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 conversion at each conversion trig */
// #else
//   AdcHandle.Init.ContinuousConvMode    = ENABLE;                        /* Continuous mode to have maximum conversion speed (no delay between conversions) */
// #endif
//   AdcHandle.Init.NbrOfConversion       = 1;                             /* Parameter discarded because sequencer is disabled */
//   AdcHandle.Init.DiscontinuousConvMode = DISABLE;                       /* Parameter discarded because sequencer is disabled */
//   AdcHandle.Init.NbrOfDiscConversion   = 1;                             /* Parameter discarded because sequencer is disabled */
// #if defined ADC_TRIGGER_FROM_TIMER
//   AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_Tx_TRGO;  /* Trig of conversion start done by external event */
// #else
//   AdcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;            /* Software start to trig the 1st conversion manually, without external event */
// #endif

//   if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
//   {
//     /* ADC initialization error */
//     Error_Handler();
//   }
//   
//   /* Configuration of channel on ADCx regular group on sequencer rank 1 */
//   /* Note: Considering IT occurring after each ADC conversion if ADC          */
//   /*       conversion is out of the analog watchdog window selected (ADC IT   */
//   /*       enabled), select sampling time and ADC clock with sufficient       */
//   /*       duration to not create an overhead situation in IRQHandler.        */
//   sConfig.Channel      = ADCx_CHANNELa;
//   sConfig.Rank         = ADC_REGULAR_RANK_1;
//   sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
//   
//   if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
//   {
//     /* Channel Configuration Error */
//     Error_Handler();
//   }
//   
//   /* Set analog watchdog thresholds in order to be between steps of DAC       */
//   /* voltage.                                                                 */
//   /*  - High threshold: between DAC steps 1/2 and 3/4 of full range:          */
//   /*                    5/8 of full range (4095 <=> Vdda=3.3V): 2559<=> 2.06V */
//   /*  - Low threshold:  between DAC steps 0 and 1/4 of full range:            */
//   /*                    1/8 of full range (4095 <=> Vdda=3.3V): 512 <=> 0.41V */

//   /* Analog watchdog 1 configuration */
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
// }

// #if defined(ADC_TRIGGER_FROM_TIMER)
// /**
//   * @brief  TIM configuration
//   * @param  None
//   * @retval None
//   */
// static void TIM_Config(void)
// {
//   TIM_MasterConfigTypeDef master_timer_config;
//   RCC_ClkInitTypeDef clk_init_struct = {0};       /* Temporary variable to retrieve RCC clock configuration */
//   uint32_t latency;                               /* Temporary variable to retrieve Flash Latency */
//   
//   uint32_t timer_clock_frequency = 0;             /* Timer clock frequency */
//   uint32_t timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
//   
//   /* Configuration of timer as time base:                                     */ 
//   /* Caution: Computation of frequency is done for a timer instance on APB1   */
//   /*          (clocked by PCLK1)                                              */
//   /* Timer period can be adjusted by modifying the following constants:       */
//   /* - TIMER_FREQUENCY: timer frequency (unit: Hz).                           */
//   /* - TIMER_FREQUENCY_RANGE_MIN: timer minimum frequency (unit: Hz).         */
//   
//   /* Retrieve timer clock source frequency */
//   HAL_RCC_GetClockConfig(&clk_init_struct, &latency);
//   /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
//   /* clock source.                                                            */
//   if (clk_init_struct.APB1CLKDivider == RCC_HCLK_DIV1)
//   {
//     timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
//   }
//   else
//   {
//     timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
//   }
//   
//   /* Timer prescaler calculation */
//   /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
//   timer_prescaler = (timer_clock_frequency / (TIMER_PRESCALER_MAX_VALUE * TIMER_FREQUENCY_RANGE_MIN)) +1;
//   
//   /* Set timer instance */
//   TimHandle.Instance = TIMx;
//   
//   /* Configure timer parameters */
//   TimHandle.Init.Period            = ((timer_clock_frequency / (timer_prescaler * TIMER_FREQUENCY)) - 1);
//   TimHandle.Init.Prescaler         = (timer_prescaler - 1);
//   TimHandle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
//   TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
//   TimHandle.Init.RepetitionCounter = 0x0;
//   
//   if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
//   {
//     /* Timer initialization Error */
//     Error_Handler();
//   }

//   /* Timer TRGO selection */
//   master_timer_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
//   master_timer_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

//   if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &master_timer_config) != HAL_OK)
//   {
//     /* Timer TRGO selection Error */
//     Error_Handler();
//   }
//   
// }
// #endif /* ADC_TRIGGER_FROM_TIMER */

/* Note: This example, on some other STM32 boards, is performing              */
/*       DAC configuration here.                                              */
/*       On STM32F103RB-Nucleo, the device has no DAC available,              */
/*       therefore analog signal must be supplied externally.                 */

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_BUTTON_PIN)
  {
    /* Set variable to report push button event to main program */
    ubUserButtonClickEvent = SET;
  
    /* Manage ubUserButtonClickCount to increment it circularly from 0 to     */
    /* maximum value defined                                                  */
    if (ubUserButtonClickCount < USERBUTTON_CLICK_COUNT_MAX)
    {
      ubUserButtonClickCount++;
    }      
    else
    {
      ubUserButtonClickCount=0;
    }
    
  }
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
// {

// }

// /**
//   * @brief  Conversion DMA half-transfer callback in non blocking mode 
//   * @param  hadc: ADC handle
//   * @retval None
//   */
// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
// {

// }

// /**
//   * @brief  Analog watchdog callback in non blocking mode. 
//   * @param  hadc: ADC handle
//   * @retval None
//   */
//   void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
// {
//   /* Set variable to report analog watchdog out of window status to main      */
//   /* program.                                                                 */
//   ubAnalogWatchdogStatus = SET;
// }

// /**
//   * @brief  ADC error callback in non blocking mode
//   *        (ADC conversion with interruption or transfer by DMA)
//   * @param  hadc: ADC handle
//   * @retval None
//   */
// void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
// {
//   /* In case of ADC error, call main error handler */
//   Error_Handler();
// }





/////////////////////////////////////////////////
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

  
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  uint8_t Count;
	UartReady = SET;
	for(Count=0;Count<RXBUFFERSIZE;Count++)
  CommandToStart[Count]=aRxBuffer[Count];
	memset(aRxBuffer,0,RXBUFFERSIZE);
	    if(HAL_UART_Receive_IT(UartHandle, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
  {
    Error_Handler();
  }
  
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
  while(1)
  {
    /* Toggle LED2 */
    BSP_LED_Toggle(LED2);
    HAL_Delay(500);
  }
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

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
