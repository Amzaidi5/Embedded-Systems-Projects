/** Robert 

This starter project: 
1. configured TIM3 as a  base timer .  Every 0.5 seconds there will ba an  update event (overflow) interrupt, 
2. configured TIM4 (channel 1) as Output Compared interrupt timer. Every 1/1000 second there is a capture.
3. Configured LCD.
4. configured USER BOTTON in EXTI mode. 
5. configured RNG
6. configured EEPROM emulator.
7. configured PC1 as the pin for EXTI 1. 


  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/main.c 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   This example describes how to configure and use GPIOs through 
  *          the STM32F4xx HAL API.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)


/* Private macro -------------------------------------------------------------*/

 

/* Private variables ---------------------------------------------------------*/
HAL_StatusTypeDef Hal_status;  //HAL_ERROR, HAL_TIMEOUT, HAL_OK, of HAL_BUSY 

TIM_HandleTypeDef    Tim3_Handle,Tim4_Handle;
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t Tim3_PrescalerValue,Tim4_PrescalerValue;

__IO uint16_t Tim4_CCR; // the pulse of the TIM4
int reset = 0; //determines if the external reset button was pushed
int best = 10000; //starts our best time at 10s
__O uint16_t factor = 0; //Used to find our reaction time 
int user = 0; //used to detect if the user button has been pushed
__IO uint8_t UBPressed = 0; //if user button is pressed, =1
__IO uint8_t extern_UBPressed=0; // if external button if pressed
__IO uint16_t OC_Count=0; // used to count our waiting time from State 1 to State 2

char lcd_buffer[14];    // LCD display buffer

RNG_HandleTypeDef Rng_Handle;
uint32_t random;

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t EEREAD;  //to practice reading the BESTRESULT save in the EE, for EE read/write, require uint16_t type




/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void EXTILine0_Config(void);

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);


void TIM3_Config(void);
void TIM4_Config(void);
void TIM4_OC_Config(void);

static void EXTILine1_Config(void); // configure the exti line1, for exterrnal button, using PB1



/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 /* This sample code shows how to use STM32F4xx GPIO HAL API to toggle PG13 
     IOs (connected to LED3 on STM32F429i-Discovery board) 
    in an infinite loop.
    To proceed, 3 steps are required: */

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
	
	 /* Configure the system clock to 180 MHz */
  SystemClock_Config();

	//configure use button as exti mode. 
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   // BSP_functions in stm32f429i_discovery.c
																			//Use this line, so do not need extiline0_config()
	
	/* Configure EXTI Line0 (connected to PA0 pin,  the USER 
	BUTTON ) in interrupt mode */
  //EXTILine0_Config();
	
	
  //Configure LED3 and LED4 ======================================
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
  //BSP_LED_Toggle(LED3);
	//BSP_LED_Toggle(LED4);
	
	
	//Configer timer =================================
	TIM3_Config();
	
	TIM4_Config();
	
	Tim4_CCR=500;       //  with clock counter freq as 500,000, this will make OC Freq as 1ms.
	TIM4_OC_Config();
 
	// ===========Config the GPIO for external interupt==============
	EXTILine1_Config();


//Configure LCD====================================================
		// Initialization steps :
		//     o Initialize the LCD using the LCD_Init() function.
		//    o Apply the Layer configuration using LCD_LayerDefaultInit() function    
		// 			o Select the LCD layer to be used using LCD_SelectLayer() function.
		//     o Enable the LCD display using LCD_DisplayOn() function.	
		//LCD_Init();
	BSP_LCD_Init();
	//BSP_LCD_LayerDefaultInit(uint16_t LayerIndex, uint32_t FB_Address);
	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
														// the LayerIndex may be 0 and 1. if is 2, then the LCD is dark.
	//BSP_LCD_SelectLayer(uint32_t LayerIndex);
	BSP_LCD_SelectLayer(0);
	//BSP_LCD_SetLayerVisible(0, ENABLE); //do not need this line.
	BSP_LCD_Clear(LCD_COLOR_RED);  //need this line, otherwise, the screen is ddark	
	BSP_LCD_DisplayOn();
 
	BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	
	LCD_DisplayString(8, 2, (uint8_t *)"Reaction Time: "); //Display Reaction time Title
	LCD_DisplayString(3, 2, (uint8_t *)"2TA4 Lab2 :("); //Display Lab 
	LCD_DisplayString(9, 2, (uint8_t *)"     "); //clear space for current reactioon time 
	LCD_DisplayString(10, 2, (uint8_t *)"Best Time: ");	//Display Best time Title
//	LCD_DisplayInt(7, 6, 123456789);
//	LCD_DisplayFloat(8, 6, 12.3456789, 4);


 //**************test random number *********************
	Rng_Handle.Instance=RNG;  //Everytime declare a Handle, need to assign its Instance a base address. like the timer handles.... 													
	
	HAL_RNG_Init(&Rng_Handle);
	HAL_RNG_GenerateRandomNumber (&Rng_Handle, &random); //Generates a random number and assigns it to random
	random = random %1500; //Used to make sure our random number is within 1.5-3 seconds approximately
	random = random +1500; //Used to make sure our random number is within 1.5-3 seconds approximately
	LCD_DisplayInt(13,2,random); //Displays random number on LCD
	
		
		
	
	
		EE_ReadVariable(VirtAddVarTab[0], &EEREAD); //Reads from the EEPROM the best time value	
		LCD_DisplayString(11,2,(uint8_t *)"     "); //Clears space on LCD for best time value
		LCD_DisplayInt(11, 2, EEREAD); //Displays best time value on LCD
	
	
	
//******************* use emulated EEPROM ====================================

	//Unlock the Flash Program Erase controller 
	HAL_FLASH_Unlock();
		
// EEPROM Init 
	//EE_Init();
	
 
	//test EEPROM----
	//EE_WriteVariable(VirtAddVarTab[0], 300);
	//EE_ReadVariable(VirtAddVarTab[0], &EEREAD);	
		//LCD_DisplayString(11,9,(uint8_t *)"     ");
		//LCD_DisplayInt(11, 9, EEREAD);		
 
 



  /* Infinite loop */
  while (1)
  {	
				
		
		
  }
	
	
	
	
}  

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
 
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  Configures EXTI Line0 (connected to PA0 pin) in interrupt mode
  * @param  None
  * @retval None
  */





void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		while (*ptr!=NULL)
    {
				BSP_LCD_DisplayChar(COLUMN(ColumnNumber),LINE(LineNumber), *ptr); //new version of this function need Xpos first. so COLUMN() is the first para.
				ColumnNumber++;
			 //to avoid wrapping on the same line and replacing chars 
				if ((ColumnNumber+1)*(((sFONT *)BSP_LCD_GetFont())->Width)>=BSP_LCD_GetXSize() ){
					ColumnNumber=0;
					LineNumber++;
				}
					
				ptr++;
		}
}

void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		sprintf(lcd_buffer,"%d",Number);
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}

void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint)
{  
  //here the LineNumber and the ColumnNumber are NOT  pixel numbers!!!
		char lcd_buffer[15];
		
		sprintf(lcd_buffer,"%.*f",DigitAfterDecimalPoint, Number);  //6 digits after decimal point, this is also the default setting for Keil uVision 4.74 environment.
	
		LCD_DisplayString(LineNumber, ColumnNumber, (uint8_t *) lcd_buffer);
}








// set up timer 3

void  TIM3_Config(void)
{

		/* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock /2) /10 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  Tim3_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
   
  /* Initialize TIM3 peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  Tim3_Handle.Init.Period = 5000 - 1;
  Tim3_Handle.Init.Prescaler = Tim3_PrescalerValue;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim3_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&Tim3_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}


// configure Timer4 base.
void  TIM4_Config(void)
{

		/* -----------------------------------------------------------------------
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock /2) /10 KHz) - 1
       
    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock 
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency  
  ----------------------------------------------------------------------- */  
  
  /* -----------------------------------------------------------------------
    TIM4 Configuration: Output Compare Timing Mode:
    
    In this example TIM4 input clock (TIM4CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM4CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM4CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM4 counter clock at 0.5 MHz, the prescaler is computed as follows:
       Prescaler = (TIM4CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /500000) - 1
                                              
    (if CCR1_Val=50,000   ,  CC1 update rate = TIM4 counter clock / CCR1_Val = 10 Hz
    ==> Toggling frequency = 5 Hz)
		To make the CC interupt happens every 1ms. need to play with the prescaler and the CCR1_Val
		if want to keep the timer counter frequency as 500,000Hz, the CCR1_Val need to me 500.

		 ----------------------------------------------------------------------- */ 	

/* Compute the prescaler value to have TIM3 counter clock equal to 500 KHz */
  Tim4_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 500000) - 1;
  
  /* Set TIM3 instance */
  Tim4_Handle.Instance = TIM4; //TIM4 is defined in stm32f429xx.h
   
  /* Initialize TIM4 peripheral as follows:
       + Period = 65535
       + Prescaler = ((SystemCoreClock/2)/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
	
	Tim4_Handle.Init.Period = 65535;  //does no matter, so set it to max.
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  //if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK)
  //{
    /* Initialization Error */
  //  Error_Handler();
  //} 
}



void  TIM4_OC_Config(void)
{
		Tim4_OCInitStructure.OCMode=  TIM_OCMODE_TIMING;
		Tim4_OCInitStructure.Pulse=Tim4_CCR;     // 500, this means every 1/1000 second generate an inerrupt
		Tim4_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
		
		HAL_TIM_OC_Init(&Tim4_Handle); // if the TIM4 has not been set, then this line will call the callback function _MspInit() 
													//in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC.
	
		HAL_TIM_OC_ConfigChannel(&Tim4_Handle, &Tim4_OCInitStructure, TIM_CHANNEL_1); //must add this line to make OC work.!!!
	
	   /* **********see the top part of the hal_tim.c**********
		++ HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal. 
			similar to PWD mode and Onepulse mode!!!
	
	*******************/
	
	 	HAL_TIM_OC_Start_IT(&Tim4_Handle, TIM_CHANNEL_1); //this function enable IT and enable the timer. so do not need
				//HAL_TIM_OC_Start() any more
				
		
}
	

static void EXTILine1_Config(void)  //for STM32f429_DISCO board, can not use PA1, PB1 and PD1,---PC1 is OK!!!!
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PC1 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //see  stm32fxx_hal_tim.c for different callback function names. 
																															//for timer 3 , Timer 3 use update event initerrupt
{
	if ((*htim).Instance==TIM3) 		//We use TIM3 for state 0 because the LEDs can toggle at alower frequency mre visible to the human eye
	//If the user buttn has not been pushed we are in state 0 and so we toggle oth LED's	
	if(user == 0){
			
	BSP_LED_Toggle(LED3);
	BSP_LED_Toggle(LED4);
	
			
		}
		
	
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
		if ((*htim).Instance==TIM4) {   
		//If the user button is pushed move to state 1 from state 0
				if(user == 1 && OC_Count < random){
					OC_Count=OC_Count+1;     //Increment wait time OC_Count 
					BSP_LED_Off(LED4);			//Turn LED's off
					BSP_LED_Off(LED3);
				}
		//If generated number of seconds has passed, switch from state 1 to 2	
				else if (user == 1 && OC_Count==random)  {   //random = generated wait time from RNG
					BSP_LED_On(LED4);	  //Turn on both LED's
					BSP_LED_On(LED3);
					factor++;						//Increment factor to find reaction time
					//OC_Count=0;		
				}
				
				//If the user button is pushed again but factor has not started incrementing yet-cheating detected. Switch from State  1 to State 0
				else if(user == 2 && factor == 0){
					user =0; //reset back to state 0
					factor =0; //reset back to state 0
					reset =0; //reset back to state 0
					OC_Count = 0;}
				
					//If no cheating and user button is pushed again: Switch from state 2 to state 3
				else if (user >= 2 && reset ==0){
					LCD_DisplayInt(9,2,factor); //Display the reaction time
					EE_ReadVariable(VirtAddVarTab[0], &EEREAD); //read the current best time from the EEPROM
					if(factor < EEREAD){  //If the reaction time is less than the current best time
						best = factor;
						EE_Init();
							EE_WriteVariable(VirtAddVarTab[0], best); //Put the reaction time into the EEPROM
							EE_ReadVariable(VirtAddVarTab[0], &EEREAD);	
							LCD_DisplayString(11,2,(uint8_t *)"     "); //Clear the LCD best time section
							LCD_DisplayInt(11, 2, EEREAD);		//Display the new best time on the LCD
					}
			
    }
//If the reset button is pressed we switch from state 3 to state 0.				
				else if(user >= 2	&& reset == 1){
					user = 0;
					factor = 0;
					OC_Count = 0;
					reset = 0;
					
					
}}
		
		//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h

}


/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
  {
    user++; //increment user to signify the user button has been pushed
  }
	
	
	if(GPIO_Pin == GPIO_PIN_1)
  {
		reset++; //increment reset to signify the reset button has been pushed
	}
 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
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
