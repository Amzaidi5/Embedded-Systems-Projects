/****** 


1.The Fist Extern button (named extBtn1)  connected to PC1, ExtBtn1_PC1_Config  //
		2014: canot use pin PB1, for 429i-DISCO ,pb1 is used by LCD. if use this pin, always interrupt by itself
					can not use pin PA1, since it is used by gyro. if use this pin, never interrupt.
					pd1----WILL ACT AS PC13, To trigger the RTC timestamp event
					....ONLY PC1 CAN BE USED TO FIRE EXTI1 !!!!
2. the Second external button (extBtn2)
		2014: 
		PA2: NOT OK. (USED BY LCD??)
		PB2: ????.
		PC2: ok, BUT sometimes (every 5 times around), press pc2 will trigger exti1, which is configured to use PC1. (is it because of using internal pull up pin config?)
		      however, press PC1 does not affect exti 2. sometimes press PC2 will also affect time stamp (PC13)
		PD2: OK,     
		PE2:  OK  (PE3, PE4 PE5 , seems has no other AF function, according to the table in manual for discovery board)
		PF2: NOT OK. (although PF2 is used by SDRAM, it affects LCD. press it, LCD will flick and displayed chars change to garbage)
		PG2: OK
		

*/

#include "main.h"

// Define handles for ADC and timers
ADC_HandleTypeDef AdcHandle;
TIM_HandleTypeDef Tim3_Handle, Tim4_Handle;

#define COLUMN(x) ((x) * (((sFONT *)BSP_LCD_GetFont())->Width))    //see font.h, for defining LINE(X)

#define FLAG_ACTIVE                        1
#define FLAG_INACTIVE                      0



int test=0;

TIM_OC_InitTypeDef Tim3_OCInitStructure;
uint16_t Tim3_PrescalerValue;
uint16_t TIM3Prescaler=1799;   //with the prescaler as 180, every 500 ticks of TIM3 is 1ms. if 1800, then every 50 ticks is 1ms
uint16_t TIM3Period=1000;    //1,000 ticks of TIM3, with 1800 prescaller, is 20ms
TIM_OC_InitTypeDef Tim4_OCInitStructure;
uint16_t TIM4Prescaler=1799;   //with the prescaler as 180, every 500 ticks of TIM3 is 1ms. if 1800, then every 50 ticks is 1ms
uint16_t TIM4Period=1000;    //1,000 ticks of TIM3, with 1800 prescaller, is 20ms
__IO uint16_t TIM4_CCR1_Val=200, TIM4_CCR2_Val=400,TIM4_CCR3_Val=600,TIM4_CCR4_Val=800;  // for dynamically set the pulse width  for the four channels. 																																// can vary it from 1 to 1,000 to vary the pulse width
__IO uint16_t ADC3ConvertedValue=0;


 volatile double  setPoint=24.5;  //NOTE: if declare as float, when +0.5, the compile will give warning:
															//"single_precision perand implicitly converted to double-precision"
															//ALTHOUGH IT IS A WARNING, THIS WILL MAKE THE PROGRAM not WORK!!!!!!
															//if declare as float, when setPoint+=0.5, need to cast : as setPioint+=(float)0.5, otherwise,
															//the whole program will not work! even this line has  not been used/excuted yet
															//BUT, if declare as double, there is no such problem.
															
	    														
	//			Project Options -> Target -> Floating Point Hardware -> Use FPU

	//You must then have code to enable the FPU hardware prior to using any FPU instructions. This is typically in the ResetHandler or SystemInit()

	//            system_stm32f4xx.c
  ////            void SystemInit(void)
	//							{
	//								/* FPU settings ------------------------------------------------------------*/
	//								#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	//									SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	//								#endif
	//							...											
	//		-----MODIFY the system_stm32f4xx.c in the above way, will also fix the "float" type problem mentioned above. 												
	//   the is noted in 2016. by 2021, this problem may be fixed by STM.											

double measuredTemp;
__IO uint16_t uhADCxConvertedValue = 0;


void  LEDs_Config(void);
void  LEDs_On(void);
void  LEDs_Off(void);
void  LEDs_Toggle(void);

// Functions

static double getTemp(void);
static uint16_t readADCValMV(void);

void LCD_DisplayString(uint16_t LineNumber, uint16_t ColumnNumber, uint8_t *ptr);
void LCD_DisplayInt(uint16_t LineNumber, uint16_t ColumnNumber, int Number);
void LCD_DisplayFloat(uint16_t LineNumber, uint16_t ColumnNumber, float Number, int DigitAfterDecimalPoint);

static void   displayTemp                (void);
static void   displayTempSetpoint        (void);
static double getTemp                    (void);





static void   ADC_Config                 (void);
void ExtBtn1_Config(void);  //for the first External button
void ExtBtn2_Config(void);

void ADC_Config(void);
void TIM4_Config(void);
void TIM3_PWM_Config(void);

static void SystemClock_Config(void);
static void Error_Handler(void);

//-----------Private Variabes
volatile uint8_t               TEMP_POLL_TIMER_FLAG      = FLAG_INACTIVE;         // Temperature poll flag
double               currentTemperature=1;
int upcount=0, downcount=0;
int hold = 500;

int main(void){
	//Configure all external functions
	ADC_Config();
	ADC_ChannelConfTypeDef sConfig;
	currentTemperature  = getTemp();
	TIM4_Config();
	TIM3_PWM_Config();
	HAL_Init();
	SystemClock_Config();
  HAL_InitTick(0x0000); 
	LEDs_Config();
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);   // BSP_functions in stm32f429i_discovery.c
	ExtBtn1_Config();
	ExtBtn2_Config();																	
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);   //LCD_FRAME_BUFFER, defined as 0xD0000000 in _discovery_lcd.h
	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear(LCD_COLOR_WHITE);  //need this line, otherwise, the screen is dark	
	BSP_LCD_DisplayOn();
	BSP_LCD_SetFont(&Font20);  //the default font,  LCD_DEFAULT_FONT, which is defined in _lcd.h, is Font24
	LCD_DisplayString(3, 2, (uint8_t *) "Lab4 Starter ");
	LCD_DisplayString(9, 0, (uint8_t *) "Current ");
  LCD_DisplayString(10, 0, (uint8_t *)"setPoint");
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);
		
		
	while(1) {
	
	if (TEMP_POLL_TIMER_FLAG == FLAG_ACTIVE)
        {
          TEMP_POLL_TIMER_FLAG = FLAG_INACTIVE;
          
          currentTemperature = getTemp();//gets current temperature
          /* Display current temperature */
          displayTemp();
					//Display setPoint
					LCD_DisplayFloat(10, 10, setPoint, 2);
					//Checks temperature and compares to set point and  will turn on fan accordingly
					if(currentTemperature - setPoint >= 6){
						BSP_LED_Toggle(LED3);
						Tim3_OCInitStructure.Pulse=800;  //400
						HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2); 
						HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2);
					}
					else if(currentTemperature - setPoint >= 4){
						Tim3_OCInitStructure.Pulse=600;  
						HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2); 
						HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2);
						
					}
					else if(currentTemperature - setPoint >= 2){
						Tim3_OCInitStructure.Pulse=400;  
						HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2); 
						HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2);
						
					}
					else if(currentTemperature - setPoint > 0){
						Tim3_OCInitStructure.Pulse=200;  
						HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2); 
						HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2);
						
					}
					else if(currentTemperature-setPoint <=0){
						Tim3_OCInitStructure.Pulse=0;  
						HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2); 
						HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2);
						
					}
					
        }
				//Will check for button press and move stepoint accordingly
		if(!HAL_GPIO_ReadPin (GPIOC, GPIO_PIN_1) && (HAL_GetTick() - upcount) >= hold ){
			setPoint = setPoint + 0.5;
			upcount=HAL_GetTick();
		}
		
		if(!HAL_GPIO_ReadPin (GPIOD, GPIO_PIN_2) && (HAL_GetTick() - downcount) >= hold ){
			setPoint = setPoint - 0.5;
			downcount = HAL_GetTick();
		}
		
		
		
			
	} // end of while loop
	
}  //end of main


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
	
//Function to read from temperature sensor	
static uint16_t readADCValMV(void)
{HAL_ADC_Start(&AdcHandle);
 HAL_ADC_PollForConversion(&AdcHandle, 500);
    uint32_t output = HAL_ADC_GetValue(&AdcHandle);
    
    /* Read the converted value */
    return (uint16_t) output;
}
//Converts ADC value to temperature in celcius
static double getTemp(void)
{
  double ADCVal = (double) readADCValMV();
  /* Divide ADC value in mv by 3 to remove gain, and divide by 10 to convert to celcius */
  return (ADCVal*0.024);
}
//Displays temperature to LCD	
static void displayTemp(void)
{
 LCD_DisplayFloat(9,10,currentTemperature, 2);
	
}
	
	
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




//Configure timer 4 to run temperature sensor
void  TIM4_Config(void)
{


 uint16_t  Tim4_PrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
  
  
  Tim4_Handle.Instance = TIM4; //TIM3 is defined in stm32f429xx.h
   
  Tim4_Handle.Init.Period = 50000 - 1;
  Tim4_Handle.Init.Prescaler = Tim4_PrescalerValue;
  Tim4_Handle.Init.ClockDivision = 0;
  Tim4_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  if(HAL_TIM_Base_Init(&Tim4_Handle) != HAL_OK) // this line need to call the callback function _MspInit() in stm32f4xx_hal_msp.c to set up peripheral clock and NVIC..
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  
  if(HAL_TIM_Base_Start_IT(&Tim4_Handle) != HAL_OK)   //the TIM_XXX_Start_IT function enable IT, and also enable Timer
																											//so do not need HAL_TIM_BASE_Start() any more.
  {
    /* Starting Error */
    Error_Handler();
  }
	
	
	
}

//Configure timer 3 for PWM for fan
void  TIM3_PWM_Config(void)
{  
  /* Set TIM3 instance */
  Tim3_Handle.Instance = TIM3; //TIM3 is defined in stm32f429xx.h
   
  Tim3_Handle.Init.Period = TIM3Period;; //pwm frequency? 
  Tim3_Handle.Init.Prescaler = TIM3Prescaler;
  Tim3_Handle.Init.ClockDivision = 0;
  Tim3_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Tim3_Handle.Init.RepetitionCounter = 0;  //default is 0
	

	Tim3_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
 
 
	if(HAL_TIM_PWM_Init(&Tim3_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
	//configure the PWM channel
	Tim3_OCInitStructure.OCMode=  TIM_OCMODE_PWM1; //TIM_OCMODE_TIMING;
	Tim3_OCInitStructure.OCFastMode=TIM_OCFAST_DISABLE;
	Tim3_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;
	//Tim3_OCInitStructure.OCNPolarity = TIM_OCNPOLARITY_HIGH //complementary polarity. 
																	//This parameter is valid only for TIM1 and TIM8.
	//Tim3_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET; //This parameter is valid only for TIM1 and TIM8.
  //Tim3_OCInitStructure.OCNIdleState= TIM_OCNIDLESTATE_RESET; //This parameter is valid only for TIM1 and TIM8.
	
	Tim3_OCInitStructure.Pulse=TIM4_CCR1_Val;   //200
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
	
	
	Tim3_OCInitStructure.Pulse=TIM4_CCR2_Val;  //400
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
	}
	
	
	Tim3_OCInitStructure.Pulse=TIM4_CCR3_Val;   //600;
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
	}
	
	
	Tim3_OCInitStructure.Pulse=TIM4_CCR4_Val;   //800;
	
	if(HAL_TIM_PWM_ConfigChannel(&Tim3_Handle, &Tim3_OCInitStructure, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
	}
	
	
	//***************************
	
if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	


if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	


if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	


if(HAL_TIM_PWM_Start(&Tim3_Handle, TIM_CHANNEL_4) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }  
	
}

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef * htim )
{
  /* If general timer set corresponding flag */
  if (htim->Instance == TIM4)
  {	BSP_LED_Toggle(LED4);
    TEMP_POLL_TIMER_FLAG = FLAG_ACTIVE;
  }
}
//Configure ADC for temperature sensor
void ADC_Config(void){
		ADC_ChannelConfTypeDef sConfig;
			  /*##-1- Configure the ADC peripheral #######################################*/
		AdcHandle.Instance          = ADCx;
		
		AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
		AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
		AdcHandle.Init.ScanConvMode = DISABLE;
		AdcHandle.Init.ContinuousConvMode = ENABLE;
		AdcHandle.Init.DiscontinuousConvMode = DISABLE;
		AdcHandle.Init.NbrOfDiscConversion = 0;
		AdcHandle.Init.ExternalTrigConvEdge = DISABLE;
		AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
		AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		AdcHandle.Init.NbrOfConversion = 1;
		AdcHandle.Init.DMAContinuousRequests = DISABLE;
		AdcHandle.Init.EOCSelection = DISABLE;
				
		if(HAL_ADC_Init(&AdcHandle) != HAL_OK)
		{
			/* Initialization Error */
			Error_Handler(); 
		}

		/*##-2- Configure ADC regular channel ######################################*/  
		sConfig.Channel = ADCx_CHANNEL;
		sConfig.Rank = 1;
		sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
		sConfig.Offset = 0;
		
		if(HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
		{
			/* Channel Configuration Error */
			Error_Handler(); 
		}

		/*##-3- Start the conversion process and enable interrupt ##################*/
		/* Note: Considering IT occurring after each number of ADC conversions      */
		/*       (IT by DMA end of transfer), select sampling time and ADC clock    */
		/*       with sufficient duration to not create an overhead situation in    */
		/*        IRQHandler. */ 
		if(HAL_ADC_Start(&AdcHandle)!= HAL_OK)
		{
			/* Start Conversation Error */
			Error_Handler(); 
		}
	
	
}
void LEDs_Config(void)
{
 /* Initialize Leds mounted on STM32F429-Discovery board */
	BSP_LED_Init(LED3);   //BSP_LED_....() are in stm32f4291_discovery.c
  BSP_LED_Init(LED4);
}

void LEDs_On(void){
/* Turn on LED3, LED4 */
 BSP_LED_On(LED3);
  BSP_LED_On(LED4);
}

void LEDs_Off(void){
/* Turn on LED3, LED4 */
  BSP_LED_Off(LED3);
  BSP_LED_Off(LED4);
}
void LEDs_Toggle(void){
/* Turn on LED3, LED4 */
  BSP_LED_Toggle(LED3);
  BSP_LED_Toggle(LED4);
}




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








void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
  if(GPIO_Pin == KEY_BUTTON_PIN)  //GPIO_PIN_0
  {
			
  }
	
	
	if(GPIO_Pin == GPIO_PIN_1)
  {
		 upcount = HAL_GetTick();//Gets tick time of button press

	}  //end of PIN_1

	if(GPIO_Pin == GPIO_PIN_2)
  {
	 downcount = HAL_GetTick();
			
	} //end of if PIN_2	
	
	
}



void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef * htim) //see  stm32fxx_hal_tim.c for different callback function names. 
{																																//for timer4 
	//	if ((*htim).Instance==TIM4)
			 //BSP_LED_Toggle(LED4);
		//clear the timer counter!  in stm32f4xx_hal_tim.c, the counter is not cleared after  OC interrupt
		//__HAL_TIM_SET_COUNTER(htim, 0x0000);   //this maro is defined in stm32f4xx_hal_tim.h
	
}
 
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef * htim){  //this is for TIM3_pwm
	
	//__HAL_TIM_SET_COUNTER(htim, 0x0000);   //not necessary  
}



static void Error_Handler(void)
{
  /* Turn LED4 on */
  BSP_LED_On(LED4);
  while(1)
  {LCD_DisplayInt(13,2,13);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
{
  /* Turn LED3 on: Transfer process is correct */
  BSP_LED_Toggle(LED3);
	
}

//-----------Button Configurations
void ExtBtn1_Config(void)     // for GPIO C pin 1
// can only use PA0, PB0... to PA4, PB4 .... because only  only  EXTI0, ...EXTI4,on which the 
	//mentioned pins are mapped to, are connected INDIVIDUALLY to NVIC. the others are grouped! 
		//see stm32f4xx.h, there is EXTI0_IRQn...EXTI4_IRQn, EXTI15_10_IRQn defined
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_1;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void ExtBtn2_Config(void){  //**********PD2.*********** // Set up interupt for external bullton 2 on pin PD2
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOB clock */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  
  /* Configure PA0 pin as input floating */
  GPIO_InitStructure.Mode =  GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull =GPIO_PULLUP;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
	//GPIO_InitStructure.Speed=GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);

	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);   //is defined the same as the __HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_1); ---check the hal_gpio.h
	__HAL_GPIO_EXTI_CLEAR_FLAG(GPIO_PIN_2);// after moving the chunk of code in the GPIO_EXTI callback from _it.c (before these chunks are in _it.c)
																					//the program "freezed" when start, suspect there is a interupt pending bit there. Clearing it solve the problem.
  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
	
	
	
	
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

