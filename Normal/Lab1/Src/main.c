/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "dma.h"
#include "dma2d.h"
#include "ltdc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.c"
#include "stm32746g_discovery_sdram.c"
#include "stdio.h"
#include "string.h"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// UART buffer
char inputStr = 0;
// pins
GPIO_PinState btnPin;
// buttom
int btn = 0, dbnc_cnt = 0, dbnc_en = 0;
// led
int led_cnt = 0;
// control flags
int fmode = 0, inputNum_ea = 0, LEDBlink_ea = 1, debug = 0;
// timing
int LEDBlink_T = 100;
// counter
char LEDBlink_cnt = 0;
// LCD display string
char LCDChar = 0;
//LCD display line #
int displayLine = 0;
// custom functions
void display(char* str);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	BSP_SDRAM_Init();  
	BSP_LCD_Init();
	BSP_LCD_LayerRgb565Init(LTDC_ACTIVE_LAYER, SDRAM_DEVICE_ADDR);
	//BSP_LCD_SelectLayer(1);
	BSP_LCD_SetLayerVisible(LTDC_ACTIVE_LAYER, ENABLE);
	/* Set Foreground Layer */
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
	BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_Clear(LCD_COLOR_BLACK);	
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_DisplayOn();
	//BSP_LCD_DisplayStringAtLine(0, (uint8_t*)"Hollow World!");
	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
					
		//HAL_UART_Receive(&huart1, (uint8_t *)&rstr, 10, 0xFFFF);
		//HAL_UART_Transmit(&huart1, (uint8_t *)&rstr, 10, 0xFFFF);
				
		// button state
		btnPin = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_11);		
		// button press
		if(btnPin && btn == 0 && dbnc_cnt == 0){
			dbnc_en = 1;
		}
		if(btnPin && btn == 0 && dbnc_cnt >= 20){
			dbnc_en = 0;
			dbnc_cnt = 0;
			btn = 1;
		}
		// button release
		if(btnPin == 0 && btn && dbnc_cnt == 0){
			dbnc_en = 1;
		}
		if(btnPin == 0 && btn && dbnc_cnt >= 20){
			dbnc_en = 0;
			dbnc_cnt = 0;
			btn = 0;
			// do things after/when button release
			if(!LEDBlink_cnt){ // change mode only if LED not blinking in mode 2
				fmode = !fmode;
				switch(fmode){
					case 0:
						//mode 1
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);// initial LED off
						inputNum_ea = 0; // cannot input
						LEDBlink_ea = 1; // always blinking
						LEDBlink_T = 100; // set blinking period
						if(debug)
							printf("mode 1\n\r");
						break;
					case 1:
						// mode 2
						HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);// initial LED off
						inputNum_ea = 1; // input first
						LEDBlink_ea = 0; // wait for input
						LEDBlink_T = 500; // set binking period
						if(debug)
							printf("mode 2\n\r");
						HAL_UART_Receive_IT(&huart1, (uint8_t *)&inputStr, 1);
						break;
					default: // default mode 1
						inputNum_ea = 0;
						LEDBlink_ea = 1;
						LEDBlink_T = 100;
						break;
				}
			}
		}
		// led blink
		if(LEDBlink_ea && led_cnt >= LEDBlink_T){
			switch(fmode){
				case 0:
					// mode 1
					HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_1);
					break;
				case 1:
					// mode 2
					if(LEDBlink_cnt > 0){
						HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_1);
						if(!HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_1)){// display when LED is OFF
							LEDBlink_cnt--;
							printf("%d\n\r", LEDBlink_cnt);
							sprintf(&LCDChar, "%d", LEDBlink_cnt);
							display(&LCDChar);
						}
					}	else {// blinking finished
						inputNum_ea = 1;
						LEDBlink_ea = 0;
						LEDBlink_cnt = 0;
						printf("end\n\r");
						display("end");
						HAL_UART_Receive_IT(&huart1, (uint8_t *)&inputStr, 1);
					}
					break;
				default:
					HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);
					break;
			}
		}
		
		HAL_Delay(1);
		if(dbnc_en)
			dbnc_cnt++;
		if(led_cnt < LEDBlink_T)
			led_cnt++;
		else
			led_cnt = 0;		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF); 
	return ch;
}
// UART Receive callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	  UNUSED(huart);
		// mode 2, wait for input
		if(inputNum_ea){
			if(debug)
				printf("input str:%c\n\r", inputStr);			
			LEDBlink_cnt = inputStr - '0';
			// input data validation
			if(LEDBlink_cnt < 1 || LEDBlink_cnt > 9){ // invalid input
				if(debug)
					printf("input invalid\n\r");
				LEDBlink_cnt = 0;
				HAL_UART_Receive_IT(&huart1, (uint8_t *)&inputStr, 1);
			}else{// valid input
				printf("start\n\r%d\n\r", LEDBlink_cnt);
				display("start");
				sprintf(&LCDChar, "%d", LEDBlink_cnt);
				display(&LCDChar);
				inputNum_ea = 0;
				LEDBlink_ea = 1;
			}			
		}
}
// LCD display
void display(char* str){
	BSP_LCD_ClearStringLine(displayLine);
	BSP_LCD_DisplayStringAtLine(displayLine, (uint8_t*)str);
	displayLine++;
	if(displayLine > 10)
		displayLine = 0;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
