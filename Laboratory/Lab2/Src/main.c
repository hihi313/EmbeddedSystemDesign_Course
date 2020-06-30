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
#include "dma2d.h"
#include "ltdc.h"
#include "sai.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_sdram.h"
#include "stm32746g_discovery_ts.h"
#include "stdio.h"
#include <stdlib.h>
#include <math.h>
#define ARM_MATH_CM7 1
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int min(int, int);
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
#define AUDIO_BUFFER_SIZE   ((uint16_t)1024)
static short audioBuff[AUDIO_BUFFER_SIZE];
float32_t fftIn[AUDIO_BUFFER_SIZE];
float32_t fftOut[AUDIO_BUFFER_SIZE];
float32_t outBuff[AUDIO_BUFFER_SIZE];
int x, y, baseline = 0, fftBaseL = 0, halfHeight = 136, fftHalfH = 136, top = 0, fftTop = 0, bottom = 272, fftBottom = 0;
int signal[2], fftSig[2];
static TS_StateTypeDef  TS_State;
short tsX, tsY;
double idx, f, pwr, timeScale = 1;
int timeLiveDisp = 1;
int ts = 0, tsPrs = 0, dbnc_cnt = 0, dbnc_en = 0;
char str[100];
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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_SAI2_Init();
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
	BSP_AUDIO_IN_Init(AUDIO_FREQUENCY_8K, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
  BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int i;
	x = BSP_LCD_GetXSize();
	y = BSP_LCD_GetYSize();
	baseline = y/4;
	fftBaseL = y;
	halfHeight = min(y-baseline, baseline-0);
	fftHalfH = min(y-fftBaseL, fftBaseL-0);
	top = 0;
	bottom = y/2;
	fftTop = y/2;
	fftBottom = y-1;
	BSP_AUDIO_IN_Record(&audioBuff[0], AUDIO_BUFFER_SIZE);
	arm_rfft_fast_instance_f32 S; 
	timeScale = AUDIO_BUFFER_SIZE*1.0/x;
	
	for(i = 0; i < AUDIO_BUFFER_SIZE; i++){
		audioBuff[i] = 10.0f*sin(2*3.1415926f*200*i/8000);
	}
	while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		

		BSP_LCD_Clear(LCD_COLOR_BLACK);
		BSP_LCD_DrawLine(0, y/2, x, y/2);
		
		for(i = 0; i < AUDIO_BUFFER_SIZE; i++){
			fftIn[i] = audioBuff[i]/10.0f;
		}
		arm_rfft_fast_init_f32(&S, AUDIO_BUFFER_SIZE); 
		arm_rfft_fast_f32(&S, fftIn, fftOut, 0); 
		arm_cmplx_mag_f32(fftOut, outBuff, AUDIO_BUFFER_SIZE);
		
		for(i = 0 ; i < x; i++){ 
			signal[0] = baseline - (short)(audioBuff[(int)(i*timeScale)]);
			signal[1] = baseline - (short)(audioBuff[(int)((i+1)*timeScale)]);
			if(signal[0] < top)
				signal[0] = top;
			else if (signal[0] > bottom)
				signal[0] = bottom;
			if(signal[1] < top)
				signal[1] = top;
			else if (signal[1] > bottom)
				signal[1] = bottom;
			BSP_LCD_DrawLine(i, signal[0], i+1, signal[1]);
			
			fftSig[0] = fftBaseL - outBuff[(int)(i*255/480.0)]/10;
			fftSig[1] = fftBaseL - outBuff[(int)((i+1)*255/480.0)]/10; 
			if(fftSig[0] < fftTop)
				fftSig[0] = fftTop;
			else if (fftSig[0] > fftBottom)
				fftSig[0] = fftBottom;
			if(fftSig[1] < fftTop)
				fftSig[1] = fftTop;
			else if (fftSig[1] > fftBottom)
				fftSig[1] = fftBottom;
			BSP_LCD_DrawLine(i, fftSig[0], i+1, fftSig[1]);
		}
		/*
		for(i = 0; i < AUDIO_BUFFER_SIZE; i++){
			printf("audioBuff[%d]:%d\t", i, audioBuff[i]);
			printf("fftIn[%d]:%f\t", i, fftIn[i]);
			printf("fftOut[%d]:%f\t", i, fftOut[i]);
			printf("outBuff[%d]:%f\n\r", i, outBuff[i]);
		}
		*/
		BSP_TS_GetState(&TS_State);
		ts = TS_State.touchDetected;
    if(ts){
			tsX = TS_State.touchX[0];
			tsY = TS_State.touchY[0];
		}
		
		if(ts && tsPrs == 0 && dbnc_cnt == 0){
			dbnc_en = 1;
		}
		if(ts && tsPrs == 0 && dbnc_cnt == 2){
			dbnc_en = 0;
			dbnc_cnt = 0;
			tsPrs = 1;
		}
		if(tsPrs && ts == 0 && dbnc_cnt == 0){
			dbnc_en = 1;
		}
		if(tsPrs && ts == 0 && dbnc_cnt == 2){
			dbnc_en = 0;
			dbnc_cnt = 0;
			tsPrs = 0;
			if(tsY < y/2){
				timeLiveDisp = !timeLiveDisp;
				if(!timeLiveDisp){
					BSP_AUDIO_IN_Stop(CODEC_PDWN_SW);
					sprintf(str, "");
				}else{
					BSP_AUDIO_IN_Record(&audioBuff[0], AUDIO_BUFFER_SIZE);
					sprintf(str, "");
				}				
			}
			if(!timeLiveDisp && tsY > y/2){
				idx = tsX * 255.0/479.0;
				pwr = outBuff[(int)idx];
				f = idx * 2 * 8000.0 /1024.0;
				sprintf(str, "Freq:%.2f,Power:%.2f", f, pwr);				
				//for(i = 0; i < AUDIO_BUFFER_SIZE; i++) printf("%d, %f, %f, %f\n\r", audioBuff[i], fftIn[i], fftOut[i], outBuff[i]);
			}	
		}
		BSP_LCD_DrawPixel(tsX, tsY, 0xffffff7f);
		BSP_LCD_DisplayStringAt(x, y-30, (uint8_t*)str, LEFT_MODE);
		
		
		HAL_Delay(10);
		if(dbnc_en)
			dbnc_cnt++;
		if(dbnc_cnt > 5){
			dbnc_en = 0;
			dbnc_cnt = 0;
		}
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_SAI2;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF); 
	return ch;
}
void BSP_AUDIO_IN_TransferComplete_CallBack(void){
	//BSP_AUDIO_IN_Pause();
  return;
}
void BSP_AUDIO_IN_HalfTransfer_CallBack(void){
  return;
}
int min(int i, int j){
	return (i < j)? i : j ;
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
