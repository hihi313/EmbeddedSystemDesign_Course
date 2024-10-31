  /**
  ******************************************************************************
  * @file    GUI_App.c
  * @author  MCD Application Team
  * @brief   Simple demo drawing "Hello world"  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "GUI_App.h"
#include "GUI.h"

#include "DIALOG.h"
extern  WM_HWIN CreateFramewin(void); 
extern void BSP_Touch_Update(void);
extern int x, y;
extern GUI_MEMDEV_Handle hMem;
extern int printf(const char * __restrict /*format*/, ...);
int leftBound = 92;

void GRAPHICS_MainTask(void) {

  /* 1- Create a FrameWin using GUIBuilder */
  WM_HWIN hWin = CreateFramewin();
 
/* USER CODE BEGIN GRAPHICS_MainTask */
	WM_HWIN hWinOld;
	GUI_PID_STATE State;
 /* User can implement his graphic application here */
  /* Hello Word example */
//    GUI_Clear();
//    GUI_SetColor(GUI_WHITE);
//    GUI_SetFont(&GUI_Font32_1);
//    GUI_DispStringAt("Hello world!", (LCD_GetXSize()-150)/2, (LCD_GetYSize()-20)/2);
	GUI_SetBkColor(GUI_WHITE);
	GUI_SetPenSize(5);
	GUI_SetColor(GUI_BLACK);
	hMem = GUI_MEMDEV_Create(0, 0, x, y);
	int pX = -1, pY = -1;
	int cnt = 0;
	do {    
		if(cnt >= 100){
			pX = -1;
			pY = -1;
		}
		BSP_Touch_Update();
		GUI_PID_GetCurrentState(&State);
		if(State.x < WM_GetWindowSizeX(hWin)-leftBound){ //int WM_GetWindowSizeX(WM_HWIN hWin);
			
			hWinOld = WM_SelectWindow(WM_GetClientWindow(hWin));
			if(pX < 0){
				GUI_DrawPoint(State.x, State.y);
			}else{
				GUI_DrawLine(pX, pY, State.x, State.y);
			}
			WM_SelectWindow(hWinOld);
			pX = State.x;
			pY = State.y;
			cnt = 0;
		}
		cnt++;
		GUI_Delay(1); 
	} while (1);
/* USER CODE END GRAPHICS_MainTask */
  while(1)
{
      GUI_Delay(100);
}
}

/*************************** End of file ****************************/
