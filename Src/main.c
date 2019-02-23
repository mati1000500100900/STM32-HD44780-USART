
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "LCD_HD44780.h"
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern uint32_t SystemCoreClock;

void DWT_Init(void) {
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

uint32_t DWT_Get(void) {
	return DWT->CYCCNT;
}

__inline uint8_t DWT_Compare(int32_t tp) {
	return (((int32_t) DWT_Get() - tp) < 0);
}

void DWT_Delay(uint32_t us) // microseconds
{
	int32_t tp = DWT_Get() + us * (SystemCoreClock / 1000000);
	while (DWT_Compare(tp))
		;
}

unsigned char crc8(unsigned char poly, unsigned char* data, int size) {
	unsigned char crc = 0x00;
	int bit;

	while (size--) {
		crc ^= *data++;
		for (bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ poly;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define BUF_SIZE 256
volatile uint8_t Buf_Rx[BUF_SIZE];
volatile uint8_t Buf_Tx[BUF_SIZE];
volatile uint8_t Busy_Rx = 0, Empty_Rx = 0, Busy_Tx = 0, Empty_Tx = 0;
volatile uint8_t temp = 0;
volatile uint8_t endFrameDetected = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		Empty_Rx++;
		if (Buf_Rx[Empty_Rx - 1] == '_')
			endFrameDetected = 1;
		if (Empty_Rx >= BUF_SIZE) {
			Empty_Rx = 0;
		}
	}
	HAL_UART_Receive_IT(&huart2, &Buf_Rx[Empty_Rx], 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (Busy_Tx != Empty_Tx) {
			temp = Buf_Tx[Busy_Tx];
			Busy_Tx++;
			if (Busy_Tx >= BUF_SIZE)
				Busy_Tx = 0;
			HAL_UART_Transmit_IT(&huart2, &temp, 1);
		}
	}
}

void frameDecoder() {
	char frameBuffer[255];
	char frame[127];
	char escapedFrame[127];
	uint8_t i = 0;
	while (Busy_Rx != Empty_Rx) {
		frameBuffer[i++] = Buf_Rx[Busy_Rx++];
		if (Busy_Rx >= BUF_SIZE) {
			if (Busy_Rx == Empty_Rx) {
				Busy_Rx = 0;
				Empty_Rx = 0;
			} else {
				Busy_Rx = 0;
			}
		}
	}
	//obcinanie pocz¹tku
	uint8_t k = 0;
	uint8_t startDetected = 0;
	for (uint8_t j = 0; j < i; j++) {
		if (frameBuffer[j] == '^') {
			startDetected = 1;
			k = 0;
		} else if (startDetected == 1) {
			frame[k++] = frameBuffer[j];
		}
	}
	if (startDetected != 0) {
		//deescapowanie ramki
		uint8_t m = 0;
		uint8_t escapeFlag = 0;
		for (uint8_t l = 0; l < k; l++) {
			if (frame[l] == 0x5C) {
				escapeFlag = 1;
			} else if (escapeFlag == 0) {
				escapedFrame[m++] = frame[l];
			} else {
				escapeFlag = 0;
				escapedFrame[m++] = frame[l] ^ 0x20;
			}
		}
		escapedFrame[m - 1] = '\0';

		if (m > 4 && m < 126) {
			uint8_t adresn = escapedFrame[0];
			uint8_t adreso = escapedFrame[1];
			uint8_t checksum = escapedFrame[m - 2];

			char dane[127];
			uint8_t n = 0;
			for (uint8_t o = 2; o < m - 2; o++) {
				dane[n++] = escapedFrame[o];
			}
			dane[n] = '\0';

			//sprawdzamy adres
			if (adreso == MY_ADDRESS) {
				//if(checksum==crc8(0x83,escapedFrame,m-2)){
				execute(dane, n, adresn);
				//}
				//else sendFrame(adresn,"wrong checksum",14);
			}
		}
	}
}

void execute(char *indata, uint8_t len, uint8_t adres) {
	unsigned int arga, argb;
	char str[110];
	uint8_t chardata[9];
	if (strncmp("cursoron", indata, 8) == 0) {
		LCD_Cursor(1);
		sendFrame(adres, "OK", 2);
	} else if (strncmp("cursoroff", indata, 9) == 0) {
		LCD_Cursor(0);
		sendFrame(adres, "OK", 2);
	} else if (strncmp("blinkon", indata, 7) == 0) {
		LCD_Blink(1);
		sendFrame(adres, "OK", 2);
	} else if (strncmp("blinkoff", indata, 8) == 0) {
		LCD_Blink(0);
		sendFrame(adres, "OK", 2);
	} else if (strncmp("cls", indata, 3) == 0) {
		LCD_Cls();
		sendFrame(adres, "OK", 2);
	} else if (strncmp("home", indata, 4) == 0) {
		LCD_Home();
		sendFrame(adres, "OK", 2);
	} else if (sscanf(indata, "setcursor:%u:%u", &arga, &argb)) {
		if (arga < LCD_X && argb < LCD_Y) {
			LCD_Locate((uint8_t) arga, (uint8_t) argb);
			sendFrame(adres, "OK", 2);
		} else {
			sendFrame(adres, "wrong argument", 14);
		}
	} else if (sscanf(indata, "printxy:%u:%u:%s", &arga, &argb, str)) {
		if (arga < LCD_X && argb < LCD_Y) {
			LCD_Locate((uint8_t) arga, (uint8_t) argb);
			LCD_String(str);
			sendFrame(adres, "OK", 2);
		} else {
			sendFrame(adres, "wrong argument", 14);
		}
	} else if (sscanf(indata, "define:%u:", &arga)) {
		if (arga >= 0 && arga < 8 && len == 17) {
			LCD_DefChar(arga, indata + 9);
			LCD_Locate(0, 0);
			sendFrame(adres, "OK", 2);
		} else
			sendFrame(adres, "wrong argument", 14);
	} else if (sscanf(indata, "write:%u", &arga)) {
		if (arga >= 0 && arga < 8) {
			LCD_Char(arga);
			sendFrame(adres, "OK", 2);
		} else
			sendFrame(adres, "wrong argument", 14);
	} else if (sscanf(indata, "read:%u", &arga)) {
		if (arga >= 0 && arga < 8) {
			LCD_ReadChar(arga, chardata);
			sendFrame(adres, chardata, 8);
			LCD_Locate(0,0);
		} else
			sendFrame(adres, "wrong argument", 14);
	} else if (strncmp("print:", indata, 6) == 0) {
		if (len > 6) {
			LCD_String(indata + 6);
			sendFrame(adres, "OK", 2);
		} else
			sendFrame(adres, "wrong argument", 14);
	} else {
		sendFrame(adres, "unknown command", 15);
	}
}

void sendFrame(uint8_t adres, uint8_t *senddata, uint8_t length) {
	uint8_t frame[128];
	frame[0] = MY_ADDRESS;
	frame[1] = adres;
	uint8_t i;
	for (i = 0; i < length; i++) {
		frame[i + 2] = senddata[i];
	}
	frame[length + 2] = crc8(0x83, frame, length + 2); //crc

	uint8_t escapedFrame[128]; //escapujemy
	uint8_t m = 1;
	escapedFrame[0] = '^';
	for (uint8_t l = 0; l < length + 3; l++) {
		if (frame[l] == 0x5C || frame[l] == 0x5E || frame[l] == 0x5F) {
			escapedFrame[m++] = 0x5C;
			escapedFrame[m++] = frame[l] ^ 0x20;
		} else {
			escapedFrame[m++] = frame[l];
		}
	}
	escapedFrame[m] = '_';

	uint8_t idx = Empty_Tx; //wrzucamy ramkê do bufora wyjœciowego
	for (int i = 0; i < m + 1; i++) {
		Buf_Tx[idx] = escapedFrame[i];
		idx++;
		if (idx > 255) {
			idx = 0;
		}
	}
	__disable_irq();

	if (Busy_Tx == Empty_Tx
			&& __HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) == SET) {
		Empty_Tx = idx;
		temp = Buf_Tx[Busy_Tx];
		Busy_Tx++;
		if (Busy_Tx > 255) {
			Busy_Tx = 0;
		}
		HAL_UART_Transmit_IT(&huart2, &temp, 1); //transmitujemy pierwszy bajt
	} else {
		Empty_Tx = idx;
	}
	__enable_irq();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	//sendFrame(0x62, "RESET", 5);
	HAL_UART_Receive_IT(&huart2, &Buf_Rx[Empty_Rx], 1);

	LCD_Init();
	LCD_Locate(0, 0);
	LCD_String("Mateusz Tokarczyk");
	LCD_Locate(9, 3);
	LCD_String("Grupa 7 IST");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (endFrameDetected) {
			endFrameDetected = 0;
			frameDecoder();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_Pin|RS_Pin|RW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D0_Pin|D1_Pin|D2_Pin|D3_Pin 
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin RS_Pin RW_Pin */
  GPIO_InitStruct.Pin = EN_Pin|RS_Pin|RW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D7_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D3_Pin D4_Pin D5_Pin D6_Pin */
  GPIO_InitStruct.Pin = D3_Pin|D4_Pin|D5_Pin|D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
