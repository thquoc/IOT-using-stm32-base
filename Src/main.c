/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CLCD.h"
#include <stdio.h>
#include <string.h>

CLCD_Name LCD1;
uint8_t count;
char LCD_send[30];
char UART_send[20];

uint32_t soilADCValue=0;
//uint32_t value=0;
float soilADCPercent = 0.0;
float soilValue=0.0;

uint8_t buttonState1=0;
uint8_t buttonState2=0;

uint32_t previousMillis = 0;  // Bi?n d? luu tr? th?i gian tru?c dó
const uint32_t interval = 60000;  

char mode_auto[20] ="System:AUTO";
char mode_manual[20] = "System status:MANUAL";
char LCD_state_light[20];
char LCD_state_pump[20];
char STATE[5];
char currentDate[11];
char currentTime[9];

//extern RTC_HandleTypeDef hrtc;







// Task handles


// ï¿½inh nghia hï¿½m map
float map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void readADCsoil();
void Toggle_LED(GPIO_TypeDef* buttonGPIOPort, uint16_t buttonGPIOPin, GPIO_TypeDef* ledGPIOPort, uint16_t ledGPIOPin);
void Read_Light_Sensor_And_Control_LED(GPIO_TypeDef* sensorPort, uint16_t sensorPin, GPIO_TypeDef* ledPort, uint16_t ledPin);
// Khai bÃ¡o biáº¿n tráº¡ng thÃ¡i cháº¿ Ä‘á»™
typedef enum {
    MODE_AUTO,
    MODE_MANUAL
} Mode;

Mode mode = MODE_AUTO;  // Khá»Ÿi táº¡o vá»›i cháº¿ Ä‘á»™ tá»± Ä‘á»™ng
void Toggle_Mode(GPIO_TypeDef* modeButtonPort, uint16_t modeButtonPin, volatile uint8_t* mode);
void sendUARTData(float soilADCPercent);
void controlPump(float soilADCPercent);

void displayLCD(char mode[20],char Lcd_send[30]);
//void displayLCD(char mode[20], char lcd_send[30]

//void RTC_to_DateString(char *buffer, RTC_DateTypeDef *date);
//void RTC_to_TimeString(char *buffer, RTC_TimeTypeDef *time);
//void getCurrentDate(char *buffer);
//void getCurrentTime(char *buffer);



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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
//osThreadId_t readSensorTaskHandle;
//osThreadId_t controlLightTaskHandle;
//osThreadId_t displayLCDTaskHandle;

//osMessageQueueId_t sensorValueQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
//void readSensorTask (void *argument);
//void controlLightTask(void *argument);
//void displayLCDTask (void *argument);

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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	   //HAL_ADC_Start_DMA(&hadc1, &var, 1);
//		LCD16X2_Init(MyLCD);
//    LCD16X2_Clear(MyLCD);
//    LCD16X2_Set_Cursor(MyLCD, 1, 1);
//    LCD16X2_Write_String(MyLCD, "  DeepBlue");
//    LCD16X2_Set_Cursor(MyLCD, 2, 1);
//    LCD16X2_Write_String(MyLCD, "STM32 Course");
	HAL_ADC_Start_DMA(&hadc1, &soilADCValue,1);
	//HAL_Delay(6000);
	//value = soilADCValue*3.3/4095;
	//soilADCPercent = ((float )soilADCValue / 4095.0) * 100.0;
	//HAL_Delay(500);

	CLCD_4BIT_Init(&LCD1, 16,2, RS_GPIO_Port,RS_Pin, EN_GPIO_Port, EN_Pin,
									D4_GPIO_Port,D4_Pin,D5_GPIO_Port,D5_Pin,
									D6_GPIO_Port, D6_Pin,D7_GPIO_Port,D7_Pin);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		CLCD_SetCursor(&LCD1,0,0);
//		CLCD_WriteString(&LCD1,"IOT Nguyen Quoc");
//		sprintf(LCD_send,"Soil Value: %d",(int)soilValue);
//		CLCD_SetCursor(&LCD1,0,1);
//		CLCD_WriteString(&LCD1, LCD_send);
//		
		//sendUARTData(soilADCPercent);
//		HAL_Delay(6000);
		Toggle_Mode(mode_GPIO_Port, mode_Pin, (uint8_t *)&mode);
		if(mode ==MODE_AUTO ){
//			CLCD_SetCursor(&LCD1,0,0);
//			CLCD_WriteString(&LCD1,"IOT Nguyen Quoc");
			sprintf(LCD_send,"Soil Value: %d",(int)soilValue);
			//scanf(LCD_send," ");
			strcat(LCD_send," ");
//			CLCD_SetCursor(&LCD1,0,1);
//			CLCD_WriteString(&LCD1, LCD_send);
			displayLCD(mode_auto,LCD_send);
		
			sendUARTData(soilADCPercent);
			Read_Light_Sensor_And_Control_LED(CBAS_GPIO_Port, CBAS_Pin, RDEN_GPIO_Port,RDEN_Pin);
			controlPump(soilADCPercent);
			
		}
		else if(mode == MODE_MANUAL){
			
			Toggle_LED(DEN_GPIO_Port,DEN_Pin, RDEN_GPIO_Port, RDEN_Pin);
			Toggle_LED(MBOM_GPIO_Port, MBOM_Pin, RBOM_GPIO_Port, RBOM_Pin);
			displayLCD(LCD_state_light,LCD_state_pump);
			
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|EN_Pin|D4_Pin|D5_Pin
                          |RBOM_Pin|RDEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CBAS_Pin DEN_Pin */
  GPIO_InitStruct.Pin = CBAS_Pin|DEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin EN_Pin D4_Pin D5_Pin
                           RBOM_Pin RDEN_Pin */
  GPIO_InitStruct.Pin = RS_Pin|EN_Pin|D4_Pin|D5_Pin
                          |RBOM_Pin|RDEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : mode_Pin RST_Pin MBOM_Pin */
  GPIO_InitStruct.Pin = mode_Pin|RST_Pin|MBOM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
      soilADCPercent = map(soilADCValue,0,4095,0,100);
			soilValue =soilADCPercent;

    }
}


void sendUARTData(float soilADCPercent) {
  uint32_t currentMillis = HAL_GetTick();  // L?y th?i gian hi?n t?i
  
  // Ki?m tra xem dã d?n lúc g?i d? li?u chua
  if (currentMillis - previousMillis >= interval) {
    // Luu th?i gian hi?n t?i là th?i di?m cu?i cùng d? li?u du?c g?i di
    previousMillis = currentMillis;
//		if(soilADCPercent> 60){
//			sprintf(STATE,"ON");
//		}
//		else if(soilADCPercent <= 60) sprintf(STATE, "OFF");
//		getCurrentDate(currentDate);
//		getCurrentTime(currentTime);
//    
    // G?i d? li?u qua UART
    //sprintf(UART_send, "{\"soilValue\": %4.2f, \"currentDate\": \"%s\", \"currentTime\": \"%s\", \"state\": %s}", soilADCPercent, currentDate, currentTime, STATE);
		sprintf(UART_send,"soilValue: %4.2f",soilADCPercent);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)UART_send, strlen(UART_send));
		while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {}
		//memset(UART_send, 0, sizeof(UART_send));
  }
}
void controlPump(float soilADCPercent){
	if(soilADCPercent > 60.0){
		HAL_GPIO_WritePin(RBOM_GPIO_Port, RBOM_Pin,GPIO_PIN_RESET);
	
	}
	else {
		HAL_GPIO_WritePin(RBOM_GPIO_Port, RBOM_Pin,GPIO_PIN_SET);
	
	}

}


void readADCsoil(){
	HAL_ADC_Start_DMA(&hadc1, &soilADCValue,1);
	//return soilADCValue;

}

void Toggle_LED(GPIO_TypeDef* buttonGPIOPort, uint16_t buttonGPIOPin, GPIO_TypeDef* ledGPIOPort, uint16_t ledGPIOPin) {
    // Ä?á»?c tráº¡ng thÃ¡i cá»§a nÃºt báº¥m
    if (HAL_GPIO_ReadPin(buttonGPIOPort, buttonGPIOPin) == GPIO_PIN_RESET) {
        // Chá»? cho Ä‘áº¿n khi nÃºt báº¥m Ä‘Æ°á»£c tháº£ ra
        while (HAL_GPIO_ReadPin(buttonGPIOPort, buttonGPIOPin) == GPIO_PIN_RESET);
        // Ä?áº£o tráº¡ng thÃ¡i cá»§a LED
        HAL_GPIO_TogglePin(ledGPIOPort, ledGPIOPin);
			if (buttonGPIOPin == MBOM_Pin && HAL_GPIO_ReadPin(RBOM_GPIO_Port,RBOM_Pin) ==GPIO_PIN_RESET){
			sprintf(LCD_state_pump,"pump:ON ");
			//CLCD_Clear(&LCD1);
		
		
		}
		else if(buttonGPIOPin == MBOM_Pin && HAL_GPIO_ReadPin(RBOM_GPIO_Port,RBOM_Pin) ==GPIO_PIN_SET){
			sprintf(LCD_state_pump,"pump:OFF");
			//CLCD_Clear(&LCD1);
		
		
		}
		if (buttonGPIOPin == DEN_Pin && HAL_GPIO_ReadPin(RDEN_GPIO_Port,RDEN_Pin) ==GPIO_PIN_RESET){
			sprintf(LCD_state_light,"light:ON ");
			//CLCD_Clear(&LCD1);
		
		}
		else if(buttonGPIOPin == DEN_Pin && HAL_GPIO_ReadPin(RDEN_GPIO_Port,RDEN_Pin) ==GPIO_PIN_SET){
			sprintf(LCD_state_light,"light:OFF");
			//CLCD_Clear(&LCD1);
		
		
		}
    }
//		if (buttonGPIOPin == MBOM_Pin && HAL_GPIO_ReadPin(RBOM_GPIO_Port,RBOM_Pin) ==GPIO_PIN_RESET){
//			sprintf(LCD_state_pump,"pump:on");
//			//CLCD_Clear(&LCD1);
//		
//		
//		}
//		else{
//			sprintf(LCD_state_pump,"pump:off");
//			//CLCD_Clear(&LCD1);
//		
//		
//		}
//		if (buttonGPIOPin == DEN_Pin && HAL_GPIO_ReadPin(RDEN_GPIO_Port,RDEN_Pin) ==GPIO_PIN_RESET){
//			sprintf(LCD_state_light,"light:on");
//			//CLCD_Clear(&LCD1);
//		
//		}
//		else{
//			sprintf(LCD_state_light,"light:off");
//			//CLCD_Clear(&LCD1);
//		
//		
//		}
}

void displayLCD(char mode[20],char Lcd_send[30]){
			CLCD_SetCursor(&LCD1,0,0);
			CLCD_WriteString(&LCD1,mode);
			//sprintf(LCD_send,"Soil Value: %d",(int)soilValue);
			CLCD_SetCursor(&LCD1,0,1);
			CLCD_WriteString(&LCD1, Lcd_send);
	

}

void Read_Light_Sensor_And_Control_LED(GPIO_TypeDef* sensorPort, uint16_t sensorPin, GPIO_TypeDef* ledPort, uint16_t ledPin) {
    // Ä?á»?c tráº¡ng thÃ¡i cá»§a cáº£m biáº¿n Ã¡nh sÃ¡ng
    if (HAL_GPIO_ReadPin(sensorPort, sensorPin) == GPIO_PIN_RESET) {
        // Ä?iá»?u khiá»ƒn LED dá»±a trÃªn tráº¡ng thÃ¡i cá»§a cáº£m biáº¿n Ã¡nh sÃ¡ng
        HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_SET);  // tat den
    } else {
        HAL_GPIO_WritePin(ledPort, ledPin, GPIO_PIN_RESET);  // bat den
    }
}

void Toggle_Mode(GPIO_TypeDef* modeButtonPort, uint16_t modeButtonPin, volatile uint8_t* mode) {
    // Ä?á»?c tráº¡ng thÃ¡i cá»§a nÃºt chuyá»ƒn Ä‘á»•i cháº¿ Ä‘á»™
    if (HAL_GPIO_ReadPin(modeButtonPort, modeButtonPin) == GPIO_PIN_RESET) {
        // Chá»? nÃºt báº¥m Ä‘Æ°á»£c tháº£ ra
        while (HAL_GPIO_ReadPin(modeButtonPort, modeButtonPin) == GPIO_PIN_RESET);

        // Chuyá»ƒn Ä‘á»•i cháº¿ Ä‘á»™
        if (*mode == MODE_AUTO) {
            *mode = MODE_MANUAL;
        } else {
            *mode = MODE_AUTO;
        }
				
				CLCD_Clear(&LCD1);
		

        // Ä?á»£i má»™t chÃºt Ä‘á»ƒ trÃ¡nh nháº¥n nÃºt quÃ¡ nhanh
        HAL_Delay(300);
    }
}

// Hàm d? chuy?n d?i th?i gian RTC thành chu?i ngày tháng nam
//void RTC_to_DateString(char *buffer, RTC_DateTypeDef *date) {
//  sprintf(buffer, "%02d/%02d/%04d", date->Date, date->Month, date->Year);
//}

//// Hàm d? chuy?n d?i th?i gian RTC thành chu?i gi? phút giây
//void RTC_to_TimeString(char *buffer, RTC_TimeTypeDef *time) {
//  sprintf(buffer, "%02d:%02d:%02d", time->Hours, time->Minutes, time->Seconds);
//}

//// Hàm d? l?y ngày tháng nam hi?n t?i t? RTC
//void getCurrentDate(char *buffer) {
//  RTC_DateTypeDef date;

//  // Ð?c ngày tháng nam hi?n t?i t? RTC
//  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

//  // Chuy?n d?i th?i gian thành chu?i ngày tháng nam
//  RTC_to_DateString(buffer, &date);
//}

//// Hàm d? l?y gi? phút giây hi?n t?i t? RTC
//void getCurrentTime(char *buffer) {
//  RTC_TimeTypeDef time;
//  // Ð?c gi? phút giây hi?n t?i t? RTC
//  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
//  // Chuy?n d?i th?i gian thành chu?i gi? phút giây
//  RTC_to_TimeString(buffer, &time);
//}

// Implement task functions
//void readSensorTask(void *argument) {
//	uint32_t soilSensor;
//	for (;;){
//		//Doc gia tri cam bien
//		HAL_ADC_Start_DMA (&hadc1,&soilSensor,1);
//		osMessageQueuePut(sensorValueQueue, &soilSensor, 0, osWaitForever);
//		osDelay(100);
//	
//		
//		
//	}
//	
//}

//task dieu khien den
//void readSensorTask (void *argument);
//void controlLightTask(void *argument){
//	uint32_t soilSensor;
//	uint32_t soilSensorValue;
//	for(;;){
//		//lay gia tri tu hang doi
//		osMessageQueueGet(sensorValueQueue, &soilSensor,NULL, osWaitForever);
//		//chuyen tin hieu analog
//		soilSensorValue = map(soilSensor,0,4095,0,100);
//		
//		//dieu khien den dua tren gia tri cam bien
//		if (soilSensorValue >50){
//			//Bat den
//			HAL_GPIO_WritePin(RBOM_GPIO_Port, RBOM_Pin, GPIO_PIN_RESET);
//		}
//		else {
//			HAL_GPIO_WritePin(RBOM_GPIO_Port, RBOM_Pin, GPIO_PIN_SET);
//			
//		}
//		
//		
//		
//		
//	}
//	
//}
//void displayLCDTask (void *argument){
////	uint32_t soilSensor;
////  uint32_t soilSensorValue;
//	LCD16X2_Init(MyLCD);
//  LCD16X2_Clear(MyLCD);
//	for(;;){
//	//lay gia tri tu hang doi
////	osMessageQueueGet(sensorValueQueue, &soilSensor,NULL, osWaitForever);
////	//chuyen tin hieu analog
////	soilSensorValue = map(soilSensor,0,4095,0,100);
//	//HAL_Init();
//  //SystemClock_Config();
//  //MX_GPIO_Init();
//  LCD16X2_Init(MyLCD);
//  LCD16X2_Clear(MyLCD);
//  LCD16X2_Set_Cursor(MyLCD, 1, 1);
//  LCD16X2_Write_String(MyLCD, "  DeepBlue");
//  LCD16X2_Set_Cursor(MyLCD, 2, 1);
//  LCD16X2_Write_String(MyLCD, "STM32 Course");

//}
//	}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
