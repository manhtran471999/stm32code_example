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
#include "stm32f1xx_hal.h"//Thu vien Hal
#include "max30100_for_stm32_hal.h"//Thu vien Max
#include "stdio.h"//Thu vien C
#include "rc522.h"//Thu vien RFID
#include "fonts.h"//Thu vien font
#include "ssd1306.h"//Thu vien oled
#include "test.h"
#include "mlx90614.h"//Thu vien nhiet do
I2C_HandleTypeDef hi2c1;//Khoi tao I2C1
I2C_HandleTypeDef hi2c2;//Khoi tao I2C2
SPI_HandleTypeDef hspi1;//Khoi tao
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;//Khoi tao timer 3






/* USER CODE BEGIN PV */
uint8_t bd=0;
uint8_t t=10;
// variable RFID
uint8_t dataUDP_permit[]="Ma the hop le ";
uint8_t dataUDP_deny[]="Ma the khong hop le ";
uint8_t dulieu[9];
uint8_t thutu=0;
uint8_t haha;
float nhietdo=0;
unsigned int dk=0,dk2=0;
unsigned char CardID[5],CardS[5];
unsigned char MyID[5] = {0x29, 0xc7, 0x67, 0x99, 0x10};	//My card on my keys
unsigned char macardht[10];
float temp =0;
unsigned dem=0;
char buffer[20];
float tong=0;
uint8_t num=0;
uint8_t nut=0,mode=0,dkcheck=0;
uint8_t nut2=0;
/* USER CODE END PV */
////RC522
uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
uint8_t MFRC522_ReadRegister(uint8_t addr);
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
void MFRC522_CalulateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
uint8_t MFRC522_SelectTag(uint8_t* serNum);
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
void MFRC522_Init(void);
void MFRC522_Reset(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);
void MFRC522_Halt(void);
////////
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
#ifdef __GNUC__
 /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
 #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
 #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE 
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
 return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==htim3.Instance)
  {
		dem++;
		if(dem==10&&dk==0)
		{
			temp= MLX90614_ReadTemp(0x00,0x07);
			sprintf(buffer,"TEMP:%.2f%cC",temp,0xF8);
			dem=0;
		}
		if(dk==1)
		{
			if(dem>=15&&dem<=40)
			{
				temp= MLX90614_ReadTemp(0x00,0x07);
				tong+=temp;
			}
			if(dem==41)
			{
				tong=tong/25;
				if(tong>=35&&tong<36)
				{
					tong=tong+1;
				}
				else if (tong<35&&tong>=34)
				{
					tong=tong+2;
				}
				else if(tong<34&&tong>=33)
				{
					tong=tong+3;
				}
				dk=2;
			}
		}
		if(dk==2||dk==3)
		{
			dem=0;
		}
		if(dem==80&&dk==5)
		{
			dk=0;
			dem=0;
			tong=0;
			SSD1306_Clear();
		}
		if(HAL_GPIO_ReadPin(button_GPIO_Port,button_Pin)==0)
		{
			nut=1;
		}
	  if(HAL_GPIO_ReadPin(button_GPIO_Port,button_Pin)==1&&mode==1&&nut==1)
		{
			mode=0;
			nut=0;
			dkcheck=0;
			SSD1306_Clear();
		}
	  if(HAL_GPIO_ReadPin(button_GPIO_Port,button_Pin)==1&&mode==0&&nut==1)
		{
			dk=0;
			mode=1;
			tong=0;
			nut=0;
			SSD1306_Clear();
		}
		if(HAL_GPIO_ReadPin(button_GPIO_Port,GPIO_PIN_14)==0)
		{
			nut2=1;
		}
		if(HAL_GPIO_ReadPin(button_GPIO_Port,GPIO_PIN_14)==1&&nut2==1)
		{
			
			dkcheck=0;
			dk=0;
			mode=0;
			tong=0;
			nut=0;
			nut2=0;
			SSD1306_Clear();
			dem=0;
			nhietdo=0;
		}
		
		
	}
}
void blinkled(void)
{
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN11,HIGH);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN11,LOW);
}
int main(void)
{
  uint8_t len;	
  uint16_t times=0; 
	
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	MX_TIM3_Init();
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart1,&bd,1);
	MAX30100_Init(&hi2c1,&huart2);
	MAX30100_SetSpO2SampleRate(MAX30100_SPO2SR_DEFAULT);
	MAX30100_SetLEDPulseWidth(MAX30100_LEDPW_DEFAULT);
	MAX30100_SetLEDCurrent(MAX30100_LEDCURRENT_DEFAULT, MAX30100_LEDCURRENT_DEFAULT);
	MAX30100_SetMode(MAX30100_HRONLY_MODE);
	MFRC522_Init();
	SSD1306_Init();
	SSD1306_GotoXY (0,1);
  SSD1306_Puts ("DAI HOC", &Font_11x18, 1);
  SSD1306_GotoXY (0,20);
  SSD1306_Puts ("GTVT:)", &Font_11x18, 1);
  SSD1306_UpdateScreen(); //display
  HAL_Delay(2000);
	SSD1306_Clear();
  while (1)
  {
		if(mode==0){
		if(dk==0)
		{
			SSD1306_GotoXY (0,0);
			SSD1306_Puts ("->DH GTVT<-",&Font_11x18,1);
      SSD1306_GotoXY (0,16);	
		  SSD1306_Puts(buffer,&Font_11x18, 1);
	    SSD1306_UpdateScreen();
			SSD1306_GotoXY (0,33);
			SSD1306_Puts("QUET THE",&Font_11x18, 1);
			SSD1306_UpdateScreen();
			HAL_Delay(10);
		}
		else if(dk==1)
		{
			sprintf(buffer,"%02x%02x%02x%02x%02x",CardS[0], CardS[1], CardS[2], CardS[3], CardS[4]);
			SSD1306_GotoXY (0,0);
      SSD1306_Puts(buffer,&Font_11x18, 1);
			SSD1306_GotoXY (0,15);
		  SSD1306_Puts("SENSOR TEMP",&Font_11x18, 1);
			SSD1306_UpdateScreen();
		}
		else if(dk==2)
		{
			sprintf(buffer,"%02x%02x%02x%02x%02x",CardS[0], CardS[1], CardS[2], CardS[3], CardS[4]);
			SSD1306_GotoXY (0,0);
      SSD1306_Puts(buffer,&Font_11x18, 1);
			SSD1306_GotoXY (0,15);
			sprintf(buffer,"TEMP:%.2f%cC",tong,0xF8);
		  SSD1306_Puts(buffer,&Font_11x18,1);
			SSD1306_GotoXY (20,35);
		  SSD1306_Puts("HR_|",&Font_11x18, 1);
			SSD1306_GotoXY (60,35);
		  SSD1306_Puts("SPO2",&Font_11x18, 1);
			SSD1306_UpdateScreen();
			haha=0x61;
			HAL_UART_Transmit(&huart1,&haha,1,1000);
			dk=3;
		}
		else if(dk==4)
		{
			sprintf(buffer,"%02x%02x%02x%02x%02x",CardS[0], CardS[1], CardS[2], CardS[3], CardS[4]);
			SSD1306_GotoXY (0,0);
      SSD1306_Puts(buffer,&Font_11x18, 1);
			SSD1306_GotoXY (0,16);
			sprintf(buffer,"TEMP:%.2f%cC",tong,0xF8);
		  SSD1306_Puts(buffer,&Font_11x18,1);
			//SSD1306_GotoXY (20,35);
		  //SSD1306_Puts("HR",&Font_7x10, 1);
			//SSD1306_GotoXY (60,33);
		  //SSD1306_Puts("SPO2",&Font_7x10, 1);
			SSD1306_GotoXY (20,35);
		  SSD1306_Puts(dulieu,&Font_11x18, 1);
			SSD1306_Puts("%",&Font_11x18,1);
			printf("<%02x%02x%02x%02x%02x|",CardS[0], CardS[1], CardS[2], CardS[3], CardS[4]);
		  sprintf(buffer,"%.2f|",tong);		
			HAL_UART_Transmit(&huart2,buffer,5,1000);
		  haha=0x7c;
			HAL_UART_Transmit(&huart2,&haha,1,1000);
			HAL_UART_Transmit(&huart2,dulieu,6,1000);
			SSD1306_UpdateScreen();
			dk=5;	
		}
		if ((MFRC522_Check(CardID) == MI_OK)&&dk==0)			
	  {
			CardS[0]=CardID[0];
			CardS[1]=CardID[1];
			CardS[2]=CardID[2];
			CardS[3]=CardID[3];
			CardS[4]=CardID[4];
			dk=1;
			SSD1306_Clear();
			dem=0;
			
		}
		if(MFRC522_Check(CardID) == MI_OK&&dk>0) 
	  {
  			if(MFRC522_Compare(CardID,CardS) != MI_OK) 
				{
					CardS[0]=CardID[0];
			    CardS[1]=CardID[1];
			    CardS[2]=CardID[2];
			    CardS[3]=CardID[3];
			    CardS[4]=CardID[4];
					SSD1306_Clear();
					dk=1;
					dem=0;
					tong=0;
				}
		}
		}
	  else if(mode==1)
		{
			SSD1306_GotoXY (0,0);
			SSD1306_Puts ("->DH GTVT<-",&Font_11x18,1);
      SSD1306_GotoXY (0,16);
			SSD1306_Puts ("INDEN CARD",&Font_11x18,1);
			if(dkcheck==0)
			{
				SSD1306_GotoXY (0,33);
				SSD1306_Puts("QUET THE",&Font_11x18, 1);
				SSD1306_UpdateScreen();
		  }
			if ((MFRC522_Check(CardID) == MI_OK)&&dkcheck==0)			
			{
				CardS[0]=CardID[0];
				CardS[1]=CardID[1];
				CardS[2]=CardID[2];
				CardS[3]=CardID[3];
				CardS[4]=CardID[4];
				dkcheck=1;
			  sprintf(buffer,"%02x%02x%02x%02x%02x",CardS[0], CardS[1], CardS[2], CardS[3], CardS[4]);
				SSD1306_GotoXY (0,33);
				SSD1306_Puts(buffer,&Font_11x18, 1);
				SSD1306_UpdateScreen();
				printf( "-%02x%02x%02x%02x%02x", CardS[0], CardS[1], CardS[2], CardS[3], CardS[4]);
			}
			if(MFRC522_Check(CardID) == MI_OK&&dkcheck==1) 
			{
  			if(MFRC522_Compare(CardID,CardS) != MI_OK) 
				{
					CardS[0]=CardID[0];
			    CardS[1]=CardID[1];
			    CardS[2]=CardID[2];
			    CardS[3]=CardID[3];
			    CardS[4]=CardID[4];
					sprintf(buffer,"%02x%02x%02x%02x%02x",CardS[0], CardS[1], CardS[2], CardS[3], CardS[4]);
					SSD1306_GotoXY (0,33);
				  SSD1306_Puts(buffer,&Font_11x18, 1);
					SSD1306_UpdateScreen();
					printf( "-%02x%02x%02x%02x%02x",CardS[0],CardS[1],CardS[2],CardS[3],CardS[4]);
					
				}
			}
		}
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==huart1.Instance)
  {
		HAL_UART_Receive_IT(&huart1,&bd,1);
		if(dk2==1&&bd==0x3E)
		{
			
			dk=4;
			dk2=0;
			thutu=0;			
			SSD1306_Clear();
			//HAL_UART_Transmit(&huart2,dulieu,5,1000);
		}
		if(dk2==1)
		{
			dulieu[thutu]=bd;
			thutu++;
		}
		if(bd==0x3C)
		{
			dk2=1;
			thutu=0;
		}	
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);
	
	
	
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

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
