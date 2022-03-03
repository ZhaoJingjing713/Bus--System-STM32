/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Author: Enrique Phan
  * Date :	2020/09/20
  * Github: PHANzgz
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../../Drivers/OV7670/OV7670.h"
#include "LCD.h"
#include "fonts.h"
#include "hot_camera.h"
#include "GPS.h"
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_GRAYSCALE 1
#define RX_BUF_MAXLEN 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
#if USE_GRAYSCALE
uint32_t frame_buffer[OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2];
#else
uint32_t frame_buffer[OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2];
#endif
uint8_t new_capture = 0;



uint8_t hot_buffer[7];	//中断缓冲
uint8_t T[8]; //温度
uint8_t wifi_rec[1];
uint8_t tem[8];
uint8_t count_send_gps_time=0;

int total_people=0;

uint8_t rdata[1];
uint8_t rdata1[1];
uint16_t point=0;
uint16_t hot_point=0;
uint8_t ifOneline=false;
uint8_t USART_RX_BUF[RX_BUF_MAXLEN];
GPS_Data GPSData;
int num=0;
//默认地址
uint8_t latitude[latitude_Length]="3888.8519";
uint8_t longitude[longitude_Length]="12153.6728";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DCMI_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_FMC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static void onFrameCallback();
//void get_t();
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
	tem[0]=0xEE;
	tem[1]=0xE1;
	tem[2]=0x01;
	tem[3]=0x55;
	tem[4]=0xFF;
	tem[5]=0xFC;
	tem[6]=0xFD;
	tem[7]=0xFF;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

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
  MX_DCMI_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Init(&huart4);
  HAL_UART_Init(&huart1);
  HAL_UART_Init(&huart7);
  HAL_UART_Init(&huart2);
  HAL_GPIO_WritePin(CAMERA_PWR_DWN_GPIO_Port, CAMERA_PWR_DWN_Pin, GPIO_PIN_RESET); // Turn on camera
  ov7670_init(&hdcmi, &hdma_dcmi, &hi2c1);

#if USE_GRAYSCALE
  ov7670_config(OV7670_MODE_QVGA_YUV);
#else
  ov7670_config(OV7670_MODE_QVGA_RGB565);
#endif
  ov7670_registerCallback(NULL, NULL, &onFrameCallback);
  lcd_init();
  ILI9341_GramScan(3);
  ILI9341_Clear(0, 0, 320,240 , WHITE);
  ILI9341_DispStringLine_EN(LINE(120),"        Bus system based on stm32");
  HAL_Delay(1000);
  LCD_ClearLine(LINE(0));
  HAL_UART_Receive_IT(&huart7, rdata, 1);
  HAL_UART_Receive_IT(&huart2, wifi_rec, 1);
  HAL_UART_Receive_IT(&huart1, hot_buffer, 7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	HAL_UART_Transmit(&huart1, &tem, 8, 100);
//	HAL_UART_Receive_IT(&huart1, hot_buffer, 1);
	  int a=1;
	  //lcd_auto_clear(500);
#if USE_GRAYSCALE
	  // DCMI setting "Byte select" must be "capture every other byte to ignore Cb or Cr data"
	  // DCMI setting "Byte select start" must be "Interface captures second data"
	  if (new_capture){
		  LCD_ImgShow_gray((unsigned char*) frame_buffer);
		  if (HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin)==0){
			  total_people++;
			  HAL_Delay(1000);
			  char mark[5]="IMG$";
			  char mark2[10]="$IMGEND";
			  HAL_UART_Init(&huart4);
			  HAL_UART_Transmit(&huart4, &mark,5, 100);
			  uint32_t tem_data[OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2];
			  for(int i=0;i<OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2;i++){
				  tem_data[i]=frame_buffer[i];
			  }
			  grey_transmit((uint8_t*)tem_data);
			  HAL_Delay(1000);
			  HAL_UART_Transmit(&huart4, &mark2, 10, 1000);
			  uint8_t *rec=(char*)malloc(sizeof(char));
//			  while(HAL_UART_Receive(&huart2, rec, 1,100)!=HAL_OK);
//			  HAL_UART_Transmit(&huart4, rec, 1,100);
			  char name[5];
			  int find=1;
			  if(wifi_rec[0]=='1')
				  strcpy(name,"mym");
			  else if(wifi_rec[0]=='2')
				  strcpy(name,"dxc");
			  else if(wifi_rec[0]=='3')
				  strcpy(name,"zjj");
			  else
				  find=0;
			  free(rec);
			  if(find){
				  char showline[100]="             Welcome ";
				  strcat(showline,name);
				  ILI9341_Clear(0, 200, 320,240 , WHITE);
				  ILI9341_DispStringLine_EN(LINE(13),showline);
	//				  hot_camera_init(huart1);
	//				  HAL_UART_Receive_IT(&huart1, hot_buffer, 10);
	//				  HAL_Delay(1000);
	//				  HAL_UART_Transmit(&huart4, hot_data, 2055, 1000);
	//				  float high=hot_camer_get_temperature(hot_data);
//				  get_t();
				  char str[50]="           Your temperature is ";
				  strcat(str,T);
				  //HAL_UART_Transmit(&huart4, str, 50, 100);
				  ILI9341_DispStringLine_EN(LINE(14),str);
			  }else{
				  char showline[50]="             Recognize failed ";
				  ILI9341_Clear(0, 200, 320,240 , WHITE);
				  ILI9341_DispStringLine_EN(LINE(13),showline);
			  }
			  HAL_Delay(1000);
		  }
		  else if(HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin)==0){
			if(total_people>0)
			  total_people--;
			HAL_Delay(2000);
		  }
	  }
#else
	  if (new_capture)

		  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0){
			  LCD_ImgShow((uint8_t*)frame_buffer);
			  if (HAL_GPIO_ReadPin(SWITCH2_GPIO_Port, SWITCH2_Pin)==0){
				  total_people++;
				  char mark[5]="IMG$";
				  char mark2[10]="$IMGEND";
				  HAL_UART_Init(&huart4);
				  HAL_UART_Transmit(&huart4, &mark,5, 100);
				  for(int i=0;i<OV7670_QVGA_WIDTH * OV7670_QVGA_HEIGHT/2;i++){
					HAL_UART_Transmit(&huart4, &frame_buffer[i],4, 10000);
					for(int i=0;i<30000;i++);
				  }
					HAL_Delay(1000);
				  HAL_UART_Transmit(&huart4, &mark2, 10, 1000);
				  ILI9341_Clear(0, 200, 320,240 , WHITE);
				  ILI9341_DispStringLine_EN(LINE(13),"              xxx Welcome");
//				  hot_camera_init(huart1);
//				  HAL_UART_Receive_IT(&huart1, hot_buffer, 10);
//				  HAL_Delay(1000);
//				  HAL_UART_Transmit(&huart4, hot_data, 2055, 1000);
//				  float high=hot_camer_get_temperature(hot_data);
				  char str_t[10];
 				  int high=get_t();
				  itoa(high,str_t,10);
				  char tem_c=str_t[2];
				  str_t[2]='.';
				  str_t[3]=tem_c;
				  str_t[4]='\0';
				  char str[50]="           Your temperature is ";
                  strcat(str,str_t);
				  //HAL_UART_Transmit(&huart4, str, 50, 100);
				  ILI9341_DispStringLine_EN(LINE(14),str);
				  HAL_Delay(1000);
			  }
			  else if(HAL_GPIO_ReadPin(SWITCH1_GPIO_Port, SWITCH1_Pin)==0){
				if(total_people>=0)
				  total_people--;
			  }
		  }
		  else
			  LCD_ImgShow_hot(hot_data);

#endif

	  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0){
		  //ov7670_startCap(OV7670_CAP_SINGLE_FRAME, (uint32_t)frame_buffer);
		  ov7670_startCap(OV7670_CAP_CONTINUOUS, (uint32_t)frame_buffer);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t start[10]="START\n";
	  uint8_t end[10]="END\n";
	  //HAL_UART_Transmit(&huart4, start, 10, 100);
	  if(ifOneline){
		  //HAL_UART_Transmit(&huart4, &USART_RX_BUF,strlen(USART_RX_BUF),1000);
			  strcpy(GPSData.GPS_Buffer,USART_RX_BUF);
			  if(strstr(USART_RX_BUF,"GNRMC")!=NULL){
				  //HAL_UART_Transmit(&huart4, end, 10,1000);
				  transfer(USART_RX_BUF);
				  if(GPSData.isUseful){
					  HAL_GPIO_WritePin(Green_LED_GPIO_Port,Green_LED_Pin, 0);
					  if(HAL_GPIO_ReadPin(Yellow_LED_GPIO_Port, Yellow_LED_Pin)==0)
						  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin,1);
				  }else{
					  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, 0);
					  if(HAL_GPIO_ReadPin(Green_LED_GPIO_Port,Green_LED_Pin)==0)
						  HAL_GPIO_WritePin(Green_LED_GPIO_Port,Green_LED_Pin,1);
				  }
				  if(num<0)
					  num=0;
				  uint8_t message[messageLength],tem[3];
				  if(num<=9){
					  tem[0]=(num+48);
					  tem[1]='\0';
				  }
				  else{
					  tem[0]=(num/10+48);
					  tem[1]=(num-num/10*10+48);
				  }
				  HAL_Delay(2);
				 // HAL_UART_Transmit(&huart4, GPSData.GPS_Buffer, sizeof(GPSData.GPS_Buffer), 100);
				  HAL_Delay(2);
				  memcpy(message,GPSData.address,sizeof(GPSData.address));
				  strcat(message,",");
				  char people_num[3];
				  itoa(total_people,people_num,10);
				  strcat(message,people_num);
				  strcat(message,",");
				  char gps_end[8]="$GPSEND";
				  strcat(message,gps_end);
				  HAL_Delay(1);
				  if(count_send_gps_time++==10){
					  HAL_UART_Transmit(&huart4, message, sizeof(message), 100);
					  count_send_gps_time=0;
				  }
				  HAL_Delay(1);
			  }
			  ifOneline=0;
			  memset(USART_RX_BUF,0,sizeof(USART_RX_BUF));
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_SPI1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_8);
}

/**
  * @brief DCMI Initialization Function
  * @param None
  * @retval None
  */
static void MX_DCMI_Init(void)
{

  /* USER CODE BEGIN DCMI_Init 0 */

  /* USER CODE END DCMI_Init 0 */

  /* USER CODE BEGIN DCMI_Init 1 */

  /* USER CODE END DCMI_Init 1 */
  hdcmi.Instance = DCMI;
  hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
  hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
  hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
  hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
  hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
  if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DCMI_Init 2 */

  /* USER CODE END DCMI_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0xC010151E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 460800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 38400;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_ENABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 9;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 12;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BG_Pin|CAMERA_PWR_DWN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Green_LED_Pin|Red_LED_Pin|myLCD_DC_Pin|WIFI_RST_Pin
                          |WIFI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SWITCH1_Pin SWITCH2_Pin */
  GPIO_InitStruct.Pin = SWITCH1_Pin|SWITCH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BG_Pin CAMERA_PWR_DWN_Pin CAMERA_RESET_Pin */
  GPIO_InitStruct.Pin = BG_Pin|CAMERA_PWR_DWN_Pin|CAMERA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_LED_Pin Red_LED_Pin myLCD_DC_Pin WIFI_CS_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin|Red_LED_Pin|myLCD_DC_Pin|WIFI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : WIFI_RST_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(WIFI_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Yellow_LED_Pin */
  GPIO_InitStruct.Pin = Yellow_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Yellow_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void grey_transmit(uint8_t *data){
	for(int i=1;i<320*240*2;i++){
			int16_t gray = (uint16_t) data[i++];
			HAL_UART_Transmit(&huart4, &gray, 1, 1000);

	}
}
static void onFrameCallback(){
	new_capture = 1;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart1){
		 //HAL_UART_Transmit(&huart4, hot_buffer,7, 1000);
		strcpy(T,hot_buffer);
		T[8]='\0';
		HAL_UART_Receive_IT(huart, hot_buffer, 7);	//用此种发送方式在回调函数中，当发送的数据没有10位时，会让返回的数据不正�???????
	}else if(huart==&huart7){
		//HAL_UART_Transmit(&huart4, rdata, 1, 10);
		if(rdata[0]=='$')
			point=0;
		USART_RX_BUF[point++]=rdata[0];

		if(rdata[0]=='\n'){
			ifOneline=1;
			point=0;
		}
		HAL_UART_Receive_IT(huart, rdata, 1);	//用此种发送方式在回调函数中，当发送的数据没有10位时，会让返回的数据不正�???????
	}else if(huart==&huart2){
		//HAL_UART_Transmit(&huart4, wifi_rec, 1, 100);
		HAL_UART_Receive_IT(huart, wifi_rec, 1);
	}
//	HAL_UART_Transmit(&huart1, OT_UartBuffer, 10, 1000);
}
void GPS_Init()
{
	memset(GPSData.GPS_Buffer,0,GPS_Buf_Length);
	memset(GPSData.latitude,0,latitude_Length);
	memset(GPSData.longitude,0,longitude_Length);
	GPSData.isGetData=false;
	GPSData.isUseful=false;
}
void transfer(uint8_t *string)
{
	if(strstr(USART_RX_BUF,"GNRMC")==NULL)
		return;
	char str1[100],str2[100],str3[100];
	int i,count,j,t;
	count=0;
	for(i=0;string[i]!='\0';i++)
	{
		if(string[i]==',')
		{
			count++;
			if(count==2)
			{
				t=0;
				for(j=i+1;string[j]!=',';j++)
				{
					str1[t]=string[j];
					t++;
				}
				str1[t]='\0';
			}
			if(count==3)
			{
				t=0;
				for(j=i+1;string[j]!=',';j++)
				{
					str2[t]=string[j];
					t++;
				}
				str2[t]='\0';
			}
			if(count==5)
			{
				t=0;
				for(j=i+1;string[j]!=',';j++)
				{
					str3[t]=string[j];
					t++;
				}
				str3[t]='\0';
			}
		}
	}
	if(str1[0]=='A'){
		GPSData.isUseful=1;
		strcpy(GPSData.latitude,str2);
		strcpy(GPSData.longitude,str3);
		strcpy(latitude,str2);
		strcpy(longitude,str3);
	}else{
		GPSData.isUseful=0;
		strcpy(GPSData.latitude,latitude);
		strcpy(GPSData.longitude,longitude);
	};
	memcpy(GPSData.address,GPSData.latitude,latitude_Length);
	strcat(GPSData.address,",");
	strcat(GPSData.address,GPSData.longitude);
	//strcat(GPSData.address,"\n");
}
void parseGPS()
{
	char *subString;
	char *subNext;
	if(GPSData.isGetData){
		GPSData.isGetData=false;
		for(int i=0;i<9;i++){
			if((subString=strstr(GPSData.GPS_Buffer,","))!=NULL){
				subString++;
				if((subNext=strstr(subString,","))!=NULL){
					char usefullBuffer[2];
					switch(i)
					{
						case 1:memcpy(usefullBuffer,subString,subNext-subString);
						case 2:memcpy(GPSData.latitude,subString,subNext-subString); break;
						case 4:memcpy(GPSData.longitude,subString,subNext-subString); break;
						default:break;
					}
					subString=subNext;

					if(usefullBuffer[0]=='A')
						GPSData.isUseful=true;
					else if(usefullBuffer[0]=='V')
					{
						GPSData.isUseful=false;
					}
				}
			}
		}
	}
}

void printGPS()
{
	parseGPS();
	if(!GPSData.isUseful){
		memcpy(GPSData.latitude,latitude,sizeof(latitude));
		memcpy(GPSData.longitude,longitude,sizeof(longitude));
	}
	GPSData.isGetData=false;
}
//void get_t()
//{
//	do{
//	for(int i=0;i<5;i++){
//		T[i]=hot_buffer[i+1];
//	}
//	T[6]='\0';
//	}while(strstr(T,'}')!=NULL||strstr(T,'{')!=NULL);
//}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
