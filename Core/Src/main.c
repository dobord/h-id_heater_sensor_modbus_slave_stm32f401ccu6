/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include "Modbus.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "heater_modbus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define HID_INTERNAL_SENSOR_TBL(R,X)\
	/*    NAME    TYPE             N   I   TITLE              			*/ \
	R(X	, vref	, voltage 		,  0 , 1 , "Опорное напряжение"			)\
	R(X	, vbat	, voltage 		,  1 , 1 , "Напряжение батареи часов"	)\
	R(X	, tcpu	, temperature 	,  0 , 1 , "Температура CPU"			)\
	R(X	, t1 	, temperature 	,  1 , 0 , "Датчик 1"					)\
	R(X	, t2	, temperature 	,  2 , 0 , "Датчик 2"					)\
	R(X	, t3	, temperature 	,  3 , 0 , "Датчик 3"					)\
	R(X	, t4	, temperature 	,  4 , 0 , "Датчик 4"					)\

#define HID_HEATER_DECL_INTERNAL_SENSOR_ID_ENUM(PREFIX, NAME, TYPE, NUMBER, INVISIBLE, TITLE)\
    PREFIX##NAME,

#define HID_HEATER_DECL_INTERNAL_SENSOR_ID_DESC(PREFIX, NAME, TYPE, NUMBER, INVISIBLE, TITLE)\
{ \
	.number = NUMBER, \
	.invisible = INVISIBLE, \
	.sensor_type = sensor_type_##TYPE \
},

#define HID_HEATER_DECL_SENSOR_TYPE_DESC(PREFIX, INIT, SYMBOL, NAME, TITLE, DIMENSION)\
{ \
	.type = sensor_type_##NAME, \
	.symbol = #SYMBOL, \
	.name = #NAME, \
	.title = TITLE "", \
	.dimension = DIMENSION "" \
},


#define MY_MIN(A,B) ((A) < (B)? (A) : (B))
#define MY_MAX(A,B) ((A) > (B)? (A) : (B))
#define MY_CLAMP(Vmin,V,Vmax) (MY_MAX((Vmin), MY_MIN((V),(Vmax))))

enum InternalSensorID
{
	HID_INTERNAL_SENSOR_TBL(HID_HEATER_DECL_INTERNAL_SENSOR_ID_ENUM, isid_)
	max_num_of_internal_sensors,
	length_of_ctl_reg_internal_sensors = (max_num_of_internal_sensors + 15)/16
};

enum InternalRelayID
{
	max_num_of_internal_relays = 8,
	length_of_ctl_reg_internal_relays = (max_num_of_internal_relays + 15)/16
};


enum HeaterSystemConfigConstatnts
{
	hscc_short_io_addr = 16,
	hscc_short_io_version = 1,
	hscc_short_io_length = 9,
	hscc_long_io_version = 1,
	hscc_long_io_length = 2 + MAX_BUFFER/2,
	hscc_software_type = hsst_sensor_temperature,
};


/// Память доступная для внешнего управления устройством
typedef struct DeviceMemoryModbus
{
	/// Регистры состояния внутренних реле
	uint16_t reserved_relay_out[16];

	/// Начало порта обмена короткими сообщениями
	/// (смещение строго = 16 для любого устройства)
	uint16_t short_response;
	uint16_t short_request;
	uint16_t short_data[hscc_short_io_length];

	/// Наличие данных от внутренних датчиков
	uint16_t int_sens_status[length_of_ctl_reg_internal_sensors];

	/// Данные внутренних датчиков
	uint16_t int_sens_value[max_num_of_internal_sensors];

	/// Регистр управления устройством
	DeviceControlRegister_SensorTemperature dev_ctl;

	/// Начало порта обмена длинными сообщениями
	uint16_t long_response;
	uint16_t long_request;
	uint16_t long_data[hscc_long_io_length];

} DeviceMemoryModbus;

enum Constants
{
	DeviceMemoryModbusSize = sizeof(DeviceMemoryModbus),
	DeviceMemoryModbusLength = DeviceMemoryModbusSize/2,
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_REFRESH_PERIOD   1000    /* Internal temperature refresh period */
#define MAX_CONVERTED_VALUE   4095    /* Max converted value */
#define AMBIENT_TEMP            25    /* Ambient Temperature */
#define VSENS_AT_AMBIENT_TEMP  760    /* VSENSE value (mv) at ambient temperature */
#define AVG_SLOPE               25    /* Avg_Solpe multiply by 10 */
#define VREF                  3300

enum ADC_Constants
{
	adc_num_channels = 6,
	adc_num_samples = 64
};

static const float s_pt1000_r0 = 1000.0f;
static const float s_pt1000_a = 3.9083e-3f;
static const float s_pt1000_b = -5.775e-7f;
//static const float s_pt1000_c = -4.183e-12f;

static const float s_termo_zero_c = -273.15f;
static const float s_ntc10k_a = 1.111e-3f;
static const float s_ntc10k_b = 237.987e-6f;
static const float s_ntc10k_c = 65.468e-9f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskUSB */
osThreadId_t myTaskUSBHandle;
const osThreadAttr_t myTaskUSB_attributes = {
  .name = "myTaskUSB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskRS485 */
osThreadId_t myTaskRS485Handle;
const osThreadAttr_t myTaskRS485_attributes = {
  .name = "myTaskRS485",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
volatile modbusHandler_t g_modbus_usb;
volatile modbusHandler_t g_modbus_rs485;
volatile DeviceMemoryModbus g_modbus_mem = { 0 };

volatile uint16_t g_adc_values[adc_num_channels] = { 0 };
volatile uint16_t g_adc_avg_values[adc_num_channels] = { 0 };
volatile uint16_t g_adc_sko_values[adc_num_channels] = { 0 };
volatile uint16_t g_adc_dma_buffer[adc_num_channels] = { 0 };
volatile uint16_t g_adc_samples[adc_num_samples][adc_num_channels] = { 0 };
volatile uint16_t g_adc_num_samples = 0;
volatile uint16_t g_adc_sample_pos = 0;
volatile uint8_t g_adc_update = 0;

//static float g_r1[4] = {10660, 10000, 10380, 9715};
static const float g_r1[4] = {10660, 10000, 10660, 10000};

static const SensorTypeDesc s_sensor_type_desc[sensor_type__num] = {
	HID_HEATER_SENSOR_TYPE_TBL(HID_HEATER_DECL_SENSOR_TYPE_DESC, )
};

static const SensorDescription s_internal_sensors_desc[max_num_of_internal_sensors] = {
	HID_INTERNAL_SENSOR_TBL(HID_HEATER_DECL_INTERNAL_SENSOR_ID_DESC, isid_)
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);
void StartTaskUSB(void *argument);
void StartTaskRS485(void *argument);

/* USER CODE BEGIN PFP */
static float calc_temperature_ntc10k(float r);
static float calc_temperature_pt1000(float r);
static bool testU16Bit(volatile uint16_t* mem, uint16_t bit_idx);
static void setU16Bit(volatile uint16_t* mem, uint16_t bit_idx, bool value);
static void handle_short_request();
static void handle_long_request();
static void handle_requests(void);
static void write_u16_masked(uint16_t reg, uint16_t data, uint16_t mask);
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_adc_dma_buffer, adc_num_channels);

  /* Modbus Slave initialization */
  g_modbus_usb.uModbusType = MB_SLAVE;
  //g_modbus_usb.uModbusType = MB_MASTER;
  g_modbus_usb.xTypeHW = USART_HW_DMA;
  g_modbus_usb.port =  &huart1; // This is the UART port connected to STLINK
  g_modbus_usb.u8id = 1; //slave ID, always different than zero
  //g_modbus_usb.u8id = 0; //slave ID for master always 0
  g_modbus_usb.u16timeOut = 1000;
  g_modbus_usb.EN_Port = NULL; // No RS485
   //g_modbus_usb.EN_Port = LD2_GPIO_Port; // RS485 Enable
   //g_modbus_usb.EN_Pin = LD2_Pin; // RS485 Enable
  g_modbus_usb.u16regs = (uint16_t*) &g_modbus_mem;
  g_modbus_usb.u16regsize= sizeof(g_modbus_mem)/sizeof(uint16_t);
   //Initialize Modbus library
  ModbusInit((modbusHandler_t *)&g_modbus_usb);
  //Start capturing traffic on serial Port
  ModbusStart((modbusHandler_t *)&g_modbus_usb);


  /* Modbus Slave initialization */
  g_modbus_rs485.uModbusType = MB_SLAVE;
  //g_modbus_rs485.uModbusType = MB_MASTER;
  g_modbus_rs485.xTypeHW = USART_HW_DMA;
  g_modbus_rs485.port =  &huart2; // This is the UART port connected to RS485
  g_modbus_rs485.u8id = 1; //slave ID, always different than zero
  //g_modbus_rs485.u8id = 0; //slave ID for master always 0
  g_modbus_rs485.u16timeOut = 1000;
  //g_modbus_rs485.EN_Port = NULL; // No RS485
  g_modbus_rs485.EN_Port = RS485_RSE_GPIO_Port; // RS485 Enable
  g_modbus_rs485.EN_Pin = RS485_RSE_Pin; // RS485 Enable
  g_modbus_rs485.u16regs = (uint16_t*) &g_modbus_mem;
  g_modbus_rs485.u16regsize= sizeof(g_modbus_mem)/sizeof(uint16_t);
   //Initialize Modbus library
  ModbusInit((modbusHandler_t*)&g_modbus_rs485);
  //Start capturing traffic on serial Port
  ModbusStart((modbusHandler_t*)&g_modbus_rs485);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskUSB */
  myTaskUSBHandle = osThreadNew(StartTaskUSB, NULL, &myTaskUSB_attributes);

  /* creation of myTaskRS485 */
  myTaskRS485Handle = osThreadNew(StartTaskRS485, NULL, &myTaskRS485_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SENSOR_T3_Pin|SENSOR_T4_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, _74HC595D_STCP_Pin|RS485_RSE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED3_Pin */
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SENSOR_T3_Pin SENSOR_T4_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = SENSOR_T3_Pin|SENSOR_T4_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : _74HC595D_STCP_Pin RS485_RSE_Pin */
  GPIO_InitStruct.Pin = _74HC595D_STCP_Pin|RS485_RSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if( g_adc_num_samples < adc_num_samples )
		++g_adc_num_samples;

	memcpy((uint16_t*)g_adc_samples + g_adc_sample_pos * adc_num_channels,
			(uint16_t*)g_adc_dma_buffer,
			adc_num_channels * sizeof(uint16_t));
	++g_adc_sample_pos;

	if( g_adc_sample_pos >= adc_num_samples )
		g_adc_sample_pos = 0;

	g_adc_update = 1;
}

static float calc_temperature_pt1000(float r)
{
	float ret = (-s_pt1000_a + sqrt(s_pt1000_a*s_pt1000_a  - 4*s_pt1000_b*(1 - r/s_pt1000_r0)))/(2*s_pt1000_b);
	return ret;
}

static float calc_temperature_ntc10k(float r)
{
	float lnr = log(r);
	float ret = s_termo_zero_c + 1.0f/(s_ntc10k_a + s_ntc10k_b*lnr + s_ntc10k_c * lnr*lnr*lnr);
	return ret;
}

static bool testU16Bit(volatile uint16_t* mem, uint16_t bit_idx)
{
	return (mem[bit_idx >> 4] >> (bit_idx & 0xf)) & 1;
}

static void setU16Bit(volatile uint16_t* mem, uint16_t bit_idx, bool value)
{
	if( value )
		mem[bit_idx >> 4] |= (1 << (bit_idx & 0xf));

	else
		mem[bit_idx >> 4] &= ~(1 << (bit_idx & 0xf));
}

static void write_u16_masked(uint16_t reg, uint16_t data, uint16_t mask)
{
	uint16_t* dst = ((uint16_t*)&g_modbus_mem) + reg;
	uint16_t r = *dst;
	r &= ~(mask & ~data);
	r |= (mask & data);
	*dst = r;
}

static void handle_short_request()
{
	uint16_t req = g_modbus_mem.short_request;
	g_modbus_mem.short_response = 0;
	g_modbus_mem.short_request = 0;
	volatile uint16_t* nwords = g_modbus_mem.short_data;
	uint16_t nargs = *nwords;
	UnionRegister* ret = (UnionRegister*)(g_modbus_mem.short_data + 1);

	if( nargs >= hscc_short_io_length )
	{
		// ошибка в кол-ве аргументов запроса
		*nwords = 0;
		memset(ret->u8, 0, (hscc_short_io_length - 1) * 2);
		return;
	}

	switch( req )
	{
	case ssr_get_short_io_version:
		*nwords = 1;
		ret->u16[0] = hscc_short_io_version;
		break;

	case ssr_get_short_io_length:
		*nwords = 1;
		ret->u16[0] = hscc_short_io_length;
		break;

	case ssr_get_long_io_version:
		*nwords = 1;
		ret->u16[0] = hscc_long_io_version;
		break;

	case ssr_get_long_io_length:
		*nwords = 1;
		ret->u16[0] = hscc_long_io_length;
		break;

	case ssr_get_software_type:
		*nwords = 1;
		ret->u16[0] = hscc_software_type;
		break;

	case ssr_get_device_memory_modbus_length:
		*nwords = 1;
		ret->u16[0] = DeviceMemoryModbusLength;
		break;

	case ssr_get_rs485_id:
		*nwords = 1;
		ret->u16[0] = g_modbus_rs485.u8id;
		break;

	case ssr_set_rs485_id:
	{
		*nwords = 1;
		uint8_t ecode = !g_modbus_rs485.u8id;

		if( g_modbus_rs485.u8id )
			g_modbus_rs485.u8id = ret->u16[0];

		ret->u16[0] = ecode;
		break;
	}

	case ssr_get_devid:
		*nwords = 2;
		ret->u32[0] = HAL_GetDEVID();
		break;

	case ssr_get_revid:
		*nwords = 2;
		ret->u32[0] = HAL_GetREVID();
		break;

	case ssr_get_uid:
		*nwords = 6;
		ret->u32[0] = HAL_GetUIDw0();
		ret->u32[1] = HAL_GetUIDw1();
		ret->u32[2] = HAL_GetUIDw2();
		break;

	case ssr_get_addr_long_io:
		*nwords = 1;
		ret->u16[0] = offsetof(DeviceMemoryModbus, long_response) / 2;
		break;

	case ssr_get_addr_dev_ctl:
		*nwords = 1;
		ret->u16[0] = offsetof(DeviceMemoryModbus, dev_ctl) / 2;
		break;

	case ssr_get_addr_int_sens_status:
		*nwords = 1;
		ret->u16[0] = offsetof(DeviceMemoryModbus, int_sens_status) / 2;
		break;

	case ssr_get_addr_int_sens_value:
		*nwords = 1;
		ret->u16[0] = offsetof(DeviceMemoryModbus, int_sens_value) / 2;
		break;

	case ssr_get_max_num_of_internal_sensors:
		*nwords = 1;
		ret->u16[0] = max_num_of_internal_sensors;
		break;

	case ssr_get_max_num_of_internal_relays:
		*nwords = 1;
		ret->u16[0] = max_num_of_internal_relays;
		break;

	case ssr_replace_masked_bits:
	{
		*nwords = 0;
		ReplaceMaskedBitsHeader* h = (ReplaceMaskedBitsHeader*) ret->u16;
		ReplaceMaskedBitsData* d;

		while( nargs >= (sizeof(*h) + sizeof(*d))/2 )
		{
			d = (ReplaceMaskedBitsData*)(h + 1);

			nargs -= sizeof(*h)/2;

			if( h->num * sizeof(*d)/2 > nargs )
				// bug
				break;

			if( h->reg_addr_start >= DeviceMemoryModbusLength )
				// bug
				break;

			if( h->reg_addr_start + h->num > DeviceMemoryModbusLength )
				// bug
				break;

			uint16_t* dst = ((uint16_t*)&g_modbus_mem) + h->reg_addr_start;
			for( int n = h->num; n; --n, ++d, ++dst )
			{
				uint16_t r = *dst;
				r &= ~(d->mask & ~d->data);
				r |= (d->mask & d->data);
				*dst = r;
			}

			h = (ReplaceMaskedBitsHeader*)d;
		}
		break;
	}

	case ssr_rtc0_read_date_time:
	{
		RtcDataTime* r = (RtcDataTime*) ret->u8;
		*nwords = (sizeof(*r) + 1) / 2;
		RTC_TimeTypeDef t;
		RTC_DateTypeDef d;
		HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
		r->Year = d.Year;
		r->Month = d.Month;
		r->Date = d.Date;
		r->WeekDay = d.WeekDay;
		r->Hours = t.Hours;
		r->Minutes = t.Minutes;
		r->Seconds = t.Seconds;
		r->_1s256 = ( t.SubSeconds << 8 ) / ( 1 + t.SecondFraction );
		break;
	}

	case ssr_rtc0_write_date_time:
	{
		*nwords = 0;
		RtcDataTime* r = (RtcDataTime*) ret->u8;
		RTC_TimeTypeDef t;
		RTC_DateTypeDef d;
		d.Year = r->Year;
		d.Month = r->Month;
		d.Date = r->Date;
		d.WeekDay = r->WeekDay;
		t.Hours = r->Hours;
		t.Minutes = r->Minutes;
		t.Seconds = r->Seconds + 1;
		uint16_t wait_ms = ( (uint32_t)(256 - r->_1s256) * 1000ul ) >> 8;
		osDelay(wait_ms);
		HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
		HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);
		break;
	}

	default:
		*nwords = 0;
		memset(ret->u8, 0, (hscc_short_io_length - 1) * 2);
		return;
	}

	g_modbus_mem.short_response = req;
}

static void handle_long_request()
{
	uint16_t req = g_modbus_mem.long_request;
	g_modbus_mem.long_response = 0;
	g_modbus_mem.long_request = 0;
	volatile uint16_t* nwords = g_modbus_mem.long_data;
	uint16_t nargs = *nwords;

	if( nargs >= hscc_long_io_length )
	{
		// ошибка в кол-ве аргументов запроса
		*nwords = 0;
		return;
	}

	UnionRegister* ret = (UnionRegister*)(g_modbus_mem.long_data + 1);

	switch( req )
	{
	case slr_get_long_io_version:
		*nwords = 1;
		ret->u16[0] = hscc_long_io_version;
		break;

	case slr_get_long_io_length:
		*nwords = 1;
		ret->u16[0] = hscc_long_io_length;
		break;

	case slr_call_short_request_list_no_args_ret_1u16:
		for( int i = 0; i < nargs; ++i )
		{
			g_modbus_mem.short_request = ret->u16[i];
			g_modbus_mem.short_data[0] = 0;
			handle_short_request();
			ret->u16[i] = g_modbus_mem.short_data[1];
		}
		break;

	case slr_call_short_request_list_args_1u16_ret_1u16:
		for( int i = 0; i < nargs/2; ++i )
		{
			g_modbus_mem.short_request = ret->u16[2*i];
			g_modbus_mem.short_data[0] = 1;
			g_modbus_mem.short_data[1] = ret->u16[2*i + 1];
			handle_short_request();
			ret->u16[2*i] = g_modbus_mem.short_response;
			ret->u16[2*i + 1] = g_modbus_mem.short_data[1];
		}
		break;

	case slr_get_int_sens_desc:
	{
		memcpy(ret->u8, s_internal_sensors_desc, sizeof(s_internal_sensors_desc));
		*nwords = sizeof(s_internal_sensors_desc)/2;
		break;
	}

	case slr_replace_masked_bits:
	{
		*nwords = 0;
		ReplaceMaskedBitsHeader* h = (ReplaceMaskedBitsHeader*) ret->u16;
		ReplaceMaskedBitsData* d;

		while( nargs >= (sizeof(*h) + sizeof(*d))/2 )
		{
			d = (ReplaceMaskedBitsData*)(h + 1);

			nargs -= sizeof(*h)/2;

			if( h->num * sizeof(*d)/2 > nargs )
				// bug
				break;

			if( h->reg_addr_start >= DeviceMemoryModbusLength )
				// bug
				break;

			if( h->reg_addr_start + h->num > DeviceMemoryModbusLength )
				// bug
				break;

			uint16_t* dst = ((uint16_t*)&g_modbus_mem) + h->reg_addr_start;
			for( int n = h->num; n; --n, ++d, ++dst )
			{
				uint16_t r = *dst;
				r &= ~(d->mask & ~d->data);
				r |= (d->mask & d->data);
				*dst = r;
			}

			h = (ReplaceMaskedBitsHeader*)d;
		}
		break;
	}

	default:
		*nwords = 0;
		return;
	}

	g_modbus_mem.long_response = req;
}

static void handle_requests(void)
{
	if( g_modbus_mem.short_request )
		handle_short_request();

	if( g_modbus_mem.long_request )
		handle_long_request();

	if( g_modbus_mem.dev_ctl.debug_int_relay_to_led )
	{
		// Отладочный вывод состояния реле в светодиоды
		g_modbus_mem.dev_ctl.led_red = testU16Bit(g_modbus_mem.reserved_relay_out, 0);
		g_modbus_mem.dev_ctl.led_green = testU16Bit(g_modbus_mem.reserved_relay_out, 1);
		g_modbus_mem.dev_ctl.led_blue = testU16Bit(g_modbus_mem.reserved_relay_out, 2);
		//g_modbus_mem.dev_ctl.led_orange = testU16Bit(g_modbus_mem.reserved_relay_out, 3);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		uint32_t curr_time = HAL_GetTick();
		if( g_adc_update )
		{
			g_adc_update = 0;
			const int ns = g_adc_num_samples;

			for( int ch = 0; ch < adc_num_channels; ++ch )
			{
				uint32_t avg_prev = g_adc_avg_values[ch];
				uint32_t sko_prev = g_adc_sko_values[ch];
				uint32_t avg = 0;
				float sko = 0;
				uint32_t value = 0;
				int n = 0;
				for( int i = 0; i < ns; ++i )
				{
					uint16_t v = g_adc_samples[i][ch];
					avg += v;
					float d = (float)v - (float)avg_prev;
					sko += d*d;
					if( abs( d ) <= 3*sko_prev )
					{
						value += v;
						++n;
					}
				}
				avg /= ns;
				sko = sqrt(sko);
				sko /= ns;
				sko = ceil(sko);
				g_adc_avg_values[ch] = avg;
				g_adc_sko_values[ch] = sko;

				if( n )
				{
					value <<= 4;
					value /= n;
					g_adc_values[ch] = value;
				}
				taskYIELD();
			}

			float v_ref = (((uint32_t)g_adc_values[0]) * VREF)*0.001f/65535.0f;
			float t_cpu = (((((float)g_adc_values[1]) * VREF)/65535) - VSENS_AT_AMBIENT_TEMP) * 10 / ((float)AVG_SLOPE) + AMBIENT_TEMP;

			// сопротивления термометров
			float r[4];
			for( int i = 0; i < 4 ; ++i )
			{
				float a = (float)(g_adc_values[2+i]) / 65535.0f;
				r[i] = g_r1[i] * a / ( 1.0f - a );
			}

			taskYIELD();

			float t[4];
			for( int i = 0; i < 2; ++i )
				t[i] = calc_temperature_ntc10k(r[i]);

			for( int i = 0; i < 2; ++i )
				t[i+2] = calc_temperature_pt1000(r[i+2]);

			taskYIELD();

			setU16Bit(g_modbus_mem.int_sens_status, isid_vref, 1);
			g_modbus_mem.int_sens_value[isid_vref] = lround(v_ref*1000);

			setU16Bit(g_modbus_mem.int_sens_status, isid_tcpu, 1);
			g_modbus_mem.int_sens_value[isid_tcpu] = lround(t_cpu*10);

		  for( int i = 0; i < 4; ++i )
			{
				int idx = isid_t1 + i;
				bool is_ok = g_adc_values[i+2] != 0xffff;
				setU16Bit(g_modbus_mem.int_sens_status, idx, is_ok);

				if( is_ok )
					g_modbus_mem.int_sens_value[idx] = lround(t[i]*10);
			}
		}

		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, !g_modbus_mem.dev_ctl.led_red);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, !g_modbus_mem.dev_ctl.led_green);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, !g_modbus_mem.dev_ctl.led_blue);
		osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskUSB */
/**
* @brief Function implementing the myTaskUSB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUSB */
void StartTaskUSB(void *argument)
{
  /* USER CODE BEGIN StartTaskUSB */
  /* Infinite loop */
  for(;;)
  {
	xSemaphoreTake(g_modbus_usb.ModBusSphrHandle , portMAX_DELAY);
	xSemaphoreGive(g_modbus_usb.ModBusSphrHandle);
	handle_requests();
	osDelay(10);
  }
  /* USER CODE END StartTaskUSB */
}

/* USER CODE BEGIN Header_StartTaskRS485 */
/**
* @brief Function implementing the myTaskRS485 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskRS485 */
void StartTaskRS485(void *argument)
{
  /* USER CODE BEGIN StartTaskRS485 */
  /* Infinite loop */
  for(;;)
  {
	xSemaphoreTake(g_modbus_rs485.ModBusSphrHandle , portMAX_DELAY);
	xSemaphoreGive(g_modbus_rs485.ModBusSphrHandle);
	handle_requests();
    osDelay(10);
  }
  /* USER CODE END StartTaskRS485 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
