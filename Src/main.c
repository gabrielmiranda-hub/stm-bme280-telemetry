/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "lora.h"
#include "lapesp-types.h"
#include "lapesp-protocol.h"
#include "sensor-data.h"
#include <string.h>
#include <stdio.h>
#include "BNO080.h"
#include "BME280_STM32.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t id;
    double temperatura;
    double umidade;
    double pressao;
    double altitude;
} BMEstruct;
typedef struct {
    uint8_t id;
    float gyrq0;
    float gyrq1;
    float gyrq2;
    float gyrreal;

    float gyrx;
	float gyry;
	float gyrz;

    float accelx;
    float accely;
    float accelz;

    float magx;
    float magy;
    float magz;

} BNOstruct;

typedef enum {
    MSG_BME,
    MSG_BNO,
    MSG_ADXL
} MsgType;

typedef struct {
    MsgType type;

    union {
        BMEstruct bme;
        BNOstruct bno;
    } data;

} SensorMessage;


Sensores_Data_t sensor_data;
BME_Data_t bme_data;
BNO085_Data_t bno_data;
GPS_Data_t gps_data;
BME280_Data_t bme280;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME_READY  (1U << 0)
#define BNO_READY  (1U << 1)
#define GPS	_READY (1U << 2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for BME280 */
osThreadId_t BME280Handle;
const osThreadAttr_t BME280_attributes = {
  .name = "BME280",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BNO08X */
osThreadId_t BNO08XHandle;
const osThreadAttr_t BNO08X_attributes = {
  .name = "BNO08X",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GPS */
osThreadId_t GPSHandle;
const osThreadAttr_t GPS_attributes = {
  .name = "GPS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Enviadados */
osThreadId_t EnviadadosHandle;
const osThreadAttr_t Enviadados_attributes = {
  .name = "Enviadados",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
void StartBME(void *argument);
void StartBNO08x(void *argument);
void StartGPS(void *argument);
void StartDadosTask(void *argument);

/* USER CODE BEGIN PFP */
void Task_Blink(void *arg);
void Task_BME(void *arg);
void Task4(void *arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
void Sensor_Init(void)
{
  //Init structure definition section
		BME280_Init_t BME280_InitStruct = {0};
	    Reset_BME280();
	    BME280_InitStruct.Filter = FILTER_8;
	    BME280_InitStruct.Mode = BME280_NORMAL_MODE;
	    BME280_InitStruct.OverSampling_H = OVERSAMPLING_16;
	    BME280_InitStruct.OverSampling_P = OVERSAMPLING_16;
	    BME280_InitStruct.OverSampling_T = OVERSAMPLING_16;
	    BME280_InitStruct.SPI_EnOrDıs = SPI3_W_DISABLE;
	    BME280_InitStruct.T_StandBy = T_SB_250;
	    BME280Init(BME280_InitStruct);
}
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  	 Sensores_Data_Init(&sensor_data, &bme_data, &bno_data, &gps_data);
  	 Sensor_Init();
  	 BNO080_Initialization();
  	 BNO080_enableGyro(2500);
  	 BNO080_enableRotationVector(2500); //enable rotation vector at 400Hz
     BNO080_enableAccelerometer(2500);
     BNO080_enableMagnetometer(2500);
     BNO080_calibrateAll();
     HAL_Delay(200);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (10, sizeof(SensorMessage), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BME280 */
  BME280Handle = osThreadNew(StartBME, (void*) &bme280, &BME280_attributes);

  /* creation of BNO08X */
  BNO08XHandle = osThreadNew(StartBNO08x, NULL, &BNO08X_attributes);


  /* creation of GPS */
  GPSHandle = osThreadNew(StartGPS, NULL, &GPS_attributes);

  /* creation of Enviadados */
  EnviadadosHandle = osThreadNew(StartDadosTask, NULL, &Enviadados_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

   osThreadNew(StartBME, &bme280, &BME280_attributes);
   osThreadNew(StartBNO08x, NULL, &BNO08X_attributes);
   osThreadNew(StartDadosTask, NULL, &Enviadados_attributes);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add 's, ... */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**SPI2 GPIO Configuration
  PB13   ------> SPI2_SCK
  PB14   ------> SPI2_MISO
  PB15   ------> SPI2_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI2, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI2, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */

/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_12);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8|LL_GPIO_PIN_11|LL_GPIO_PIN_12);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_11|LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBME */
/**
  * @brief  Function implementing the BME280 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBME */
void StartBME(void *argument)
{
  /* USER CODE BEGIN 5 */
	SensorMessage msg;
	/* Infinite loop */
    for(;;)
	{
    	BME280Calculation(&bme280);
    	msg.data.bme.id = 0x01;
		msg.type = MSG_BME;
		msg.data.bme.temperatura = bme280.Temperature;
		msg.data.bme.pressao = bme280.Pressure;
		msg.data.bme.altitude = bme280.AltitudeTP;
		msg.data.bme.umidade= bme280.Humidity;
		osMessageQueuePut(myQueue01Handle, &msg, 0, 0);
		osDelay(200);
	}
}
  /* USER CODE END 5 */


/* USER CODE BEGIN Header_StartBNO08x */
/**
* @brief Function implementing the BNO08X thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBNO08x */
void StartBNO08x(void *argument)
{
  /* USER CODE BEGIN StartBNO08x */
  /* Infinite loop */
	SensorMessage msg;
	for(;;)
  {
	if(BNO080_dataAvailable()){
			msg.type = MSG_BNO;
			msg.data.bno.id = 0x02;

			msg.data.bno.gyrq0 = BNO080_getQuatI();
			msg.data.bno.gyrq1 = BNO080_getQuatJ();
			msg.data.bno.gyrq2 = BNO080_getQuatK();
			msg.data.bno.gyrreal = BNO080_getQuatReal();

			msg.data.bno.gyrx = BNO080_getGyroX();
			msg.data.bno.gyry = BNO080_getGyroY();
			msg.data.bno.gyrz = BNO080_getGyroZ();

			msg.data.bno.accelx = BNO080_getAccelX();
			msg.data.bno.accely = BNO080_getAccelY();
			msg.data.bno.accelz = BNO080_getAccelZ();

			msg.data.bno.magx = BNO080_getMagX();
			msg.data.bno.magy = BNO080_getMagY();
			msg.data.bno.magz = BNO080_getMagZ();
		osMessageQueuePut(myQueue01Handle, &msg, 0, 0);
    	osDelay(200);

		}
  }

  /* USER CODE END StartBNO08x */
}

/* USER CODE BEGIN Header_StartGPS */
/**
* @brief Function implementing the GPS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPS */
void StartGPS(void *argument)
{
  /* USER CODE BEGIN StartGPS */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGPS */
}

/* USER CODE BEGIN Header_StartDadosTask */
void StartDadosTask(void *argument)
{
	SensorMessage msg;
    for (;;)
    {
        if ((osMessageQueueGet(myQueue01Handle, &msg, 0, osWaitForever)== osOK))
        {
        	if(msg.type == MSG_BME){
        		printf("Temperatura: %.2f\r\n", msg.data.bme.temperatura);
				printf("Pressao: %.2f\r\n", msg.data.bme.pressao);
				printf("Altitude: %.2f\r\n", msg.data.bme.altitude);
				printf("Umidade: %.2f\r\n", msg.data.bme.umidade);
        	}

        	else if(msg.type == MSG_BNO){
        		printf("ACC: %.2f %.2f %.2f\r\n", msg.data.bno.accelx,msg.data.bno.accely , msg.data.bno.accelz);
        		printf("QUAT: %.2f %.2f %.2f %.2f\r\n", msg.data.bno.gyrq0, msg.data.bno.gyrq1, msg.data.bno.gyrq2, msg.data.bno.gyrreal);
        		printf("MAG: %.2f %.2f %.2f\r\n", msg.data.bno.magx, msg.data.bno.magy, msg.data.bno.magz);
        		printf("MAG: %.2f %.2f %.2f\r\n", msg.data.bno.gyrx, msg.data.bno.gyry, msg.data.bno.gyrz);
        	}

        }
    }
}
/**
* @brief Function implementing the Enviadados thread.
* @param argument: Not used
* @retval None
*
*/
/* USER CODE END Header_StartDadosTask */

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
