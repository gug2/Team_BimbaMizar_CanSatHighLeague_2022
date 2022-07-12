/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "Databuf.h"
#include "MS5607SPI.h"
#include "LSM6DSLTR.h"
#include "LSM303DTR.h"
#include "STLM75M2F.h"
#include "gps.h"
#include "LoRa-RA.h"

#include "bmp280.h"
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
static FATFS fatfs;
static FIL file;
static UINT bytesWritten;

static uint8_t DEBUG_BUFFER[256];
static uint16_t DEBUG_BUFFER_SIZE = 0;

static LSM303DTR_Data_Typedef LSM303DTR_Data;
static LSM6DSLTR_Data_Typedef LSM6DSLTR_Data;
static lora_sx1276 LoRa;

FRESULT SD_mountStatus = -1;
FRESULT SD_openStatus = -1;
FRESULT SD_writeStatus = -1;
FRESULT SD_seekStatus = -1;
FRESULT SD_syncStatus = -1; // flush bytes to SD
FRESULT SD_closeStatus = -1;
MS5607StateTypeDef MS5607_status1 = MS5607_STATE_FAILED;
LSM6DSLTR_Status LSM6DSLTR_status2 = LSM6DSLTR_DEVICE_ERROR;
LSM303DTR_Status LSM303DTR_status3 = LSM303DTR_DEVICE_ERROR;
uint8_t LoRa_status4 = 1;
uint8_t LoRa_statusSend = 1;
uint32_t loopStartTick = 0;
uint32_t loopTime = 0;
uint32_t sysInitTime = 0;
uint32_t userInitTime = 0;

int16_t stlmTemperature = 0;
float stlmTemperatureFloat = 0.0F;
float ms5607_temperature_c = 0.0F;
int32_t ms5607_raw_temperature = 0;
int32_t ms5607_pressure_pa = 0;
float ms5607_altitude = 0.0F;
int32_t ms5607_altitude_i32 = 0;
int32_t ms5607_reference_pressure_pa = 0;

static GPS_Data_Typedef GPS_Data;


/* INA180A sensor */
/*#define INA180A_ADC_MAX 4095.0F
#define INA180A_ADC_REF 3.3F
#define INA180A_GAIN 200.0F
#define INA180A_SHUNT 0.01F
uint32_t ina180a_adc_value;
float ina180a_v, ina180a_a;*/
/* INA180A sensor */

#define R1 82000.0 // 82 kOm
#define R2 200000.0 // 200 kOm
uint32_t adc2_in13_value;
float adc2_in13_v;

uint32_t photoresist_adc_value;
uint8_t currentADCChannel = ADC_CHANNEL_10;

uint32_t startServoDelay = 0;
uint32_t startServoPWM = 0;
uint32_t startDelayChange = 0;

uint8_t isBuzzerWorking = 0;
uint8_t isTimerStarted = 0;

/** Thresholds start **/
uint32_t photoresist_threshold = 1200;
float ms5607_altitudeThreshold = 500.0F;
/** Thresholds end **/

BMP280_HandleTypedef bmp280;
uint8_t bmp280_status = 0;
float bmp280_reference_pressure = 0.0F;
float bmp280_pressure = 0.0F;
float bmp280_altitude = 0.0F;
int32_t bmp280_altitude_i32 = 0;

uint8_t syncSDLoopIndex = 0;
uint8_t isSDFileClosed = 0;
uint8_t canLoRaSend = 1;

/** Test sd card using HAL
 * uint8_t readBuf[512];
 * uint8_t writeBuf[512] = "test string";
 * SD_HandleTypeDef hsd;
 * HAL_StatusTypeDef initS = -1, writeS = -1, readS = -1;
**/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART RX callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	GPS_Callback(huart, &GPS_Data);
}

// ADC Conversion Callback
/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	if(hadc->Instance == ADC2) {
		// get adc value
	}
}*/

// Buzzer update Callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if(htim->Instance == TIM1) {
		if(isTimerStarted > 0) {
			if(isBuzzerWorking > 0) {
				isBuzzerWorking = 0;
			} else {
				isBuzzerWorking = 1;
			}
		}
	}
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
  uint32_t sysInitStartTick = HAL_GetTick();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  sysInitTime = HAL_GetTick() - sysInitStartTick;
  uint32_t userInitStartTick = HAL_GetTick();
  /*
   * -- SPI CHIP SELECTS --
   * PB5 - pressure sensor - MS5607
   * PC1 - accelerometer/gyroscope module - LSM6DSLTR
   * PC2 - accelerometer/magnetometer module - LSM303DTR
   * PB8 - radio module - LORA-RA-02
  */
  /*
   * -- OSR'S -- | supply current (1 sample per sec.) | conversion time | pressure resolution | temperature resolution |
   * 	 256	 |				0.9 uA				  |  0.6 ms (max)   |	  0.130 mbar	  |			0.012 C		   |
   * 	 512	 |				1.7 uA				  |  1.17 ms (max)  |	  0.084 mbar	  |			0.008 C 	   |
   * 	1024	 |				3.2 uA				  |  2.28 ms (max)  |	  0.054 mbar	  |			0.005 C 	   |
   * 	2048	 | 				6.3 uA				  |  4.54 ms (max)  |	  0.036 mbar	  |			0.003 C 	   |
   * 	4096	 |				12.5 uA				  |  9.04 ms (max)  |	  0.024 mbar 	  |			0.002 C 	   |
   */
  HAL_Delay(50);
  STLM75M2F_Init(&hi2c1);

  MS5607_SetPressureOSR(MS5607_OSR_256);
  MS5607_SetTemperatureOSR(MS5607_OSR_256);
  MS5607_status1 = MS5607_Init(&hspi1, GPIOB, GPIO_PIN_5);

  LSM6DSLTR_status2 = LSM6DSLTR_Init(&hspi1, GPIOC, GPIO_PIN_1);
  LSM6DSLTR_SetAccelerometerODR_FS(LSM6DSLTR_ODR_12_5HZ, LSM6DSLTR_FS_16G);
  LSM6DSLTR_SetGyroscopeODR_FS(LSM6DSLTR_ODR_12_5HZ, LSM6DSLTR_FS_1KDPS, LSM6DSLTR_SETTING_FS_125DPS_DISABLE);

  LSM303DTR_status3 = LSM303DTR_Init(&hspi1, GPIOC, GPIO_PIN_2);
  ////LSM303DTR_EnableThermometer();
  LSM303DTR_SetAccelerometerFS(LSM303DTR_FS_16G);
  LSM303DTR_SetAccelerometerODR(LSM303DTR_ODR_12_5HZ);
  LSM303DTR_SetCompassMode(LSM303DTR_CONTINUOUS_CONVERSION);
  LSM303DTR_SetCompassFS(LSM303DTR_FS_8GAUSS);
  LSM303DTR_SetCompassODR(LSM303DTR_ODR_12_5HZ);

  HAL_Delay(50);
  bmp280_init_default_params(&bmp280.params);
  bmp280.addr = BMP280_I2C_ADDRESS_0;
  bmp280.spi = &hspi1;
  bmp280.spiPort = GPIOB;
  bmp280.spiPin = GPIO_PIN_10;
  bmp280_status = bmp280_init(&bmp280, &bmp280.params);

  LoRa_status4 = lora_init(&LoRa, &hspi1, GPIOB, GPIO_PIN_8, 434000000);

  /** Test sd card using HAL
   * HAL_SD_MspInit(&hsd);
   * initS = HAL_SD_Init(&hsd);
  **/
  // mount the SD card BEGIN
  HAL_Delay(50);
  SD_mountStatus = f_mount(&fatfs, "", 1);
  // mount the SD card END
  userInitTime = HAL_GetTick() - userInitStartTick;

  // write init info to sd card
  SD_openStatus = f_open(&file, "data3.txt", FA_OPEN_ALWAYS | FA_WRITE);
  SD_seekStatus = f_lseek(&file, f_size(&file));
  __disable_irq();
  SD_writeStatus = f_printf(&file, "\nSystem init: %dms\n", sysInitTime);
  f_printf(&file, "User init: %dms\n", userInitTime);
  f_printf(&file, "MS5607 - %d\n", MS5607_status1);
  f_printf(&file, "LSM6DSLTR - %d\n", LSM6DSLTR_status2);
  f_printf(&file, "LSM303DTR - %d\n", LSM303DTR_status3);
  f_printf(&file, "LoRa-RA-02 - %d\n", LoRa_status4);
  f_printf(&file, "| T(ms), STLM75: temperature | MS5607: temperature, pressure | LSM6: T Gx Gy Gz Ax Ay Az | LSM3: T Ax Ay Az Mx My Mz |\n");
  f_printf(&file, "==================\n\n");
  __enable_irq();
  SD_syncStatus = f_sync(&file);

  // timer pwm
  ////setPWM(&htim2, TIM_CHANNEL_3, hzToPeriod(3000), hzToPeriod(3000)/2);
  setPWM(&htim3, TIM_CHANNEL_4, 1920, servoAngle(1920, 10));
  HAL_Delay(4000);

  // start GPS module
  GPS_Init(&huart4);

  // read reference pressure
  MS5607_Update();
  ms5607_reference_pressure_pa = MS5607_GetPressurePa();

  bmp280_read_float(&bmp280, NULL, &bmp280_reference_pressure, NULL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint32_t loopTime;
  while (1)
  {
	loopStartTick = HAL_GetTick();

	//setPWM(&htim3, TIM_CHANNEL_4, 1920, servoAngle(1920, 100));
	//HAL_Delay(4000);

	/** ina180a & photoresistor read adc start **/
	ADC_SetChannel(ADC_CHANNEL_3);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	photoresist_adc_value = HAL_ADC_GetValue(&hadc2);
	HAL_ADC_Stop(&hadc2);

	ADC_SetChannel(ADC_CHANNEL_13);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	/* INA180A sensor start */
	/*ina180a_adc_value = HAL_ADC_GetValue(&hadc2);
	ina180a_v = ina180a_adc_value / INA180A_ADC_MAX * INA180A_ADC_REF;
	ina180a_a = ina180a_v / (INA180A_SHUNT * INA180A_GAIN);*/
	/* INA180A sensor end */
	adc2_in13_value = HAL_ADC_GetValue(&hadc2);
	adc2_in13_v = adc2_in13_value * 3.3F / 4095.0F;
	adc2_in13_v = adc2_in13_v / (R2 / (R1+R2));
	HAL_ADC_Stop(&hadc2);

	// after altitude > threshold or time >= 5000
	// after get light from photoresist, 4 sec delay, 4 sec PWM enable
	if((startDelayChange == 0 || startServoDelay == 0) && (ms5607_altitude >= ms5607_altitudeThreshold || bmp280_altitude >= ms5607_altitudeThreshold) && photoresist_adc_value >= photoresist_threshold) {
		startServoDelay = loopStartTick;
		startDelayChange = loopStartTick;
	}

	if(startServoDelay != 0 && loopStartTick - startServoDelay >= 10000) { // увеличить до 10 сек
		if(startServoPWM == 0) {
			startServoPWM = loopStartTick;
		}
	}

	if(startServoPWM != 0) {
		if(loopStartTick - startServoPWM < 30000) {
			setPWM(&htim3, TIM_CHANNEL_4, 1920, servoAngle(1920, 100));
		} else {
			if(isTimerStarted == 0) {
				// start buzzer timer
				HAL_TIM_Base_Start_IT(&htim1);
				isTimerStarted = 1;
			}
		}
	}

	if(isBuzzerWorking > 0) {
		setPWM(&htim2, TIM_CHANNEL_3, hzToPeriod(1500), hzToPeriod(1500)/2);
	} else {
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
	}
	/** ina180a & photoresistor read adc end **/


	/** Test sd card using HAL
	 * writeS = HAL_SD_WriteBlocks(&hsd, writeBuf, 0x55, 1, 500);
	 * HAL_Delay(100);
	 * readS = HAL_SD_ReadBlocks(&hsd, readBuf, 0x55, 1, 500);
	**/


	////__disable_irq();
	//FRESULT createNew = f_open(&file, "data2.txt", FA_CREATE_NEW | FA_WRITE);
	//if(createNew == FR_EXIST) {
		/***SD_openStatus = f_open(&file, "data3.txt", FA_OPEN_ALWAYS | FA_WRITE);
	//}
	SD_seekStatus = f_lseek(&file, f_size(&file));
	uint16_t writeSize = sprintf(writeBuf, "Loop: %dms\r\n", loopStartTick);
	__disable_irq();
	SD_writeStatus = f_write(&file, writeBuf, writeSize, &bytesWritten);
	__enable_irq();
	SD_closeStatus = f_close(&file);***/

	// STLM TEST //
	////stlmTemp = STLM75M2F_GetTemperature();
	////float stlmFloatTemp = stlmTemp / 256.0F;
	////sprintf(writeBuf, "Temp: %.2f\r\n", stlmFloatTemp);
	// STLM TEST END //

	////LoRa_statusSend = lora_send_packet_blocking(&LoRa, writeBuf, writeSize, 1000);

	////__enable_irq();

	bmp280_read_float(&bmp280, NULL, &bmp280_pressure, NULL);
	bmp280_altitude = MS5607_Altitude(bmp280_reference_pressure, bmp280_pressure);
	bmp280_altitude_i32 = MS5607_Altitude_I32(bmp280_altitude);

	MS5607_Update();
	ms5607_raw_temperature = MS5607_GetRawTemperature();
	ms5607_temperature_c = (float)MS5607_GetTemperatureC();
	ms5607_pressure_pa = MS5607_GetPressurePa();
	ms5607_altitude = MS5607_Altitude(ms5607_reference_pressure_pa, ms5607_pressure_pa);
	ms5607_altitude_i32 = MS5607_Altitude_I32(ms5607_altitude);

	LSM303DTR_ReadData(&LSM303DTR_Data);
	LSM6DSLTR_ReadData(&LSM6DSLTR_Data);

	stlmTemperature = STLM75M2F_GetTemperature();
	stlmTemperatureFloat = stlmTemperature / 256.0F;


	Buf_clear();
	Buf_push8(0x24); // $
	Buf_push32(loopStartTick/250);
	Buf_push16(stlmTemperature);
	Buf_push32(ms5607_raw_temperature);
	Buf_push32(ms5607_altitude_i32);
	Buf_push32(bmp280_altitude_i32);
	Buf_push32(adc2_in13_value);
	Buf_push16((int16_t)photoresist_adc_value);

	Buf_push32(GPS_Data.latitude1);
	Buf_push32(GPS_Data.latitude2);
	Buf_push32(GPS_Data.longitude1);
	Buf_push32(GPS_Data.longitude2);

	Buf_push16(LSM6DSLTR_Data.gyroscope.x);
	Buf_push16(LSM6DSLTR_Data.gyroscope.y);
	Buf_push16(LSM6DSLTR_Data.gyroscope.z);
	Buf_push16(LSM6DSLTR_Data.accelerometer.x);
	Buf_push16(LSM6DSLTR_Data.accelerometer.y);
	Buf_push16(LSM6DSLTR_Data.accelerometer.z);

	//Buf_push16(LSM303DTR_Data.accelerometer.x);
	//Buf_push16(LSM303DTR_Data.accelerometer.y);
	//Buf_push16(LSM303DTR_Data.accelerometer.z);
	Buf_push16(LSM303DTR_Data.compass.x);
	Buf_push16(LSM303DTR_Data.compass.y);
	Buf_push16(LSM303DTR_Data.compass.z);
	Buf_push8(0x0A); // \n

	/***DEBUG_BUFFER_SIZE = sprintf(
		DEBUG_BUFFER,
		"%.3f, STLM75: %.2f | MS: %.2f %d | LSM6: %.2f %.2f %.2f %.2f %.2f %.2f %.2f | LSM3: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n",
		loopStartTick / 1000.0F,
		stlmTemperatureFloat,
		ms5607_temperature_c,
		ms5607_pressure_pa,
		int16_tToFloat(LSM6DSLTR_Data.temperature, 1),
		int16_tToFloat(LSM6DSLTR_Data.gyroscope.x, 1000),
		int16_tToFloat(LSM6DSLTR_Data.gyroscope.y, 1000),
		int16_tToFloat(LSM6DSLTR_Data.gyroscope.z, 1000),
		int16_tToFloat(LSM6DSLTR_Data.accelerometer.y, 16),
		int16_tToFloat(LSM6DSLTR_Data.accelerometer.x, 16),
		int16_tToFloat(LSM6DSLTR_Data.accelerometer.z, 16),
		int16_tToFloat(LSM303DTR_Data.temperature, 1),
		int16_tToFloat(LSM303DTR_Data.accelerometer.x, 16),
		int16_tToFloat(LSM303DTR_Data.accelerometer.y, 16),
		int16_tToFloat(LSM303DTR_Data.accelerometer.z, 16),
		int16_tToFloat(LSM303DTR_Data.compass.x, 8),
		int16_tToFloat(LSM303DTR_Data.compass.y, 8),
		int16_tToFloat(LSM303DTR_Data.compass.z, 8)
	);***/

	//LoRa_statusSend = lora_send_packet_blocking(&LoRa, Buf_getDatabuf(), Buf_getDatasize(), 1000);
	if(canLoRaSend > 0) {
		LoRa_statusSend = lora_send_packet(&LoRa, Buf_getDatabuf(), Buf_getDatasize());
		canLoRaSend = 0;
	} else {
		if(lora_wait_send(&LoRa) == LORA_OK) {
			canLoRaSend = 1;
		}
	}

	//
	/**__disable_irq();
	f_open(&file, "Log.txt", FA_OPEN_ALWAYS | FA_WRITE);
	f_lseek(&file, f_size(&file));
	f_write(&file, Buf_getDatabuf(), Buf_getDatasize(), &bytesWritten);
	**/

	DEBUG_BUFFER_SIZE = sprintf(
		DEBUG_BUFFER,
		"%.3f, STLM75: %.2f | MS: %.2f %d %.2f BMP: %.2f %.2f | %.2f %d GPS: %d.%d %d.%d | LSM6: %.2f %.2f %.2f %.2f %.2f %.2f | LSM3: %.2f %.2f %.2f\n",
		loopStartTick / 1000.0F,
		stlmTemperatureFloat,
		ms5607_temperature_c,
		ms5607_pressure_pa,
		ms5607_altitude,
		bmp280_pressure,
		bmp280_altitude,
		adc2_in13_v,
		photoresist_adc_value,
		GPS_Data.latitude1,
		GPS_Data.latitude2,
		GPS_Data.longitude1,
		GPS_Data.longitude2,

		int16_tToFloat(LSM6DSLTR_Data.gyroscope.x, 1000),
		int16_tToFloat(LSM6DSLTR_Data.gyroscope.y, 1000),
		int16_tToFloat(LSM6DSLTR_Data.gyroscope.z, 1000),
		int16_tToFloat(LSM6DSLTR_Data.accelerometer.x, 16),
		int16_tToFloat(LSM6DSLTR_Data.accelerometer.y, 16),
		int16_tToFloat(LSM6DSLTR_Data.accelerometer.z, 16),
		int16_tToFloat(LSM303DTR_Data.compass.x, 8),
		int16_tToFloat(LSM303DTR_Data.compass.y, 8),
		int16_tToFloat(LSM303DTR_Data.compass.z, 8)
	);
	if(!isSDFileClosed) {
		SD_seekStatus = f_lseek(&file, f_size(&file));
		__disable_irq();
		SD_writeStatus = f_write(&file, DEBUG_BUFFER, DEBUG_BUFFER_SIZE, &bytesWritten);
		if(syncSDLoopIndex >= 9) { // flush bytes to SD every 10 loops
			SD_syncStatus = f_sync(&file);
			syncSDLoopIndex = 0;
		}
		__enable_irq();
	}


	// toggle LED
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	syncSDLoopIndex++;
	//
	loopTime = HAL_GetTick() - loopStartTick;

	// DELAY CHANGE

	if(startDelayChange != 0 && loopStartTick - startDelayChange >= 180000) {
		if(loopTime < 1000) HAL_Delay(1000 - 1 - loopTime);
		if(!isSDFileClosed) {
			SD_closeStatus = f_close(&file);
			isSDFileClosed = 1;
		}
	} else {
		if(loopTime < 50) HAL_Delay(50 - 1 - loopTime);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
