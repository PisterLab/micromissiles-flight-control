/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  REGISTER_SENSOR_CONFIG0 = 0x03, // These five are from Titan
  REGISTER_FIFO_CONFIG = 0x16,
  REGISTER_FIFO_COUNTH = 0x2E,
  REGISTER_FIFO_COUNTL = 0x2F,
  REGISTER_FIFO_DATA = 0x30,

  REGISTER_IMU_READINGS = 0x1D, // Starts at TEMP_DATA1, read the next 16 bytes to 0x2C (TMST_FSYNCL)
//
//  REGISTER_TEMP_DATA1 = 0x1D,
//  REGISTER_TEMP_DATA0 = 0x1E,
//  REGISTER_ACCEL_DATA_X1 = 0x1F, // Accel data from 0x1F to 0x24 (X->Y->Z), gyro data from 0x25 to 0x2A (X->Y->Z)
//  	  	  	  	  	  	  	  	 // 0x1F contains bits [15:8], 0x20 contains bits [7:0]
//  REGISTER_TMST_FSYNCH = 0x2B,
//  REGISTER_TMST_FSYNCL = 0x2C,

  REGISTER_PWR_MGMT0 = 0x4E,
  REGISTER_GYRO_CONFIG0 = 0x4F,
  REGISTER_ACCEL_CONFIG0 = 0x50,

  REGISTER_FIFO_CONFIG1 = 0x5F, // These four are from Titan
  REGISTER_FIFO_CONFIG2 = 0x60,
  REGISTER_FIFO_CONFIG3 = 0x61,
  REGISTER_WHO_AM_I = 0x75,

  REGISTER_BANK_SEL = 0x76, // 000 for 0, 001 for 1, 010 for 2, 011 for 3, and 100 for 4
} imu_register_e;


typedef struct {
  uint8_t reserved;
  int8_t temperature_1;
  int8_t temperature_0;
  int8_t accel_x1;
  int8_t accel_x0;
  int8_t accel_y1;
  int8_t accel_y0;
  int8_t accel_z1;
  int8_t accel_z0;
  int8_t gyro_x1;
  int8_t gyro_x0;
  int8_t gyro_y1;
  int8_t gyro_y0;
  int8_t gyro_z1;
  int8_t gyro_z0;
  int8_t timestamp_1;
  int8_t timestamp_0;
} imu_packet_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPI_TIMEOUT 10
#define SPI_MAX_PACKET_SIZE 64
#define IMU_PACKET_SIZE 16 // 2B temperature, 6B X->Y->Z accel, 6B X->Y->Z gyro, 2B timestamp
#define IMU_SPI_PACKET_SIZE (IMU_PACKET_SIZE + 1)
#define IMU_DEFAULT_WHO_AM_I 0x42
//#define IMU_FLASH_SIZE (126 * 1024)
#define ACCEL_THRESHOLD_RAW 3000
#define GYRO_THRESHOLD_RAW 90
#define GRAV_HYSTERESIS 2000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t g_spi_tx_buffer[SPI_MAX_PACKET_SIZE];
uint8_t g_spi_rx_buffer[SPI_MAX_PACKET_SIZE];
//uint8_t g_imu_buffer[IMU_PACKET_SIZE];

//uint8_t g_imu_flash_buffer[IMU_FLASH_SIZE] __attribute__((section(".imu")));
//size_t g_imu_flash_num_bytes_written = 0;
uint32_t current_write_addr = 0x08070000;
const uint32_t FLASH_END_ADDR = 0x08080000;
uint32_t last_write_tick = 0;

uint32_t last_blink_tick = 0;
bool led_state = false;

bool magnet_on = false;
bool prev_magnet_on = false;

volatile int16_t current_accel_x = 0, current_accel_y = 0, current_accel_z = 0;
volatile int16_t current_gyro_x = 0, current_gyro_y = 0, current_gyro_z = 0;

volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
const float beta = 0.1;

volatile float current_roll = 0.0f;
volatile float roll_error = 0.0f;
//float target_pitch = 0.0f;
float target_roll = 0.0f;
bool has_target_orientation = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

static void red_led_on(void) {
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin, GPIO_PIN_SET);
}
static void red_led_off(void) {
  HAL_GPIO_WritePin(GPIOA, RED_LED_Pin, GPIO_PIN_RESET);
}


static void bright_led_on(void) {
  HAL_GPIO_WritePin(GPIOA, BRIGHT_LED_Pin, GPIO_PIN_SET);
}
static void bright_led_off(void) {
  HAL_GPIO_WritePin(GPIOA, BRIGHT_LED_Pin, GPIO_PIN_RESET);
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void cs_select(void) {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}
static void cs_deselect(void) {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}


static void spi_write(imu_register_e address, uint8_t data) {
  cs_select();
  g_spi_tx_buffer[0] = address & 0x7F;
  g_spi_tx_buffer[1] = data;
  HAL_SPI_Transmit(&hspi1, g_spi_tx_buffer, 2, SPI_TIMEOUT);
  cs_deselect();
}

static void spi_read(imu_register_e address, size_t length) {
  cs_select();
  g_spi_tx_buffer[0] = address | 0x80;
  HAL_SPI_TransmitReceive(&hspi1, g_spi_tx_buffer, g_spi_rx_buffer, length, SPI_TIMEOUT);
  cs_deselect();
}

static void imu_init(void) {
  spi_write(REGISTER_BANK_SEL, 0x00);
  HAL_Delay(50);
//  // Enable stream-to-FIFO mode.
//  spi_write(REGISTER_FIFO_CONFIG, 1 << 6);
//  // Enable temperature, gyroscope, and accelomerater data.
//  spi_write(REGISTER_FIFO_CONFIG1, 0x07);
  spi_write(REGISTER_PWR_MGMT0, 0x0F);
  HAL_Delay(50);
  spi_write(REGISTER_GYRO_CONFIG0, 0x44); // 500 degrees full scale, 4kHz ODR
  HAL_Delay(50);
  spi_write(REGISTER_ACCEL_CONFIG0, 0x24); // 8 g's full scale, 4kHz ODR
  HAL_Delay(10);
}

static void imu_verify(void) {
  spi_read(REGISTER_WHO_AM_I, /*length=*/2);
  if (g_spi_rx_buffer[1] == IMU_DEFAULT_WHO_AM_I) {
      red_led_on();
      HAL_Delay(1000);
      red_led_off();
  } else {
      red_led_on();
      HAL_Delay(3000);
      red_led_off();
  }
}

void save_to_flash(uint8_t *pData, uint32_t flash_address) {
  HAL_FLASH_Unlock();

//  // Loop 1
//  uint64_t data_packed = 0;
//  memcpy(&data_packed, pData, 8);
//  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_address, data_packed) != HAL_OK) {
//	  return;
//  }
//
//  // Loop 2
//  uint64_t data_packed = 0;
//    memcpy(&data_packed, pData+8, 8);
////    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_address+8, data_packed) != HAL_OK) {
////  	  break;
////    }

  for (int i = 0; i < 2; i++)
      {
          uint64_t data_packed = 0;

          // Safe copying: We use memcpy to move 8 bytes from the byte array
          // into a 64-bit variable. This avoids memory alignment crashes
          // that can happen if you just cast pointers.
          memcpy(&data_packed, (pData + (i * 8)), 8);

          // 3. Program the Double Word (64-bit)
          // Note: We increment the address by 8 bytes for the second loop iteration
          if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_address + (i * 8), data_packed) != HAL_OK)
          {
              // Optional: Handle Error (e.g., turn on Error LED)
              // Error_Handler();
              break;
          }
      }

  HAL_FLASH_Lock();
}

void TIM1_UP_TIM16_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim1);
}

//void temperature_check(int16_t detected_temperature) {
//	int16_t celsius = 25 + (detected_temperature / 132.48);
//	if (celsius > 1) { // Another option is celsius < 100 (both should obviously return true)
//		red_led_on();
//		HAL_Delay(1000);
//		red_led_off();
//	} else {
//		red_led_on();
//		HAL_Delay(1000);
//		red_led_off();
//	}
//}


//void accel_visualization(int16_t accelX, int16_t accelY, int16_t accelZ) {
//	if (abs(accelX) > ACCEL_THRESHOLD_RAW) {
//    	red_led_on();
//    } else {
//    	led_1_off();
//    }
//
//
//    if (abs(accelY) > ACCEL_THRESHOLD_RAW) {
//    	red_led_on();
//    } else {
//    	led_2_off();
//    }
//
//
//    if (abs(accelZ) > ACCEL_THRESHOLD_RAW) {
//    	red_led_on();
//    } else {
//    	red_led_off();
//    }
//}


//void gravity_visualization(int16_t accelX, int16_t accelY, int16_t accelZ) {
//	if ((fabs(accelX) > (fabs(accelY) + 2000)) && (fabs(accelX) > (fabs(accelZ) + 2000))) {
//    	led_1_on();
//    	led_2_off();
//    	led_3_off();
//    } else if ((fabs(accelY) > (fabs(accelX) + 2000)) && (fabs(accelY) > (fabs(accelZ) + 2000))) {
//    	led_1_off();
//    	led_2_on();
//    	led_3_off();
//    } else if ((fabs(accelZ) > (fabs(accelX) + 2000)) && (fabs(accelZ) > (fabs(accelY) + 2000))) {
//    	led_1_off();
//    	led_2_off();
//    	led_3_on();
//    }
//}


//void gyro_visualization(int16_t gyroX, int16_t gyroY, int16_t gyroZ) {
//
//	if (abs(gyroX) > GYRO_THRESHOLD_RAW) {
//    	led_1_on();
//    } else {
//    	led_1_off();
//    }
//
//
//    if (abs(gyroY) > GYRO_THRESHOLD_RAW) {
//    	led_2_on();
//    } else {
//    	led_2_off();
//    }
//
//
//    if (abs(gyroZ) > GYRO_THRESHOLD_RAW) {
//    	led_3_on();
//    } else {
//    	led_3_off();
//    }
//}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {

    if (hspi1.State == HAL_SPI_STATE_READY) {
    	spi_read(REGISTER_IMU_READINGS, IMU_SPI_PACKET_SIZE);
    	imu_packet_t* packet = (imu_packet_t*)g_spi_rx_buffer;
    	int16_t temperature = ((int16_t)(packet->temperature_1) << 8) | packet->temperature_0;
    	int16_t accel_x = ((int16_t)(packet->accel_x1) << 8) | packet->accel_x0;
    	int16_t accel_y = ((int16_t)(packet->accel_y1) << 8) | packet->accel_y0;
    	int16_t accel_z = ((int16_t)(packet->accel_z1) << 8) | packet->accel_z0;
    	int16_t gyro_x = ((int16_t)(packet->gyro_x1) << 8) | packet->gyro_x0;
    	int16_t gyro_y = ((int16_t)(packet->gyro_y1) << 8) | packet->gyro_y0;
    	int16_t gyro_z = ((int16_t)(packet->gyro_z1) << 8) | packet->gyro_z0;
    	int16_t timestamp = ((int16_t)(packet->timestamp_1) << 8) | packet->timestamp_0;

    	current_accel_x = accel_x;
    	current_accel_y = accel_y;
    	current_accel_z = accel_z;

    	float gx = (float)gyro_x * 0.015258f;
    	float gy = (float)gyro_y * 0.015258f;
    	float gz = (float)gyro_z * 0.015258f;

    	float gx_rot = (gx - gy) * 0.70710678f;
    	float gy_rot = (gx + gy) * 0.70710678f;

    	float ax = (float)accel_x;
    	float ay = (float)accel_y;
    	float az = (float)accel_z;
    	float ax_rot = (ax - ay) * 0.70710678f;
    	float ay_rot = (ax + ay) * 0.70710678f;

    	MadgwickAHRSupdateIMU(gx_rot, gy_rot, gz, ax_rot, ay_rot, az, 0.001f);
    	current_roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;

    	if (has_target_orientation) {
    		roll_error = current_roll - target_roll;
    		if (roll_error > 180.0f) roll_error -= 360.0f;
    		if (roll_error < -180.0f) roll_error += 360.0f;
    	} else {
    		roll_error = 0.0f;
    	}

    	// temperature_check(temperature);
    	// accel_visualization(accel_x, accel_y, accel_z);
        // gyro_visualization(gyro_x, gyro_y, gyro_z);
    	// gravity_visualization(accel_x, accel_y, accel_z);

//    	if (HAL_GPIO_ReadPin(USART1_RX_GPIO_Port, USART1_RX_Pin) == GPIO_PIN_SET) {
//    		magnet_on = true;
//    	}
//
//    	if (HAL_GPIO_ReadPin(USART1_RX_GPIO_Port, USART1_RX_Pin) == GPIO_PIN_SET) {
//    		magnet_on = false;
//    	}
//    	if (magnet_on) {
//    		if (HAL_GetTick() - last_write_tick >= 10) {
//    			last_write_tick = HAL_GetTick();
//    			if (current_write_addr + 16 < FLASH_END_ADDR) {
//    				red_led_on();
//    				save_to_flash((uint8_t*)g_spi_rx_buffer, current_write_addr);
//    				current_write_addr += 16;
//    			} else { // Flash is full
//    				magnet_on = false;
//    			}
//    		}
//    	} else {
//    		red_led_off();
//    	}

    }
    //led_activity_toggle();
  }
}

static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  imu_init();
  HAL_Delay(2000);
  imu_verify();
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_GPIO_WritePin(GPIOB, ENABLE_DRIVERS_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 1. Read current state of the sensor
	        magnet_on = (HAL_GPIO_ReadPin(MAGNET_DETECT_GPIO_Port, MAGNET_DETECT_Pin) == GPIO_PIN_SET);

	        // 2. Detect RISING edge (False changing to True)
	        if (magnet_on && !prev_magnet_on) {
	            // Reset blink timer and turn LED on immediately
	            last_blink_tick = HAL_GetTick();
	            bright_led_on();
	            led_state = true;
	            target_roll = current_roll;
	            has_target_orientation = true;
	        }

	        // 3. Detect FALLING edge (True changing to False)
	        else if (!magnet_on && prev_magnet_on) {
	            // Ensure LED stays off when the magnet is removed
	            bright_led_off();
	            led_state = false;
//
//	            // Forget the orientation
//	            target_roll = 0.0f;
	            has_target_orientation = false;
	        }

	        // 4. Handle 1 Hz Blinking (100ms ON, 900ms OFF)
	        if (magnet_on) {
	        	if (roll_error > -15.0f && roll_error < 15.0f) {
	        		red_led_on();
	        	} else {
		        	red_led_off();
	        	}
	        	if (led_state) {
	        		if (HAL_GetTick() - last_blink_tick >= 100) {
	        			last_blink_tick = HAL_GetTick();
	        			bright_led_off();
	        			led_state = false;
	        		}
	        	} else {
	        		if (HAL_GetTick() - last_blink_tick >= 900) {
	        			last_blink_tick = HAL_GetTick();
	        			bright_led_on();
	        			led_state = true;
	        		}
	        	}
	        }

	        // 5. Save current state as previous for the next loop iteration
	        prev_magnet_on = magnet_on;

	        // --- Future Stepper Motor Correction Logic Goes Here ---
	        // if (has_target_orientation) {
	        //     // Compare current_accel_x/y/z to target_pitch/roll and step motors
	        // }


//	  HAL_GPIO_WritePin(GPIOA, DIR2_U1_Pin|DIR1_U2_Pin|DIR2_U2_Pin, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOB, DIR1_U1_Pin, GPIO_PIN_SET);
//	  HAL_Delay(100);
//	  for (int i=0; i<8; i++) {
//		  HAL_GPIO_WritePin(GPIOA, STEP2_U1_Pin|STEP2_U2_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOB, STEP1_U1_Pin|STEP1_U2_Pin, GPIO_PIN_SET);
//		  HAL_Delay(125);
//		  HAL_GPIO_WritePin(GPIOA, STEP2_U1_Pin|STEP2_U2_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOB, STEP1_U1_Pin|STEP1_U2_Pin, GPIO_PIN_RESET);
//		  HAL_Delay(125);
//	  }
//
//	  HAL_GPIO_WritePin(GPIOA, DIR2_U1_Pin|DIR1_U2_Pin|DIR2_U2_Pin, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOB, DIR1_U1_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(100);
//
//	  for (int i=0; i<8; i++) {
//		  HAL_GPIO_WritePin(GPIOA, STEP2_U1_Pin|STEP2_U2_Pin, GPIO_PIN_SET);
//		  HAL_GPIO_WritePin(GPIOB, STEP1_U1_Pin|STEP1_U2_Pin, GPIO_PIN_SET);
//		  HAL_Delay(125);
//		  HAL_GPIO_WritePin(GPIOA, STEP2_U1_Pin|STEP2_U2_Pin, GPIO_PIN_RESET);
//		  HAL_GPIO_WritePin(GPIOB, STEP1_U1_Pin|STEP1_U2_Pin, GPIO_PIN_RESET);
//		  HAL_Delay(125);
//	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 127;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMAMUX_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX_OVR_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MODE_1X_Pin|STEP2_U1_Pin|DIR2_U2_Pin|DIR1_U2_Pin
                          |STEP2_U2_Pin|DIR2_U1_Pin|USART1_TX_Pin|RED_LED_Pin
                          |BRIGHT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR1_U1_Pin|STEP1_U1_Pin|STEP1_U2_Pin|MODE_0X_Pin
                          |ENABLE_DRIVERS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : VIN_SENSE_Pin */
  GPIO_InitStruct.Pin = VIN_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VIN_SENSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRST_Pin */
  GPIO_InitStruct.Pin = NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FAULT1_U1_Pin FAULT2_U1_Pin */
  GPIO_InitStruct.Pin = FAULT1_U1_Pin|FAULT2_U1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_1X_Pin STEP2_U1_Pin DIR2_U2_Pin DIR1_U2_Pin
                           STEP2_U2_Pin DIR2_U1_Pin USART1_TX_Pin RED_LED_Pin
                           BRIGHT_LED_Pin */
  GPIO_InitStruct.Pin = MODE_1X_Pin|STEP2_U1_Pin|DIR2_U2_Pin|DIR1_U2_Pin
                          |STEP2_U2_Pin|DIR2_U1_Pin|USART1_TX_Pin|RED_LED_Pin
                          |BRIGHT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR1_U1_Pin STEP1_U1_Pin STEP1_U2_Pin MODE_0X_Pin */
  GPIO_InitStruct.Pin = DIR1_U1_Pin|STEP1_U1_Pin|STEP1_U2_Pin|MODE_0X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MAGNET_DETECT_Pin */
  GPIO_InitStruct.Pin = MAGNET_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MAGNET_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FAULT2_U2_Pin FAULT1_U2_Pin */
  GPIO_InitStruct.Pin = FAULT2_U2_Pin|FAULT1_U2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_DRIVERS_Pin */
  GPIO_InitStruct.Pin = ENABLE_DRIVERS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE_DRIVERS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USART1_RX_Pin */
  GPIO_InitStruct.Pin = USART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USART1_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
