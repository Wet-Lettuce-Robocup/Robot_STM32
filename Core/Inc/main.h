/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct {
	TIM_HandleTypeDef *htim;
	TIM_HandleTypeDef *clock;

	int speed;
	uint32_t prevTime;
	int prevCount;
} Encoder;

typedef struct {
	float k_p;
	float k_i;
	float k_d;

	float errorIntegral;
	float prevError;
	uint32_t prevTime;

	Encoder *encoder;
	TIM_HandleTypeDef *clock;
} PID_Controller;

typedef struct {
	Encoder encoder;
	PID_Controller controller;
	TIM_HandleTypeDef *clock;

	TIM_HandleTypeDef *pwmTimer;
	uint8_t pwmChannel;
	GPIO_TypeDef *dirGPIOPeripheral;
	uint16_t dirGPIOPin;

	GPIO_TypeDef *faultPeripheral;
	uint16_t faultPin;

	int targetSpeed;
	bool pidActive;
} Motor;

typedef struct {
	TIM_HandleTypeDef *pwmTimer;
	uint8_t pwmChannel;
} Servo;

typedef enum {
	STATE_STOPPED,
	STATE_DRIVING,
	STATE_FAULT
} Robot_State;

typedef struct {
	Motor frontLeftMotor;
	Motor frontRightMotor;
	Motor backLeftMotor;
	Motor backRightMotor;

	GPIO_TypeDef *regEnablePeripheral;
	uint16_t regEnablePin;

	Robot_State state;
} Robot;

typedef enum {
    STATE_IDLE,
    STATE_WAIT_COMMAND,
    STATE_WAIT_DATA,
    STATE_SEND_RESPONSE
} I2C_State_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define __PID_INIT_DEFAULT(controller, clock, encoder) PID_Init(controller, clock, encoder, 1.0, 0.3, 0.01)

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void Encoder_Init(Encoder *encoder, TIM_HandleTypeDef *clock, TIM_HandleTypeDef *htim);
void PID_Init(PID_Controller *controller, TIM_HandleTypeDef *clock, Encoder *encoder,
		float k_p, float k_i, float k_d);
void Motor_Init(Motor *motor, TIM_HandleTypeDef *clock, TIM_HandleTypeDef *htim, TIM_HandleTypeDef *pwmTimer,
		uint8_t pwmChannel, GPIO_TypeDef *dirGPIOPeripheral, uint16_t dirGPIOPin,
		GPIO_TypeDef *faultPeripheral, uint16_t faultPin);
void Servo_Init(Servo *servo, TIM_HandleTypeDef *pwmTimer, uint8_t pwmChannel);

void Motor_Drive(Motor *motor, int speed);
void Motor_DrivePID(Motor *motor, int speed);
void Motor_Stop(Motor *motor);
bool Motor_CheckFault(Motor *motor);

void Robot_Drive(Robot *robot, int speed, int strafe, int turn);
void Robot_Stop(Robot *robot);

void Servo_SetAngle(Servo *servo, int angle);

void Encoder_Update(Encoder *encoder);
int PID_Update(PID_Controller *controller, int error);
void PID_Reset(PID_Controller *controller);
void Motor_Update(Motor *motor);
void Robot_Update(Robot *robot);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define CMD_DRIVE       0x01  // Read 12 bytes
#define CMD_STOP        0x02  // Read 0 bytes
#define CMD_SET_SERVO   0x10  // Read 2 bytes

#define CMD_READ_STATUS 0x80
#define CMD_READ_VEL    0x81
#define CMD_READ_ENC    0x82

#define SERVO_COUNT 3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
