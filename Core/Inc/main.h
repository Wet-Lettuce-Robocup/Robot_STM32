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
	uint16_t prevTime;
	int prevCount;

	bool invert;
	float alpha;

	uint16_t dt;
	int dc;
} Encoder;

typedef struct {
	double k_p;
	double k_i;
	double k_d;
	double maxIntegral;

	double errorIntegral;
	double prevError;
	uint16_t prevTime;

	double d_t;

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

	bool invert;

	uint16_t speed;
	bool reversed;
	int maxPWM;

	int targetSpeed;
	bool pidActive;
} Motor;

typedef struct {
	TIM_HandleTypeDef *pwmTimer;
	uint8_t pwmChannel;
} Servo;

typedef struct {
	GPIO_TypeDef *trigPeripheral;
	uint16_t trigPin;
	TIM_HandleTypeDef *echoTimer;
	uint8_t echoChannel;

	double currentDistance; // mm
	bool isFirstCaptured;
	uint32_t icVal1;
	uint32_t icVal2;
	uint32_t lastUsed;
	uint32_t delayTime;
} UltraS;

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

#define __PID_INIT_DEFAULT(controller, clock, encoder) PID_Init(controller, clock, encoder, 0.0, 0.1, 0.0, 8000)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void Encoder_Init(Encoder *encoder, TIM_HandleTypeDef *clock, TIM_HandleTypeDef *htim, bool invert, float alpha);
void PID_Init(PID_Controller *controller, TIM_HandleTypeDef *clock, Encoder *encoder,
		double k_p, double k_i, double k_d, double maxIntegral);
void Motor_Init(Motor *motor, TIM_HandleTypeDef *clock, TIM_HandleTypeDef *htim, TIM_HandleTypeDef *pwmTimer,
		uint8_t pwmChannel, GPIO_TypeDef *dirGPIOPeripheral, uint16_t dirGPIOPin,
		GPIO_TypeDef *faultPeripheral, uint16_t faultPin, bool invert, bool invertEncoder, float alpha);
void Servo_Init(Servo *servo, TIM_HandleTypeDef *pwmTimer, uint8_t pwmChannel);
void UltraS_Init(UltraS *ultrasonic, GPIO_TypeDef *trigPeripheral,uint16_t trigPin,
		TIM_HandleTypeDef *echoTimer, uint8_t echoChannel, uint32_t delayTime);

void Motor_Drive(Motor *motor, int speed);
void Motor_DrivePID(Motor *motor, int speed);
void Motor_Stop(Motor *motor);
bool Motor_CheckFault(Motor *motor);

void Robot_Drive(Robot *robot, int speed, int strafe, int turn);
void Robot_Stop(Robot *robot);

void Servo_SetAngle(Servo *servo, int angle);
void Servo_Drive(Servo *servo, int8_t dir);

void UltraS_SendPulse(UltraS *ultrasonic);

void Encoder_Update(Encoder *encoder);
int PID_Update(PID_Controller *controller, int error);
void PID_Reset(PID_Controller *controller);
void Motor_Update(Motor *motor);
void Robot_Update(Robot *robot);

float Read_Internal_Temp();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define CMD_DRIVE       0x01  // Read 12 bytes
#define CMD_STOP        0x02  // Read 0 bytes
#define CMD_SET_SERVO   0x10  // Read 2 bytes
#define CMD_DRIVE_SERVO 0x11  // Read 2 bytes
							  // Direction (byte 2): 0 = stop, 1 = forward, 2 = backward

#define CMD_READ_STATUS 0x80
#define CMD_READ_VEL    0x81
#define CMD_READ_ENC    0x82
#define CMD_READ_ULTRAS 0x83
#define CMD_READ_TEMP   0x84

#define SERVO_COUNT 	3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
