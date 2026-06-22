/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 Wet Lettuce.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdlib.h>
#include <math.h>

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

volatile I2C_State_t i2c_state = STATE_IDLE;
volatile uint8_t command_byte = 0;
volatile uint8_t rx_buffer[32];
volatile uint8_t tx_buffer[32];
volatile uint8_t rx_length = 0;
volatile uint8_t tx_length = 0;
volatile uint8_t data_received = 0;

Robot robot;
Servo servos[SERVO_COUNT];
UltraS ultrasonic;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

uint8_t GetRxLengthForCommand(uint8_t cmd);
uint8_t GetTxLengthForCommand(uint8_t cmd);
uint8_t IsReadCommand(uint8_t cmd);
void InsertIntoBuffer(int data, uint8_t *buffer);
void ProcessReceivedData(uint8_t cmd, uint8_t *data, uint8_t len);
void PrepareResponseData(uint8_t cmd);

void delay(uint16_t time);
void setup();
void loop();

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  loop();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
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

/* USER CODE BEGIN 4 */

// ==========================================
// Initialization
// ==========================================


void Encoder_Init(Encoder *encoder, TIM_HandleTypeDef *clock, TIM_HandleTypeDef *htim, float alpha) {
	encoder->clock = clock;
	encoder->htim = htim;
	encoder->speed = 0;

	encoder->prevCount = 0;
	encoder->prevTime = __HAL_TIM_GET_COUNTER(clock);

	encoder->alpha = alpha;

	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void PID_Init(PID_Controller *controller, TIM_HandleTypeDef *clock, Encoder *encoder,
		double k_p, double k_i, double k_d, double maxIntegral) {
	controller->clock = clock;
	controller->encoder = encoder;

	controller->k_p = k_p;
	controller->k_i = k_i;
	controller->k_d = k_d;
	controller->maxIntegral = maxIntegral;

	controller->errorIntegral = 0;
	controller->prevError = 0;
	controller->prevTime = __HAL_TIM_GET_COUNTER(clock);
}

void Motor_Init(Motor *motor, TIM_HandleTypeDef *clock, TIM_HandleTypeDef *htim, TIM_HandleTypeDef *pwmTimer,
		uint8_t pwmChannel, GPIO_TypeDef *dirGPIOPeripheral, uint16_t dirGPIOPin,
		GPIO_TypeDef *faultPeripheral, uint16_t faultPin, bool invert, float alpha) {
	motor->clock = clock;

	Encoder_Init(&motor->encoder, clock, htim, alpha);
	__PID_INIT_DEFAULT(&motor->controller, clock, &motor->encoder);

	motor->pwmTimer = pwmTimer;
	motor->pwmChannel = pwmChannel;
	motor->dirGPIOPeripheral = dirGPIOPeripheral;
	motor->dirGPIOPin = dirGPIOPin;

	motor->faultPeripheral = faultPeripheral;
	motor->faultPin = faultPin;

	motor->invert = invert;

	motor->targetSpeed = 0;
	motor->driveType = STOPPED;
	motor->maxPWM = 1000;

	motor->cyclesSinceStop = 0;
	motor->cyclesDelay = 100;

	HAL_TIM_PWM_Start(pwmTimer, pwmChannel);
}

void Servo_Init(Servo *servo, TIM_HandleTypeDef *pwmTimer, uint8_t pwmChannel) {
	servo->pwmTimer = pwmTimer;
	servo->pwmChannel = pwmChannel;

	HAL_TIM_PWM_Start(pwmTimer, pwmChannel);
}

void UltraS_Init(UltraS *ultrasonic, GPIO_TypeDef *trigPeripheral,uint16_t trigPin,
		TIM_HandleTypeDef *echoTimer, uint8_t echoChannel, uint32_t delayTime) {
	ultrasonic->trigPeripheral = trigPeripheral;
	ultrasonic->trigPin = trigPin;

	ultrasonic->echoTimer = echoTimer;
	ultrasonic->echoChannel = echoChannel;

	ultrasonic->lastUsed = HAL_GetTick();
	ultrasonic->delayTime = delayTime;

	ultrasonic->currentDistance = -1;
	ultrasonic->isFirstCaptured = false;

	ultrasonic->enabled = false;
}

// ==========================================
// Updating
// ==========================================

void Encoder_Update(Encoder *encoder) {
	uint16_t currentTime = __HAL_TIM_GET_COUNTER(encoder->clock);
	uint16_t d_t = currentTime - encoder->prevTime;

	int16_t currentCount = __HAL_TIM_GET_COUNTER(encoder->htim);
	int d_c = currentCount - encoder->prevCount;

	encoder->dt = d_t;
	encoder->dc = d_c;

	double raw_speed = d_c * 1000000 / d_t;

	encoder->speed = (encoder->alpha * raw_speed) + ((1.0f - encoder->alpha) * encoder->speed);
	encoder->prevTime = currentTime;
	encoder->prevCount = currentCount;
}

int PID_Update(PID_Controller *controller, int error) {
	uint16_t currentTime = __HAL_TIM_GET_COUNTER(controller->clock);
	double d_t = (double)(int16_t)(currentTime - controller->prevTime) / 1.0e6;
	d_t = 0.05;
	controller->prevTime = currentTime;

	double errorDerivative = 0;

	if (d_t > 0) {
		errorDerivative = (double)(error - controller->prevError) / d_t;
	}

	controller->prevError = error;
	controller->errorIntegral += error * d_t;

	if (controller->errorIntegral > controller->maxIntegral)
		controller->errorIntegral = controller->maxIntegral;

	if (controller->errorIntegral < -controller->maxIntegral)
			controller->errorIntegral = -controller->maxIntegral;

	int result = error * controller->k_p +
			errorDerivative * controller->k_d +
			controller->errorIntegral * controller->k_i;

	controller->d_t = result;
	return result;
}

void Motor_Update(Motor *motor) {
	Encoder_Update(&motor->encoder);
	motor->cyclesSinceStop ++;

	if (motor->driveType == PID) {
		int currentSpeed = motor->encoder.speed;
		int error = motor->targetSpeed - currentSpeed;

		int pid = PID_Update(&motor->controller, error);

		Motor_Drive(motor, pid);
	}

	else if (motor->driveType == DISCRETE) {
		Motor_Drive(motor, motor->targetSpeed);
	}
}

void Robot_Update(Robot *robot) {
	bool fault = 0;

	fault |= Motor_CheckFault(&robot->frontLeftMotor);
	fault |= Motor_CheckFault(&robot->frontRightMotor);
	fault |= Motor_CheckFault(&robot->backLeftMotor);
	fault |= Motor_CheckFault(&robot->backRightMotor);

	if (fault) {
		Robot_Stop(robot);

		robot->state = STATE_FAULT;

		return;
	}

	if (robot->state == STATE_DRIVING_TIME && HAL_GetTick() >= robot->moveCompleteTime) {
		Robot_Stop(robot);

		return;
	}

	Motor_Update(&robot->frontLeftMotor);
	Motor_Update(&robot->frontRightMotor);
	Motor_Update(&robot->backLeftMotor);
	Motor_Update(&robot->backRightMotor);
}

void UltraS_Update(UltraS *ultrasonic) {
	if (!ultrasonic->enabled) {
		return;
	}

	uint32_t currentTime = HAL_GetTick();

	if (currentTime > ultrasonic->lastUsed + ultrasonic->delayTime) {
		UltraS_SendPulse(ultrasonic);
	}
}

// ==========================================
// Miscellaneous
// ==========================================

void PID_Reset(PID_Controller *controller) {
	controller->prevError = 0;
	controller->prevTime = __HAL_TIM_GET_COUNTER(controller->clock);
	controller->errorIntegral = 0;
}

void Motor_Drive(Motor *motor, int speed) {
	if (motor->invert) speed = -speed;

	uint32_t absSpeed = abs(speed);

	if (speed == 0) {
		Motor_Stop(motor);
		return;
	}

	else if (speed < 0) {
		motor->reversed = true;
		HAL_GPIO_WritePin(motor->dirGPIOPeripheral, motor->dirGPIOPin, GPIO_PIN_RESET);
	}

	else if (speed > 0) {
		motor->reversed = false;
		HAL_GPIO_WritePin(motor->dirGPIOPeripheral, motor->dirGPIOPin, GPIO_PIN_SET);
	}

	if (absSpeed > motor->maxPWM) {
		absSpeed = motor->maxPWM;
	}

	motor->speed = absSpeed;

	__HAL_TIM_SET_COMPARE(motor->pwmTimer, motor->pwmChannel, absSpeed);
}

void Motor_DrivePID(Motor *motor, int speed) {
	if (motor->cyclesSinceStop < motor->cyclesDelay) {
		return;
	}

	if (motor->driveType != PID || abs(speed - motor->targetSpeed) > 250) {
		PID_Reset(&motor->controller);
	}

	motor->driveType = PID;
	motor->targetSpeed = speed;

	Motor_Update(motor);
}

void Motor_DriveDiscrete(Motor *motor, int speed) {
	if (motor->cyclesSinceStop < motor->cyclesDelay) {
		return;
	}

	motor->driveType = DISCRETE;
	motor->targetSpeed = speed;

	Motor_Update(motor);
}

void Motor_Stop(Motor *motor) {
	HAL_GPIO_WritePin(motor->dirGPIOPeripheral, motor->dirGPIOPin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(motor->pwmTimer, motor->pwmChannel, 0);

	motor->cyclesSinceStop = 0;

	motor->driveType = STOPPED;
}

bool Motor_CheckFault(Motor *motor) {
	return HAL_GPIO_ReadPin(motor->faultPeripheral, motor->faultPin) == GPIO_PIN_RESET;
}

void Robot_DrivePID(Robot *robot, int speed, int strafe, int turn) {
	// HAL_GPIO_WritePin(robot->regEnablePeripheral, robot->regEnablePin, GPIO_PIN_SET);

	int frontLeftSpeed = speed + strafe + turn;
	int frontRightSpeed = speed - strafe - turn;
	int backLeftSpeed = speed - strafe + turn;
	int backRightSpeed = speed + strafe - turn;

	Motor_DrivePID(&robot->frontLeftMotor, frontLeftSpeed);
	Motor_DrivePID(&robot->frontRightMotor, frontRightSpeed);
	Motor_DrivePID(&robot->backLeftMotor, backLeftSpeed);
	Motor_DrivePID(&robot->backRightMotor, backRightSpeed);

	robot->state = STATE_DRIVING;
}

void Robot_Drive(Robot *robot, int speed, int strafe, int turn) {
	// HAL_GPIO_WritePin(robot->regEnablePeripheral, robot->regEnablePin, GPIO_PIN_SET);

	int frontLeftSpeed = speed + strafe + turn;
	int frontRightSpeed = speed - strafe - turn;
	int backLeftSpeed = speed - strafe + turn;
	int backRightSpeed = speed + strafe - turn;

	Motor_DriveDiscrete(&robot->frontLeftMotor, frontLeftSpeed);
	Motor_DriveDiscrete(&robot->frontRightMotor, frontRightSpeed);
	Motor_DriveDiscrete(&robot->backLeftMotor, backLeftSpeed);
	Motor_DriveDiscrete(&robot->backRightMotor, backRightSpeed);

	robot->state = STATE_DRIVING;
}

void Robot_DriveTime(Robot *robot, int speed, int strafe, int turn, int time_ms) {
	// HAL_GPIO_WritePin(robot->regEnablePeripheral, robot->regEnablePin, GPIO_PIN_SET);

	int frontLeftSpeed = speed + strafe + turn;
	int frontRightSpeed = speed - strafe - turn;
	int backLeftSpeed = speed - strafe + turn;
	int backRightSpeed = speed + strafe - turn;

	Motor_DriveDiscrete(&robot->frontLeftMotor, frontLeftSpeed);
	Motor_DriveDiscrete(&robot->frontRightMotor, frontRightSpeed);
	Motor_DriveDiscrete(&robot->backLeftMotor, backLeftSpeed);
	Motor_DriveDiscrete(&robot->backRightMotor, backRightSpeed);

	robot->moveCompleteTime = HAL_GetTick() + time_ms;

	robot->state = STATE_DRIVING_TIME;
}

void Robot_Stop(Robot *robot) {
	Motor_Stop(&robot->frontLeftMotor);
	Motor_Stop(&robot->frontRightMotor);
	Motor_Stop(&robot->backLeftMotor);
	Motor_Stop(&robot->backRightMotor);

	// HAL_GPIO_WritePin(robot->regEnablePeripheral, robot->regEnablePin, GPIO_PIN_RESET);
	robot->state = STATE_STOPPED;
}

void Servo_SetAngle(Servo *servo, int angle) {
	angle = angle < 0 ? 0 : (angle > 180 ? 180 : angle);
	int period = __HAL_TIM_GET_AUTORELOAD(servo->pwmTimer) + 1; // 50Hz / 20ms period

	double percent_rotation = (double)angle / 90.0;

	int counts_per_ms = 0.05 * period;

	int duty_cycle = counts_per_ms * (percent_rotation + 0.5);

	__HAL_TIM_SET_COMPARE(servo->pwmTimer, servo->pwmChannel, duty_cycle);
}

void Servo_Drive(Servo *servo, int8_t dir) {
	int period = __HAL_TIM_GET_AUTORELOAD(servo->pwmTimer) + 1; // 50Hz / 20ms period
	int counts_per_ms = 0.05 * period;
	double duty_cycle_ms = (double)dir / 256.0 + 1.5;
	int duty_cycle = counts_per_ms * duty_cycle_ms;
	__HAL_TIM_SET_COMPARE(servo->pwmTimer, servo->pwmChannel, duty_cycle);
}

void UltraS_SendPulse(UltraS *ultrasonic) {
	// Check if already waiting for pulse

	if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_SET) {
		return;
	}

	HAL_GPIO_WritePin(ultrasonic->trigPeripheral, ultrasonic->trigPin, GPIO_PIN_RESET);

	delay(10);

	HAL_GPIO_WritePin(ultrasonic->trigPeripheral, ultrasonic->trigPin, GPIO_PIN_SET);

	ultrasonic->lastUsed = HAL_GetTick();
}

// ==========================================
// Setup
// ==========================================

void setupRobot(Robot *robot) {
	TIM_HandleTypeDef *clock = &htim6;
	HAL_TIM_Base_Start(clock);
	robot->regEnablePeripheral = GPIOC;
	robot->regEnablePin = GPIO_PIN_5;

	robot->state = STATE_STOPPED;

	Motor_Init(&robot->frontLeftMotor, clock, &htim2, &htim5, TIM_CHANNEL_3, GPIOD, GPIO_PIN_9, GPIOD, GPIO_PIN_7, true, 0.3);
	Motor_Init(&robot->frontRightMotor, clock, &htim3, &htim5, TIM_CHANNEL_4, GPIOD, GPIO_PIN_8, GPIOB, GPIO_PIN_0, true, 0.3);
	Motor_Init(&robot->backLeftMotor, clock, &htim4, &htim5, TIM_CHANNEL_1, GPIOD, GPIO_PIN_11, GPIOD, GPIO_PIN_14, false, 0.3);
	Motor_Init(&robot->backRightMotor, clock, &htim1, &htim5, TIM_CHANNEL_2, GPIOD, GPIO_PIN_10, GPIOE, GPIO_PIN_12, false, 0.3);
}

void setupServos(Servo *servos) {
	Servo_Init(servos, &htim8, TIM_CHANNEL_1);
	Servo_Init(servos + 1, &htim8, TIM_CHANNEL_2);
	Servo_Init(servos + 2, &htim8, TIM_CHANNEL_3);
}

void setupUltraS(UltraS *ultrasonic) {
	UltraS_Init(ultrasonic, GPIOE, GPIO_PIN_6, &htim9, TIM_CHANNEL_1, 100);
}

/**
 * @brief  Reads the internal temperature sensor and returns the value in Celsius.
 * @note   ADC and the Temperature Sensor channel must be pre-configured in CubeMX.
 * @retval Temperature in Degrees Celsius
 */
float Read_Internal_Temp() {
    uint32_t raw_adc = 0;
    float temperature = 0.0f;

    // 1. Start the ADC Peripheral
    HAL_ADC_Start(&hadc1);

    // 2. Poll for conversion complete (10ms timeout)
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        // 3. Get the raw 12-bit ADC value
        raw_adc = HAL_ADC_GetValue(&hadc1);
    }

    // 4. Stop the ADC to save power
    HAL_ADC_Stop(&hadc1);

    float v_sense = ((float)raw_adc * 3.3f) / 4095.0f;
    temperature = ((v_sense - 0.76f) / 0.0025f) + 25.0f;

    return temperature;
}

void delay (uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim9, 0);
	while (__HAL_TIM_GET_COUNTER(&htim9) < time);
}

// ==========================================
// I2C
// ==========================================

uint8_t IsReadCommand(uint8_t cmd) {
    return (cmd & 0x80) != 0;
}

uint8_t GetRxLengthForCommand(uint8_t cmd) {
    switch(cmd) {
        case CMD_DRIVE:
            return 12;
        case CMD_DRIVE_PID:
        	return 12;
        case CMD_DRIVE_TIME:
        	return 16;
        case CMD_STOP:
            return 0;
        case CMD_SET_SERVO:
        	return 2;
        case CMD_DRIVE_SERVO:
        	return 2;
        case CMD_EN_ULTRAS:
        	return 0;
        case CMD_STOP_ULTRAS:
        	return 0;
        default:
            return 0;  // Read commands or unknown
    }
}

uint8_t GetTxLengthForCommand(uint8_t cmd) {
    switch(cmd) {
        case CMD_READ_STATUS:
            return 1;
        case CMD_READ_VEL:
            return 16;
        case CMD_READ_ENC:
            return 16;
        case CMD_READ_ULTRAS:
        	return 4;
        case CMD_READ_TEMP:
        	return 4;
        default:
            return 0;  // Write commands or unknown
    }
}

void InsertIntoBuffer(int data, uint8_t *buffer) {
	buffer[0] = data >> 24 & 0xFF;
	buffer[1] = data >> 16 & 0xFF;
	buffer[2] = data >> 8 & 0xFF;
	buffer[3] = data & 0xFF;
}

int intFromBuffer(uint8_t *buffer) {
	int result = 0;

	result |= buffer[0] << 24;
	result |= buffer[1] << 16;
	result |= buffer[2] << 8;
	result |= buffer[3];

	return result;
}

void PrepareResponseData(uint8_t cmd) {
    tx_length = GetTxLengthForCommand(cmd);

    switch(cmd) {
        case CMD_READ_STATUS:
            tx_buffer[0] = robot.state;
            break;

        case CMD_READ_VEL:
            InsertIntoBuffer(robot.frontLeftMotor.encoder.speed, (uint8_t *)tx_buffer);
            InsertIntoBuffer(robot.frontRightMotor.encoder.speed, (uint8_t *)tx_buffer + 4);
            InsertIntoBuffer(robot.backLeftMotor.encoder.speed, (uint8_t *)tx_buffer + 8);
            InsertIntoBuffer(robot.backRightMotor.encoder.speed, (uint8_t *)tx_buffer + 12);
            break;

        case CMD_READ_ENC:
        	InsertIntoBuffer(__HAL_TIM_GET_COUNTER(robot.frontLeftMotor.encoder.htim), (uint8_t *)tx_buffer);
        	InsertIntoBuffer(__HAL_TIM_GET_COUNTER(robot.frontRightMotor.encoder.htim), (uint8_t *)tx_buffer + 4);
        	InsertIntoBuffer(__HAL_TIM_GET_COUNTER(robot.backLeftMotor.encoder.htim), (uint8_t *)tx_buffer + 8);
        	InsertIntoBuffer(__HAL_TIM_GET_COUNTER(robot.backRightMotor.encoder.htim), (uint8_t *)tx_buffer + 12);
            break;

        case CMD_READ_ULTRAS:
        	InsertIntoBuffer((int)ultrasonic.currentDistance, (uint8_t *)tx_buffer);
        	break;

        case CMD_READ_TEMP:
        	float temp = Read_Internal_Temp();
        	uint32_t temp_int = 100 * temp;
        	InsertIntoBuffer(temp_int, (uint8_t *)tx_buffer);
        	break;

        default:
            tx_length = 0;
            break;
    }
}

void ProcessReceivedData(uint8_t cmd, uint8_t *data, uint8_t len) {
    switch(cmd) {
        case CMD_DRIVE_PID: {
            int speed = intFromBuffer(data);
            int strafe = intFromBuffer(data + 4);
            int turn = intFromBuffer(data + 8);

            Robot_DrivePID(&robot, speed, strafe, turn);
            break;
        }

        case CMD_STOP:
        	Robot_Stop(&robot);
            break;

        case CMD_DRIVE: {
        	int speed = intFromBuffer(data);
			int strafe = intFromBuffer(data + 4);
			int turn = intFromBuffer(data + 8);

			Robot_Drive(&robot, speed, strafe, turn);
			break;
        }

        case CMD_DRIVE_TIME: {
			int speed = intFromBuffer(data);
			int strafe = intFromBuffer(data + 4);
			int turn = intFromBuffer(data + 8);
			int time_ms = intFromBuffer(data + 12);

			Robot_DriveTime(&robot, speed, strafe, turn, time_ms);
			break;
		}

        case CMD_SET_SERVO: {
        	int servoNum = data[0];
        	Servo *servo = servos + servoNum;
        	int angle = data[1];

        	Servo_SetAngle(servo, angle);
        	break;
        }
        case CMD_DRIVE_SERVO: {
        	int servoNum = data[0];
        	Servo *servo = servos + servoNum;
        	int8_t dir = data[1];

        	Servo_Drive(servo, dir);
        	break;
        }
        case CMD_EN_ULTRAS:
        	ultrasonic.enabled = true;
        	break;
        case CMD_STOP_ULTRAS:
        	ultrasonic.enabled = true;
        	break;
        default:
            break;
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        i2c_state = STATE_WAIT_COMMAND;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t*)&command_byte, 1, I2C_FIRST_FRAME);
    }

    else {
        i2c_state = STATE_SEND_RESPONSE;

        PrepareResponseData(command_byte);

        if (tx_length > 0) {
            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)tx_buffer, tx_length, I2C_LAST_FRAME);
        } else {
            i2c_state = STATE_IDLE;
            HAL_I2C_EnableListen_IT(hi2c);
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (i2c_state == STATE_WAIT_COMMAND) {
        // Command byte received

        if (IsReadCommand(command_byte)) {
        	// This shouldn't happen
            i2c_state = STATE_IDLE;
            HAL_I2C_EnableListen_IT(hi2c);
        }
        else {
            rx_length = GetRxLengthForCommand(command_byte);

            if (rx_length > 0) {
                i2c_state = STATE_WAIT_DATA;
                HAL_I2C_Slave_Seq_Receive_IT(hi2c, (uint8_t*)rx_buffer, rx_length, I2C_LAST_FRAME);
            } else {
                i2c_state = STATE_IDLE;
                data_received = 1;
                HAL_I2C_EnableListen_IT(hi2c);
            }
        }
    }
    else if (i2c_state == STATE_WAIT_DATA) {
        i2c_state = STATE_IDLE;
        data_received = 1;
        HAL_I2C_EnableListen_IT(hi2c);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Data transmission to master complete
    i2c_state = STATE_IDLE;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Re-enable listening for next transaction
    HAL_I2C_EnableListen_IT(hi2c);
}

// I2C Error Callback
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    // Handle error - reset state and re-enable listening
    i2c_state = STATE_IDLE;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM9) {
		if (!ultrasonic.isFirstCaptured) { // First edge (Rising)
			ultrasonic.icVal1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			ultrasonic.isFirstCaptured = true;
			// Change polarity to capture falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else { // Second edge (Falling)
			ultrasonic.icVal2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			__HAL_TIM_SET_COUNTER(htim, 0); // Reset counter for next cycle

			uint32_t difference;

			if (ultrasonic.icVal2 > ultrasonic.icVal1) {
				difference = ultrasonic.icVal2 - ultrasonic.icVal1;
			} else {
				difference = (0xFFFF - ultrasonic.icVal1) + ultrasonic.icVal2;
			}

			ultrasonic.currentDistance = difference * 0.34 / 2;
			ultrasonic.isFirstCaptured = false;
			// Reset polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		}
	}
}

// ==========================================
// Core Functions
// ==========================================

void setup() {
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	HAL_I2C_EnableListen_IT(&hi2c1);
	setupRobot(&robot);
	setupServos(servos);
	setupUltraS(&ultrasonic);

	HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_1);
}

void loop() {
	if (data_received) {
		data_received = 0;
		ProcessReceivedData(command_byte, (uint8_t*)rx_buffer, rx_length);
	}

	// HAL_GPIO_WritePin(robot.regEnablePeripheral, robot.regEnablePin, GPIO_PIN_SET);

	// Robot_Drive(&robot, 2000, 0, 0);
	Robot_Update(&robot);
	UltraS_Update(&ultrasonic);

	HAL_Delay(1);
}

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
