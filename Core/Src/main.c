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
#include "usb_device.h"
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "MPUXX50.h"
#include "magnetometer.h"
#include "adxl345.h"
#include "MAGNETO.h"
//#include "sr04.h"
//#include "gy85.h"
//#include "motor_control.h"
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile rxque usb = { 0 };
srcom com[2] = { 0 };
volatile wheel right_wheel = { 0 };
volatile wheel left_wheel = { 0 };
uint32_t t_count_rpm1 = 0;
uint32_t t_count_rpm2 = 0;
uint32_t pid_counter = 0;
PID left_pid = { 0 };
PID right_pid = { 0 };
volatile uint8_t check_pid = { 0 };
#define LEFT 1
#define RIGHT 2
uint8_t set_rpm1 = 0;
uint8_t set_rpm2 = 0;
uint8_t isDirect = 0;
int sampling_time = 2000;

volatile int mainmode = 0;
volatile int submode = 0;

int arrDiff[20];
uint8_t arr_i = 0;
uint16_t elapsedCounter = 0;
#define CCRMAX 8000
#define CCRMIN 0
#define USB 0
#define SERIAL 1
char serial_buffer[100];
int mDelay = 0;
#define diameter 10.16
#define slot_dist 1.59
volatile double current_dist = 0;
volatile double user_dist = 0;
volatile double distance_difference = 0;
uint8_t mag_buffer[6];
int16_t mag_x, mag_y, mag_z;
#define base 30
double arc_length = 0;
#define TRUE  1
#define FALSE 0
float roll = 0;
float pitch = 0;
uint8_t turning = 0;
#define ADC_BUFFER_SIZE 5
uint32_t adc_buffer[5] = {};

// Relay1 Left
// Relay2 Right

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
//
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void update_rpm() {
//	 M1.rpm = (avgl * 20) / 84e6;
//	 left_pid.cvalue = M1.rpm;
//}
void setpwm(uint16_t value, uint32_t chanel) {
	switch (chanel) {
	case 1:
		TIM1->CCR1 = value;
		break;
	case 2:
		TIM1->CCR2 = value;
		break;
	}
}

void getDifference(wheel *w) {
	switch (w->state) {
	case 0:
		w->sCount = TIM2->CNT;
		w->state = 1;
		w->distance += slot_dist;
		break;
	case 1:
		w->eCount = TIM2->CNT;
		t_count_rpm1 = 0;
		int diff = w->eCount - w->sCount;
		if (diff > 320000) {
			w->diff = diff;
		}
		w->distance += slot_dist;
		w->sCount = w->eCount;
		break;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == LEFT_Pin) {
		right_wheel.stop_counter = 0;
		getDifference(&right_wheel);
	} else if (GPIO_Pin == RIGHT_Pin) {
		left_wheel.stop_counter = 0;
		getDifference(&left_wheel);
	}
}
double calc_theta(int, int);
double getArcLength(int, int);
void Transmit_SERIAL_USB(char *buf, uint8_t chanel) {
	switch (chanel) {
	case 0:
		CDC_Transmit_FS(buf, strlen(buf));
		break;
	case 1:
		for (int i = 0; i < strlen(buf); i++) {
			Transmit(&huart2, buf[i]);
		}
		break;
	}
}
//void Transmit_SERIAL_USB(char *buf) {
//	Transmit_SERIAL_USB_1(buf);
//	HAL_UART_Transmit(&huart2, buf, sizeof(buf), 100);
//}
void INIT() {
	right_pid.coffp = 0.5;
	right_pid.coffi = 1; // change to 4
	right_pid.coffd = 0;
	right_pid.outmax = 8000;
	right_pid.outmin = 600;
	right_pid.imin = -5000;
	right_pid.imax = 5000;
	right_pid.integral = 0;
	right_pid.side = 1;
	right_wheel.base_start = 3000;
	right_wheel.stop_counter = 0; //Max 2000

	left_pid.coffp = 0.5; //3.056
	left_pid.coffi = 1;  //
	left_pid.coffd = 0;
	left_pid.outmax = 8000;
	left_pid.outmin = 600;
	left_pid.imin = -5000;
	left_pid.imax = 5000;
	left_pid.integral = 0;
	left_pid.side = 2;
	left_wheel.base_start = 3000;
	left_wheel.stop_counter = 0; //Max 2000
}

void calcPID(PID *in) {
	in->error = in->svalue - in->cvalue;
	in->integral += (in->error / 5);
	if (in->integral > in->imax)
		in->integral = in->imax;
	else if (in->integral < in->imin)
		in->integral = in->imin;
	int derr = in->prev_error - in->error;
	in->prev_error = in->error;
	double out = (in->coffp * in->error) + (in->coffi * in->integral)
			+ (in->coffd * derr);
	int tem_out = lrint(out);
	if (tem_out < in->outmin)
		tem_out = in->outmin;
	else if (tem_out > in->outmax)
		tem_out = in->outmax;
	in->output = tem_out;
}
// chanel 0 is usb chanel 1 is serial port
char ischar(int chanel) {
	switch (chanel) {
	case USB:
		if (usb.head == usb.tail)
			return 0;
		else
			return 1;
		break;
	case SERIAL:
		return csts(&huart2);
		break;

	}
}

char readchar(int chanel) {
	switch (chanel) {
	case USB: {
		char t = 0;
		if (ischar(chanel))
			t = usb.buf[usb.tail++];
		else
			return 0;
		usb.tail %= 100;
		return t;
	}
		break;

	case SERIAL: {
		char t = 0;
		if (ischar(chanel)) {
			t = Receive(&huart2, 10);
			return t;
		} else
			return 0;
	}
		break;
	}
}

void toggleDirection(uint8_t wheel_side) {
	switch (wheel_side) {
	case LEFT:
		TIM1->CCR2 = 0;
		HAL_Delay(2000);
		HAL_GPIO_TogglePin(Relay2_GPIO_Port, Relay2_Pin);
		break;
	case RIGHT:
		TIM1->CCR1 = 0;
		HAL_Delay(2000);
		HAL_GPIO_TogglePin(Relay1_GPIO_Port, Relay1_Pin);
		break;
	}
}

double get_distance() {
	return left_wheel.distance;
//	return(left_wheel.distance + right_wheel.distance)/2;
}

double calc_distance(int x, int y) {
	return sqrt(x * x + y * y);
}
void startRotation(double theta) {
	TIM1->CCR2 = 0;
	TIM1->CCR1 = 0;
	HAL_Delay(500);
	if (theta < 0) {
		HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, GPIO_PIN_RESET);

	} else {
		HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, GPIO_PIN_SET);
	}
	HAL_Delay(500);
	TIM1->CCR2 = 3000;
	TIM1->CCR1 = 3000;
}
#define TIME_TO_COVER 10
#define CCR_SCALE_FACTOR 66.66
#define TYRE_DIA 10.16
#define MAX_CCR 8000
#define MIN_CCR 1600
#define PI 3.1428
#define RPM_FACTOR (60 / (2 * PI * TYRE_DIA))

double TURN_RIGHT_LEFT_WITH_TIME(double radius, double distance,
		uint8_t direction) {

	//theta *= (PI / 180);
	float theta = distance / radius; //100 / 20
	double omega = theta / TIME_TO_COVER;
	double velocity_right = omega * (radius - (base / 2)); // 5
	double velocity_left = omega * (radius + (base / 2)); //6.27

	// Convert to RPM
	velocity_right = (velocity_right / RPM_FACTOR); // 9.
	velocity_left = (velocity_left / RPM_FACTOR); //11.7

	// Scale UP
	velocity_right *= CCR_SCALE_FACTOR;
	velocity_left *= CCR_SCALE_FACTOR;

	double max_velocity =
			(velocity_right > velocity_left) ? velocity_right : velocity_left;

	// Scaling Down
	if (max_velocity > MAX_CCR) {
		double scaling_ratio = MAX_CCR / max_velocity;
		velocity_right = scaling_ratio;
		velocity_left = scaling_ratio;
	}

	if (direction == LEFT) {
		TIM1->CCR1 = velocity_left;
		TIM1->CCR2 = velocity_right;
	} else {
		TIM1->CCR1 = velocity_right;
		TIM1->CCR2 = velocity_left;
	}

	return (radius - (base / 2) * theta);

}
double TURN_RIGHT_LEFT(double radius, double theta, uint8_t direction) {

	theta *= (3.14 / 180);
	// Calculate angular velocity
	double distance = radius * theta;
	double omega = theta / (distance / radius);

	// Calculate relative radii for the left and right tires
	double radius_left = radius + (base / 2);
	double radius_right = radius - (base / 2);

	// Calculate linear velocities based on angular velocity and relative radii
	double velocity_left = omega * radius_left;
	double velocity_right = omega * radius_right;

	// Convert linear velocities to RPM
	double rpm_factor = 60 / (2 * 3.14 * TYRE_DIA);
	velocity_left /= rpm_factor;
	velocity_right /= rpm_factor;

	// Scale up velocities
	velocity_left *= CCR_SCALE_FACTOR;
	velocity_right *= CCR_SCALE_FACTOR;

	// Determine the maximum velocity
	double max_velocity =
			(velocity_right > velocity_left) ? velocity_right : velocity_left;
	double min_velocity =
			(velocity_right < velocity_left) ? velocity_right : velocity_left;

	// Scaling down if necessary
	if (max_velocity > MAX_CCR) {
		double scaling_ratio = MAX_CCR / max_velocity;
		velocity_left *= scaling_ratio;
		velocity_right *= scaling_ratio;
	}
	if (min_velocity < MIN_CCR) {
		double scaling_ratio = MIN_CCR / min_velocity;
		velocity_left *= scaling_ratio;
		velocity_right *= scaling_ratio;
	}

	// Set PWM CCR values based on direction
	if (direction == LEFT) {
		TIM1->CCR1 = (uint16_t) velocity_left;
		TIM1->CCR2 = (uint16_t) velocity_right;
	} else {
		TIM1->CCR1 = (uint16_t) velocity_right;
		TIM1->CCR2 = (uint16_t) velocity_left;
	}

	// Return the new radius or any other calculated value as needed
	return radius_right;
}
void process(char *buf, uint8_t chanel) {
	char *ln = NULL;
	char msg[100];
	if ((ln = strstr(buf, "GOTOX:")) != NULL) {
		int tmp = atoi(&ln[6]);
		if ((ln = strstr(buf, ",Y:")) != NULL) {
			int tmp2 = atoi(&ln[3]);
			current_dist = get_distance();
			arc_length = getArcLength(tmp, tmp2);
			startRotation(calc_theta(tmp, tmp2));
			user_dist = calc_distance(tmp, tmp2);
			mainmode = 4;
		}
		sprintf(msg, "arch length is %d", arc_length);
		Transmit_SERIAL_USB(msg, chanel);
	} else if ((ln = strstr(buf, "SETWR:")) != NULL) {
		uint16_t tmp = atoi(&ln[6]);
		//printf("Motor 1 CCR set to %d\r\n",tmp);
		sprintf(msg, "Motor 1 CCR Set to %d\r\n", tmp);
//		left_pid.svalue = tmp*100;
		//Transmit_SERIAL_USB(msg, chanel);
		//pwmcheck(tmp, 1);
		isDirect = 1;
		TIM1->CCR1 = tmp;
	} else if ((ln = strstr(buf, "SETWL:")) != NULL) {
		uint16_t tmp = atoi(&ln[6]);
		sprintf(msg, "Motor 2 CCR set to %d\r\n", tmp);
		//		right_pid.svalue = tmp*100;
		//Transmit_SERIAL_USB(msg, chanel);
		isDirect = 1;
		//pwmcheck(tmp, 2);

		TIM1->CCR2 = tmp;
	} else if ((ln = strstr(buf, "SETPIDL:")) != NULL) {
		uint16_t tmp = atoi(&ln[8]);
		sprintf(msg, "Left Wheel PID set to %d\r\n", tmp);
		Transmit_SERIAL_USB(msg, chanel);
		//pwmcheck(tmp, 2);
		left_pid.svalue = tmp * 100;
		isDirect = 0;
		set_rpm1 = 1;
		mainmode = 1;
	} else if ((ln = strstr(buf, "SETPIDR:")) != NULL) {
		uint16_t tmp = atoi(&ln[8]);
		sprintf(msg, "Right Wheel PID set to %d\r\n", tmp);
		Transmit_SERIAL_USB(msg, chanel);
		isDirect = 0;
		//pwmcheck(tmp, 2);
		right_pid.svalue = tmp * 100;
		set_rpm2 = 1;
		mainmode = 1;
	} else if ((ln = strstr(buf, "SETPPR:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Motor 2 coffP set to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		//pwmcheck(tmp, 2);
		right_pid.coffp = tmp;
	} else if ((ln = strstr(buf, "SETPIR:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Motor 2 coffI set to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		//pwmcheck(tmp, 2);
		right_pid.coffi = tmp;
	} else if ((ln = strstr(buf, "SETPDR:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Motor 2 coffD set to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		//pwmcheck(tmp, 2);
		right_pid.coffd = tmp;
	} else if ((ln = strstr(buf, "SETPPL:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Motor 2 coffP set to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		//pwmcheck(tmp, 2);
		left_pid.coffp = tmp;
	} else if ((ln = strstr(buf, "SETPIL:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Motor 2 coffI set to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		//pwmcheck(tmp, 2);
		left_pid.coffi = tmp;
	} else if ((ln = strstr(buf, "SETPDL:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Motor 2 coffD set to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		left_pid.coffd = tmp;
	} else if ((ln = strstr(buf, "SETST:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Sampling Time SET to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		sampling_time = tmp;
	} else if ((ln = strstr(buf, "SETWD:")) != NULL) {
		double tmp = atof(&ln[7]);
		sprintf(msg, "Motor 2 coffD set to %d\r\n", (int) tmp);
		Transmit_SERIAL_USB(msg, chanel);
		left_pid.coffd = tmp;
	} else if ((ln = strstr(buf, "STOP")) != NULL) {
		Transmit_SERIAL_USB(msg, chanel);
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		left_pid.svalue = 0;
		right_pid.svalue = 0;
		isDirect = 1;
		mainmode = 0;
	} else if ((ln = strstr(buf, "RWL")) != NULL) {
		sprintf(msg, "Reversed Wheel LEFT");
		Transmit_SERIAL_USB(msg, chanel);
		toggleDirection(LEFT);
	} else if ((ln = strstr(buf, "RWR")) != NULL) {
		sprintf(msg, "Reversed Wheel RIGHT");
		Transmit_SERIAL_USB(msg, chanel);
		toggleDirection(RIGHT);
	} else if ((ln = strstr(buf, "GOSTR:")) != NULL) {
		double tmp = atof(&ln[6]);
		//if ((ln = strstr(buf, "GOSTR:")) != NULL) {
		//double tmp2 = atof(&ln[6]);
		sprintf(msg, "Rover go to distance %d with speed %d\r\n", (int) tmp,
				4000);
		Transmit_SERIAL_USB(msg, chanel);
		user_dist = tmp;
		mainmode = 2;
	} else if ((ln = strstr(buf, "CS")) != NULL) {
		printVariables(msg, chanel);
	} else if ((ln = strstr(buf, "GOSTRP:")) != NULL) {
		double tmp = atof(&ln[7]);
		//if ((ln = strstr(buf, "GOSTR:")) != NULL) {
		//double tmp2 = atof(&ln[6]);
		sprintf(msg, "Rover go to distance %d with speed %d\r\n", (int) tmp,
				3000);
		Transmit_SERIAL_USB(msg, chanel);
//			current_dist = get_distance();
//			user_dist = tmp / 4;
//			left_pid.svalue = 30 * 100;
//			right_pid.svalue = left_pid.svalue;
		isDirect = 0;
		set_rpm1 = 1;
		set_rpm2 = 1;
		mainmode = 2;
	} else if ((ln = strstr(buf, "RAD:")) != NULL) {
		double radius = atof(&ln[4]);
		if ((ln = strstr(buf, ",THETA:")) != NULL) {
			double theta = atof(&ln[7]);
			sprintf(msg, "Rover RADIUS IS: %d, THETA IS: %d\r\n", (int) radius,
					theta);
			Transmit_SERIAL_USB(msg, chanel);
			if (theta >= 0)
				user_dist = TURN_RIGHT_LEFT(radius, theta, RIGHT);
			if (theta < 0)
				user_dist = TURN_RIGHT_LEFT(radius, theta, LEFT);

			turning = 1;
			right_wheel.distance = 0;
			left_wheel.distance = 0;
			mainmode = 3;

//			current_dist = get_distance();
//			user_dist = tmp / 4;
//			left_pid.svalue = 30 * 100;
//			right_pid.svalue = left_pid.svalue;
//			isDirect = 0;
//			set_rpm1 = 1;
//			set_rpm2 = 1;
//			mainmode = 2;
		}

	}
}
char* comProcess(int chanel) {
	if (ischar(chanel)) {
		char t = readchar(chanel);
		switch (com[chanel].state) {
		case 0:
			if ('$' == t) {
				com[chanel].state = 1;
				com[chanel].cp = 0;
				com[chanel].buf[com[chanel].cp++] = t;
			}
			break;
		case 1:
			if (t >= ' ') {
				com[chanel].buf[com[chanel].cp++] = t;
				com[chanel].cp %= 100;
			} else if (t == 0x0A) {
				com[chanel].buf[com[chanel].cp] = 0;
				com[chanel].state = 2;
				return com[chanel].buf;
			}
		case 2:
			break;
		}
	}
	return NULL;
}
uint32_t adc_buffer_copy[ADC_BUFFER_SIZE] = {};
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		memcpy(adc_buffer_copy, adc_buffer, ADC_BUFFER_SIZE);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		// Sampling for the ADC
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*) adc_buffer, ADC_BUFFER_SIZE);
//	    MPU_calcAttitude(&hi2c1, &roll, &pitch); // mpu6050
		left_wheel.stop_counter++;
		right_wheel.stop_counter++;
		pid_counter++;
		if (left_wheel.stop_counter == 5000) {
			left_wheel.stop_counter = 0;
			left_wheel.rpm = 0;
			left_pid.cvalue = 0;
			left_wheel.diff = 0;
			left_wheel.distance = 0;
			left_wheel.state = 0;
		}
		if (right_wheel.stop_counter == 5000) {
			right_wheel.stop_counter = 0;
			right_wheel.rpm = 0;
			right_pid.cvalue = 0;
			right_wheel.diff = 0;
			right_wheel.distance = 0;
			right_wheel.state = 0;
		}
		if (pid_counter == sampling_time) {
			pid_counter = 0;
			check_pid = 1;
		}
	}
}

void resetCalc(PID *pid, wheel *wheel) {
	pid->cvalue = 0;
	wheel->diff = 0;
}

void saveRPM() {
	left_pid.prev_cvalue = left_pid.cvalue;
	left_wheel.prev_diff = left_wheel.diff;

	right_pid.prev_cvalue = right_pid.cvalue;
	right_wheel.prev_diff = right_wheel.diff;
}

short calcRPM(int difference) {
	if (difference == 0)
		return 0;
	double last_rpm_2 = (double) difference;
	last_rpm_2 = ((last_rpm_2 / 84e6) * 20) / 60;
	last_rpm_2 = 1 / last_rpm_2;
	if ((last_rpm_2 = lrint(last_rpm_2 * 100)) < 0) {
		arrDiff[arr_i++] = difference;
		arr_i %= 19;
	}
	return last_rpm_2;
}

void selfStart(PID *pid) {
	switch (pid->side) {
	case 2: {
		set_rpm2 = 0;
		TIM1->CCR2 = left_wheel.base_start;
		break;
	}
	case 1: {
		set_rpm1 = 0;
		TIM1->CCR1 = right_wheel.base_start;
		break;
	}
		HAL_Delay(500);
	}
}
void printRPM(PID *pid, uint32_t ccr, char *msg, uint8_t chanel) {
	switch (pid->side) {
	case 2: {
		sprintf(msg, "(currentV %d)(setV %d)(CCR %d)\r\n", (int) pid->cvalue,
				(int) pid->svalue, (int) ccr);
		break;
	}
	case 1: {
		sprintf(msg, "RightPID(2): (currentV %d)(setV %d)(CCR %d)\r\n",
				(int) pid->cvalue, (int) pid->svalue, (int) ccr);
		break;
	}
	}
	Transmit_SERIAL_USB(msg, chanel);
}

void printPID(PID *left_pid, char *msg, uint8_t chanel) {
	if (left_pid->side == 2)
		sprintf(msg, "Left: %d %d %d %d %d\r\n", (int) left_pid->cvalue,
				(int) left_pid->svalue, (int) TIM1->CCR2,
				(int) (left_pid->integral), (int) left_pid->coffd,
				(int) left_pid->coffp);
	else
		sprintf(msg, "Right: %d %d %d %d %d\r\n", (int) left_pid->cvalue,
				(int) left_pid->svalue, (int) TIM1->CCR2,
				(int) (left_pid->integral), (int) left_pid->coffd,
				(int) left_pid->coffp);
	Transmit_SERIAL_USB(msg, chanel);
//	HAL_Delay(100);
}

double calc_theta(int x, int y) {
	return (atan(x / y));
}

double getArcLength(int x, int y) {
	return (calc_theta(x, y) * (base / 2));
}

void printVariables(char *msg, uint8_t chanel) {
	sprintf(msg, "Covered Distance: (left)%d, (right)%d, User Distance: %d\r\n",
			(int) (left_wheel.distance), (int) right_wheel.distance,
			(int) user_dist);
	Transmit_SERIAL_USB(msg, chanel);
}

void handleAcc(PID *pid) {

	if (pid->cvalue < pid->svalue && (pid->outmax + 500) <= CCRMAX)
		pid->outmax += 500; // Acceleration
	else if (pid->cvalue > pid->svalue && (pid->outmax - 500) >= CCRMIN)
		pid->outmin -= 500;

}

void reverseWheel(uint8_t wheel_side) {
	switch (wheel_side) {
	case LEFT:
		TIM1->CCR2 = 0;
		HAL_Delay(500);
		HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, GPIO_PIN_SET);
		break;
	case RIGHT:
		TIM1->CCR1 = 0;
		HAL_Delay(500);
		HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, GPIO_PIN_SET);
		break;
	}
}

void reverseBothWheels() {
	toggleDirection(LEFT);
	toggleDirection(RIGHT);
}

void magnetometer(char *msg) {
	// RECEIVE X_axis
	HAL_StatusTypeDef status;
	if (status = HAL_I2C_Mem_Read(&hi2c1, HMC5883l_ADDRESS,
	HMC5883l_ADD_DATAX_MSB_MULTI, 1, DataX, 2, 100) != HAL_OK) {
		sprintf(msg, "Not X %u.", status);
		Transmit_SERIAL_USB(msg, SERIAL);
	}
	Xaxis = ((DataX[1] << 8) | DataX[0]) / 660.f;
	// RECEIVE Y_axis
	if (status = HAL_I2C_Mem_Read(&hi2c1, HMC5883l_ADDRESS,
	HMC5883l_ADD_DATAY_MSB_MULTI, 1, DataY, 2, 100) != HAL_OK) {
		sprintf(msg, "Not Y %u.", status);
		Transmit_SERIAL_USB(msg, SERIAL);
	}
	Yaxis = ((DataY[1] << 8) | DataY[0]) / 660.f;
	// RECEIVE Z_axis
	if (status = HAL_I2C_Mem_Read(&hi2c1, HMC5883l_ADDRESS,
	HMC5883l_ADD_DATAZ_MSB_MULTI, 1, DataZ, 2, 100) != HAL_OK) {
		sprintf(msg, "Not Z %u.\r\n", status);
		Transmit_SERIAL_USB(msg, SERIAL);
	}

	Zaxis = ((DataZ[1] << 8) | DataZ[0]) / 660.f;
	sprintf(msg, "X: %u, Y: %u, Z: %u\r\n", Xaxis, Yaxis, Zaxis);
	Transmit_SERIAL_USB(msg, SERIAL);
}

//void accelerate(short& outMax) {
//	if (outMax + 500)
//}
//
//void checkAcceleration(uint8_t side) {
//	switch(side) {
//	case LEFT:
//		if (left_pid.cvalue > left_pid.svalue)
//	}
//}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	char msg[100] = { 0 };
	char *buf = NULL;
	short temp_cvalue = 0;

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
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_USB_DEVICE_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	INIT();
	// Check if IMU configured properly and block if it didn't
//	if (MPU_begin(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98, 0.004) == TRUE) {
//		sprintf(msg, "Working MPU");
//		Transmit_SERIAL_USB(msg, SERIAL);
//	}
	mDelay = HAL_GetTick();

//	uint8_t RegSettingA = HMC5883l_Enable_A;
//	uint8_t RegSettingB = HMC5883l_Enable_B;
//	uint8_t RegSettingMR = HMC5883l_MR;
//	Xaxis = 0;
//	Yaxis = 0;
//	Zaxis = 0;
//	HAL_I2C_Mem_Write(&hi2c1, HMC5883l_ADDRESS, 0x00 , 1, &RegSettingA , 1, 100);
//	HAL_I2C_Mem_Write(&hi2c1, HMC5883l_ADDRESS, 0x01 , 1, &RegSettingB , 1, 100);
//	HAL_I2C_Mem_Write(&hi2c1, HMC5883l_ADDRESS, 0x02 , 1, &RegSettingMR , 1, 100);
//		ADXL345_Init();
//		MAGNETO_Init();
	uint16_t x = 0, y = 0, z = 0;
	HAL_StatusTypeDef status;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if ((buf = comProcess(SERIAL)) != NULL) {
			process(buf, SERIAL);
			com[SERIAL].state = 0;
		}
		if ((buf = comProcess(USB)) != NULL) {
			process(buf, USB);
			com[USB].state = 0;
		}
		switch (mainmode) {
		case 0:
//				//magnetometer(msg);
//				status = ADXL345_ReadAxis(x, y, z);
//				sprintf(msg, "A: X: %u, Y: %u, Z: %u and status: %u\r\n", x, y,
//						z, status);
//				Transmit_SERIAL_USB(msg, SERIAL);
//				Transmit_SERIAL_USB(msg, USB);
//				status = MAGNETO_ReadAxis(x, y, z);
//				sprintf(msg, "M: X: %u, Y: %u, Z: %u and status: %u\r\n", x, y,
//						z, status);
//				Transmit_SERIAL_USB(msg, SERIAL);
//				Transmit_SERIAL_USB(msg, USB);
////				MPU_calibrateGyro(&hi2c1, 1500);
//				sprintf(msg, "The Roll:%d, Pitch:%d", roll, pitch);
//				Transmit_SERIAL_USB(msg, SERIAL);

			break;
		case 1:
			if (check_pid) {
				check_pid = 0;
				if ((temp_cvalue = calcRPM(left_wheel.diff)) != -1)
					left_pid.cvalue = temp_cvalue;
				if (left_pid.cvalue < 12000) {
					calcPID(&left_pid);
					setpwm(left_pid.output, 2);
				}
				if ((temp_cvalue = calcRPM(right_wheel.diff)) != -1)
					right_pid.cvalue = temp_cvalue;
				if (right_pid.cvalue < 12000) {
					calcPID(&right_pid);
					setpwm(right_pid.output, 1);
				}
//					if ((left_wheel.distance>user_dist) || (right_wheel.distance>user_dist)) {
//						TIM1->CCR1 = 0;
//						TIM1->CCR2 = 0;
//						user_dist = 0;
//						mainmode=0;
//					}
			}
			break;
		case 2:
			// Initializing
			if (user_dist && isDirect) {
				left_wheel.distance = 0;
				right_wheel.distance = 0;
				TIM1->CCR1 = 3500;
				TIM1->CCR2 = 3500;
				sprintf(msg,
						"Rover go to current distance %d given distance %d\r\n",
						(int) get_distance(), (int) current_dist);
				Transmit_SERIAL_USB(msg, SERIAL);
				mainmode = 3;
			}
			if (user_dist && isDirect == 0) {
				left_wheel.distance = 0;
				right_wheel.distance = 0;
				left_pid.svalue = 3500;
				right_pid.svalue = 3500;
				mainmode = 1;
			}
			break;
		case 3:
			if (right_wheel.distance >= user_dist && turning) {
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;
				user_dist = 0;
				mainmode = 0;
				turning = 0;
				break;
			}
			// handling covered distance
			if (((left_wheel.distance >= user_dist)
					|| (right_wheel.distance >= user_dist)) && turning == 0) {
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;
				user_dist = 0;
				mainmode = 0;
				//magnetometer(msg);
			}
			break;
		case 4:
			// Handling rotation
			//startRotation(theta);
			if (get_distance() - current_dist >= arc_length) {
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 0;
				HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(Relay2_GPIO_Port, Relay2_Pin, GPIO_PIN_RESET);
				sprintf(msg, "The wheel has turned for arclength of %d.\r\n ",
						arc_length * 100);
				Transmit_SERIAL_USB(msg, SERIAL);
				current_dist = 0;
				arc_length = 0;
				mainmode = 2;
			}

		}
		if ((HAL_GetTick() - mDelay) >= 1000) {
//			printVariables(msg, SERIAL);
//			printPID(&right_pid, msg, SERIAL);
//			printPID(&left_pid, msg, SERIAL);
			mDelay = HAL_GetTick();
		}
//		sprintf(msg, "LeftPID(1): (currentV %d)(setV %d)(CCR %d)(Integral %d)\r\n",
//				(int) left_pid.cvalue, (int) left_pid.svalue, (int) TIM1->CCR2, (int)(left_pid.integral));
//		Transmit_SERIAL_USB(msg);
////		printRPM(&left_pid, TIM1->CCR2, msg);
//		HAL_Delay(100);
//		sprintf(msg, "RightPID(2): (currentV %d)(setV %d)(CCR %d)\r\n",(int) right_pid.cvalue, (int) right_pid.svalue, (int) TIM1->CCR1);
//		Transmit_SERIAL_USB(msg);
//		printRPM(&right_pid, TIM1->CCR1, msg);

		//SR-04
//	  trigger();
//	  double lapse = stopTime - startTime;
//	  double dist = 17000.0 * (lapse /84e6);
//	  sprintf(msg,"%d\r\n",(int)(dist*1000));
//	  Transmit_SERIAL_USB(msg);
//
//	  //GY-85
//	  ADXL345_ReadAxis(&x, &y, &z);
//	  sprintf(msg, "X: %d", 12);
//	  Transmit_SERIAL_USB(msg);
//	  HAL_Delay(500);
		//PWM
		//  printf("Received dataalskdjf;lasd");

//	  rampUP();
//	  HAL_Delay(1000);
//	  rampDown();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 5;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = 5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 8400;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Relay1_Pin | Relay2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LEFT_Pin RIGHT_Pin */
	GPIO_InitStruct.Pin = LEFT_Pin | RIGHT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Relay1_Pin Relay2_Pin */
	GPIO_InitStruct.Pin = Relay1_Pin | Relay2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
