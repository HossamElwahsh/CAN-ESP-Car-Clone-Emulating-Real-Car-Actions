/*
 * DCM_RTOS.h
 *
 *  Created on: Oct 21, 2023
 *      Author: Salma
 */

#ifndef DCM_DCM_RTOS_H_
#define DCM_DCM_RTOS_H_

#include "stm32f1xx_hal.h"
#include "DCM_interface.h"
#include "cmsis_os.h"
#include "app_config.h"
#include "app_interface.h"

/* Gear Positions */
# define GEAR_NEUTRAL ((uint8_t)112)
# define GEAR_DRIVE   ((uint8_t)113)
# define GEAR_REVERSE ((uint8_t)119)
# define GEAR_PARKING ((uint8_t)127)

/* Steering Positions */
# define STEERING_STRAIGHT    ((uint8_t)224)
# define STEERING_RIGHT       ((uint8_t)226)
# define STEERING_SHARP_RIGHT ((uint8_t)227)
# define STEERING_LEFT 		  ((uint8_t)232)
# define STEERING_SHARP_LEFT  ((uint8_t)236)

#define GEAR_SEMAPHORE			(semaphore_transmissionHandle)
#define STEERING_SEMAPHORE      (semaphore_steeringHandle)

/* External Variables*/
extern Steering_en gl_steering_en;
extern uint8_t gl_u8_throttle;
extern transmission_en gl_transmission_en;
extern TIM_HandleTypeDef htim3;
extern TaskHandle_t TH_DCM;
extern SemaphoreHandle_t semaphore_transmissionHandle;
extern SemaphoreHandle_t semaphore_steeringHandle;

/* Global Variables Defines */
#define THROTTLE_POSITION 		(gl_u8_throttle)
#define STEERING_WHEEL_POSITION (gl_steering_en)
#define GEAR_POSITION 			(gl_transmission_en)


/* Function Prototypes */
void Task_DCM(void *pvParameters);

#endif /* DCM_DCM_RTOS_H_ */
