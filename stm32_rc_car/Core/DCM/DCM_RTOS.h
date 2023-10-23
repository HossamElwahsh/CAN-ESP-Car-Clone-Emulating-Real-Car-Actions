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

/* Speed Levels */
# define SPEED_ZERO  	((uint8_t)0)
# define SPEED_MIN  	((uint8_t)70)
# define SPEED_MED 	    ((uint8_t)80)
# define SPEED_HIGH 	((uint8_t)90)
# define SPEED_MAX      ((uint8_t)100)



#define GEAR_SEMAPHORE			(semaphore_transmissionHandle)
#define STEERING_SEMAPHORE      (semaphore_steeringHandle)

/* External Variables*/
extern Steering_en gl_steering_en;
extern throttle_en gl_throttle_en;
extern throttle_en gl_transmission_en;
extern TIM_HandleTypeDef htim3;
extern TaskHandle_t TH_DCM;
extern SemaphoreHandle_t semaphore_transmissionHandle;
extern SemaphoreHandle_t semaphore_steeringHandle;

/* Global Variables Defines */
#define THROTTLE_POSITION 		(gl_throttle_en)
#define STEERING_WHEEL_POSITION (gl_steering_en)
#define GEAR_POSITION 			(gl_transmission_en)


/* Function Prototypes */
void Task_DCM(void *pvParameters);

#endif /* DCM_DCM_RTOS_H_ */
