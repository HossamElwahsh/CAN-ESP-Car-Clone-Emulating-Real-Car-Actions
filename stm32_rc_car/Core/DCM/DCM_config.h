/*
 * DCM_config.h
 *
 *  Created on: Oct 20, 2023
 *      Author: Salma
 */

#ifndef DCM_DCM_CONFIG_H_
#define DCM_DCM_CONFIG_H_

#include "stm32f1xx_hal.h"

/* Left Motors Configurations */
#define DCM_LEFT_PORT	(GPIOA)
#define DCM_LEFT_IN1	(GPIO_PIN_4)
#define DCM_LEFT_IN2	(GPIO_PIN_5)
#define DCM_LEFT_EN     (GPIO_PIN_7)

/* Right Motors Configurations */
#define DCM_RIGHT_PORT	(GPIOA)
#define DCM_RIGHT_IN1	(GPIO_PIN_2)
#define DCM_RIGHT_IN2	(GPIO_PIN_3)
#define DCM_RIGHT_EN    (GPIO_PIN_6)
#endif /* DCM_DCM_CONFIG_H_ */
