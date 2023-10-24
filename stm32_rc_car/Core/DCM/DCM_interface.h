/*
 * DCM_interface.h
 *
 *  Created on: Oct 20, 2023
 *      Author: Salma
 */

#ifndef DCM_DCM_INTERFACE_H_
#define DCM_DCM_INTERFACE_H_

#include "app_interface.h"
#include "DCM_config.h"
#include "stm32f1xx_hal.h"
/*******************************************************************************************************************
 *                                                        Types Declaration                                        *
 *******************************************************************************************************************/
typedef struct
{
	GPIO_TypeDef * DCM_GPIOx;
	uint16_t DCM_IN1_GPIO_PIN_x;
	uint16_t DCM_IN2_GPIO_PIN_x;
	TIM_HandleTypeDef * DCM_htimx;
	uint32_t DCM_TIM_CHANNEL_x;

}st_dcm_config_t;


/*******************************************************************************************************************
 *                                                        Functions Prototypes                                     *
 *******************************************************************************************************************/
/********************************************** DCM Forward Functions **********************************************/

/**
  * @brief  moves the DC motor forward.
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveForward(st_dcm_config_t* motor);

/**
  * @brief  moves the car to the right forward.
  * @note   this function moves the car to the right forward by moving the right DC motor forward
  * 		and stopping the left DC motor
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveRightForward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/**
  * @brief  moves the car to the left forward .
  * @note   this function moves the car to the left forward by moving the left DC motor forward
  * 		and stopping the right DC motor
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveLeftForward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/**
  * @brief  moves the car to the right sharply forward.
  * @note   this function moves the car to the right sharply forward by moving the right DC motor forward
  * 		and moving the left DC motor backward.
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveRightSharpForward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/**
  * @brief  moves the car to the left sharply forward.
  * @note   this function moves the car to the left sharply by moving the left DC motor forward
  * 		and moving the right DC motor backward.
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveLeftSharpForward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/********************************************** DCM Backward Functions**********************************************/
/**
  * @brief  moves the DC motor backward.
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveBackward(st_dcm_config_t* motor);
/**
  * @brief  moves the car to the right backward.
  * @note   this function moves the car to the right backward by moving the right DC motor backward
  * 		and stopping the left DC motor
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveRightBackward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/**
  * @brief  moves the car to the left backward.
  * @note   this function moves the car to the left backward by moving the left DC motor backward
  * 		and stopping the right DC motor
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveLeftBackward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/**
  * @brief  moves the car to the right sharply backward.
  * @note   this function moves the car to the right sharply backward by moving the right DC motor backward
  * 		and moving the left DC motor forward.
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveRightSharpBackward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/**
  * @brief  moves the car to the left sharply backward.
  * @note   this function moves the car to the left sharply backward by moving the left DC motor backward
  * 		and moving the right DC motor forward.
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_MoveLeftSharpBackward(st_dcm_config_t* motor_left , st_dcm_config_t* motor_right);

/******************************************** Other DCM Functions *************************************************/
/**
  * @brief  sets the speed of the DC motor.
  *
  * @param  motor: pointer to the DC motor.
  * @param  speed: speed of the DC motor to be set.
  *    	    	   This parameter can be one of the following values:
  *                @arg SPEED_ZERO: speed is zero.
  *                @arg SPEED_MIN:  minimum speed.
  *                @arg SPEED_MED:  medium speed.
  *                @arg SPEED_HIGH: high speed.
  *                @arg SPEED_MAX:  maximum speed.
  * @retval None
  */
void DCM_SetSpeed (st_dcm_config_t* motor,uint8_t speed);

/**
  * @brief  stops the DC motor.
  * @param  motor: pointer to the DC motor.
  * @retval None
  */
void DCM_Stop(st_dcm_config_t* motor);
#endif /* DCM_DCM_INTERFACE_H_ */
