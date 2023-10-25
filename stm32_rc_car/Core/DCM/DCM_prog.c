/**
 ******************************************************************************
 * @file    DCM_prog.c
 * @author  Salma Faragalla
 * @brief   DCM module driver.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "DCM_interface.h"

/* Exported functions ---------------------------------------------------------*/

/* DCM Forward Functions -----------------------------------------------------*/
/**
 * @brief  moves the motor forward.
 * @param  motor: pointer to the motor configuration structure.
 * @retval None
 */
void DCM_MoveForward(st_dcm_config_t *motor)
{
	HAL_GPIO_WritePin(motor->DCM_GPIOx, motor->DCM_IN1_GPIO_PIN_x, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor->DCM_GPIOx, motor->DCM_IN2_GPIO_PIN_x, GPIO_PIN_RESET);
}

/**
 * @brief  moves the car to the right in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveForward(motor_left);
	DCM_Stop(motor_right);
}

/**
 * @brief  moves the car to the left in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveForward(motor_right);
	DCM_Stop(motor_left);
}

/**
 * @brief  moves the car to the right sharply in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightSharpForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveForward(motor_left);
	DCM_MoveBackward(motor_right);
}

/**
 * @brief  moves the car to the left sharply in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftSharpForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveForward(motor_right);
	DCM_MoveBackward(motor_left);
}

/* DCM Backward Functions ----------------------------------------------------*/
/**
 * @brief  moves the motor backward.
 * @param  motor: pointer to the motor configuration structure.
 * @retval None
 */
void DCM_MoveBackward(st_dcm_config_t *motor)
{
	HAL_GPIO_WritePin(motor->DCM_GPIOx, motor->DCM_IN1_GPIO_PIN_x, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->DCM_GPIOx, motor->DCM_IN2_GPIO_PIN_x, GPIO_PIN_SET);
}

/**
 * @brief  moves the car to the right in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveBackward(motor_left);
	DCM_Stop(motor_right);
}

/**
 * @brief  moves the car to the left in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveBackward(motor_right);
	DCM_Stop(motor_left);
}

/**
 * @brief  moves the car to the right sharply in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightSharpBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveBackward(motor_left);
	DCM_MoveForward(motor_right);
}

/**
 * @brief  moves the car to the left sharply in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftSharpBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right)
{
	DCM_MoveBackward(motor_right);
	DCM_MoveForward(motor_left);
}

/* Other DCM Functions -------------------------------------------------------*/
/**
 * @brief  sets the speed of the motor.
 * @param  motor: pointer to the motor configuration structure.
 * @param  speed: desired speed of the motor.
 *    	    	  This parameter can be one of the following values:
 *                @arg SPEED_ZERO: speed is zero.
 *                @arg SPEED_MIN:  minimum speed.
 *                @arg SPEED_MED:  medium speed.
 *                @arg SPEED_HIGH: high speed.
 *                @arg SPEED_MAX:  maximum speed.
 * @retval None
 */
void DCM_SetSpeed(st_dcm_config_t *motor, uint8_t speed)
{
	__HAL_TIM_SET_COMPARE((motor->DCM_htimx), motor->DCM_TIM_CHANNEL_x, speed);
	HAL_TIM_PWM_Start(motor->DCM_htimx, motor->DCM_TIM_CHANNEL_x);
}

/**
 * @brief  stops the motor.
 * @param  motor: pointer to the motor configuration structure.
 * @retval None
 */
void DCM_Stop(st_dcm_config_t *motor)
{
	HAL_GPIO_WritePin(motor->DCM_GPIOx, motor->DCM_IN1_GPIO_PIN_x, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor->DCM_GPIOx, motor->DCM_IN2_GPIO_PIN_x, GPIO_PIN_RESET);
}
