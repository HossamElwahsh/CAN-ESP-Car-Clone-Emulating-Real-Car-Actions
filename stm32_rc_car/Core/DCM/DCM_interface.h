/**
 ******************************************************************************
 * @file    DCM_interface.h
 * @author  Salma Faragalla
 * @brief   Header file of DCM module.
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DCM_DCM_INTERFACE_H_
#define DCM_DCM_INTERFACE_H_

/* Includes ------------------------------------------------------------------*/
#include "DCM_config.h"
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  DCM configuration structure definition
 */
typedef struct
{
  GPIO_TypeDef *DCM_GPIOx;
  uint16_t DCM_IN1_GPIO_PIN_x;
  uint16_t DCM_IN2_GPIO_PIN_x;
  TIM_HandleTypeDef *DCM_htimx;
  uint32_t DCM_TIM_CHANNEL_x;

} st_dcm_config_t;

/* Exported functions --------------------------------------------------------*/

/* DCM Forward Functions -----------------------------------------------------*/

/**
 * @brief  moves the motor forward.
 * @param  motor: pointer to the motor configuration structure.
 * @retval None
 */
void DCM_MoveForward(st_dcm_config_t *motor);

/**
 * @brief  moves the car to the right in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

/**
 * @brief  moves the car to the left in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

/**
 * @brief  moves the car to the right sharply in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightSharpForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

/**
 * @brief  moves the car to the left sharply in the forward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftSharpForward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

/* DCM Backward Functions ----------------------------------------------------*/
/**
 * @brief  moves the motor backward.
 * @param  motor: pointer to the motor configuration structure.
 * @retval None
 */
void DCM_MoveBackward(st_dcm_config_t *motor);

/**
 * @brief  moves the car to the right in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

/**
 * @brief  moves the car to the left in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

/**
 * @brief  moves the car to the right sharply in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveRightSharpBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

/**
 * @brief  moves the car to the left sharply in the backward direction.
 * @param  motor_left: pointer to the left motor configuration structure.
 * @param  motor_right: pointer to the right motor configuration structure.
 * @retval None
 */
void DCM_MoveLeftSharpBackward(st_dcm_config_t *motor_left, st_dcm_config_t *motor_right);

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
void DCM_SetSpeed(st_dcm_config_t *motor, uint8_t speed);

/**
 * @brief  stops the motor.
 * @param  motor: pointer to the motor configuration structure.
 * @retval None
 */
void DCM_Stop(st_dcm_config_t *motor);

#endif /* DCM_DCM_INTERFACE_H_ */
