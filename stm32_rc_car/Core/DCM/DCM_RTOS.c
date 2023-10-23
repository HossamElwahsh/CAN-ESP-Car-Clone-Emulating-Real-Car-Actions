/*
 * DCM_RTOS.c
 *
 *  Created on: Oct 23, 2023
 *      Author: Salma
 */

#include "DCM_RTOS.h"

/* DC motors objects */
st_dcm_config_t DCM_Left  = {DCM_LEFT_PORT,DCM_LEFT_IN1,DCM_LEFT_IN2,&htim3,TIM_CHANNEL_2};
st_dcm_config_t DCM_Right = {DCM_RIGHT_PORT,DCM_RIGHT_IN1,DCM_RIGHT_IN2,&htim3,TIM_CHANNEL_1};


void DCM_CheckSpeed(void)
{
    switch (THROTTLE_POSITION)
    {
    case SPEED_ZERO:
    {
        DCM_SetSpeed(&DCM_Left, SPEED_ZERO);
        DCM_SetSpeed(&DCM_Right, SPEED_ZERO);
        break;
    }
    case SPEED_MIN:
    {
        DCM_SetSpeed(&DCM_Left, SPEED_MIN);
        DCM_SetSpeed(&DCM_Right, SPEED_MIN);
        break;
    }
    case SPEED_MED:
    {
        DCM_SetSpeed(&DCM_Left, SPEED_MED);
        DCM_SetSpeed(&DCM_Right, SPEED_MED);
        break;
    }
    case SPEED_HIGH:
    {
        DCM_SetSpeed(&DCM_Left, SPEED_HIGH);
        DCM_SetSpeed(&DCM_Right, SPEED_HIGH);
        break;
    }
    case SPEED_MAX:
    {
        DCM_SetSpeed(&DCM_Left, SPEED_MAX);
        DCM_SetSpeed(&DCM_Right, SPEED_MAX);
        break;
    }
    default:
    {
        /*Do Nothing*/
        break;
    }
    }
}

void DCM_CheckSteeringForward(void)
{
    switch (STEERING_WHEEL_POSITION)
    {
    case STEERING_STRAIGHT:
    {
        DCM_MoveForward(&DCM_Left);
        DCM_MoveForward(&DCM_Right);
        break;
    }
    case STEERING_RIGHT:
    {
        DCM_MoveRightForward(&DCM_Left, &DCM_Right);
        break;
    }
    case STEERING_LEFT:
    {
        DCM_MoveLeftForward(&DCM_Left, &DCM_Right);
        break;
    }
    case STEERING_SHARP_RIGHT:
    {
        DCM_MoveRightSharpForward(&DCM_Left, &DCM_Right);
        break;
    }
    case STEERING_SHARP_LEFT:
    {
        DCM_MoveLeftSharpForward(&DCM_Left, &DCM_Right);
        break;
    }
    default:
    {
        /*Do Nothing*/
        break;
    }
    }
}
void DCM_CheckSteeringBackward(void)
{
    switch (STEERING_WHEEL_POSITION)
    {
    case STEERING_STRAIGHT:
    {
        DCM_MoveBackward(&DCM_Left);
        DCM_MoveBackward(&DCM_Right);
        break;
    }
    case STEERING_RIGHT:
    {
        DCM_MoveRightBackward(&DCM_Left, &DCM_Right);
        break;
    }
    case STEERING_LEFT:
    {
        DCM_MoveLeftBackward(&DCM_Left, &DCM_Right);
        break;
    }
    case STEERING_SHARP_RIGHT:
    {
        DCM_MoveRightSharpBackward(&DCM_Left, &DCM_Right);
        break;
    }
    case STEERING_SHARP_LEFT:
    {
        DCM_MoveLeftSharpBackward(&DCM_Left, &DCM_Right);
        break;
    }
    default:
    {
        /*Do Nothing*/
        break;
    }
    }
}
void Task_DCM(void *pvParameters)
{
	/* take semaphore once at init to clear it */
	xSemaphoreTake(GEAR_SEMAPHORE, portMAX_DELAY);

    for (;;)
	{
		if (xSemaphoreTake(GEAR_SEMAPHORE, portMAX_DELAY) == pdTRUE)
		{
			switch (GEAR_POSITION)
			{
				case GEAR_DRIVE:
				{
					DCM_CheckSpeed();
					DCM_CheckSteeringForward();
					break;
				}
				case GEAR_NEUTRAL:
				{
					DCM_Stop(&DCM_Left);
					DCM_Stop(&DCM_Right);
					break;
				}
				case GEAR_PARKING:
				{
					DCM_Stop(&DCM_Left);
					DCM_Stop(&DCM_Right);
					break;
				}
				case GEAR_REVERSE:
				{
					DCM_CheckSpeed();
					DCM_CheckSteeringBackward();
					break;
				}
				default:
				{
					/*Do Nothing*/
					break;
				}
			}
		}
	}
}



