
#include "ultrasonic_interface.h"
#include "Ultrasonic_Task.h"
#include "stm32f1xx_hal.h"
extern TIM_HandleTypeDef htim1;
extern transmission_en gl_transmission_en;
static void delay_us(uint16_t us);
static uint32_t count;
static uint16_t TIM1_ULTRA;
static uint16_t TIM2_ULTRA;
static uint8_t ULTRASONIC_FLAG;

/* Helping blocking delay function in uS */
static void delay_us (uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){

	if(ULTRASONIC_FLAG==0){
	if(gl_transmission_en ==Drive){
	TIM1_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1 );
	__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1 ,TIM_INPUTCHANNELPOLARITY_FALLING);
	}
	else if(gl_transmission_en ==Reverse){
		TIM1_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4 );
		__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_4 ,TIM_INPUTCHANNELPOLARITY_FALLING);
	}
	ULTRASONIC_FLAG=1;

	}
	else if(ULTRASONIC_FLAG==1){
     if(gl_transmission_en ==Drive){
	TIM2_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1 );

	__HAL_TIM_SET_COUNTER(htim,0);
	__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1 , TIM_INPUTCHANNELPOLARITY_RISING);
	__HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC1);
     }
     else if(gl_transmission_en ==Reverse){

    	 TIM2_ULTRA= HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4 );

    	 	__HAL_TIM_SET_COUNTER(htim,0);
    	 	__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_4 , TIM_INPUTCHANNELPOLARITY_RISING);
    	 	__HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC4);
     }

	
		ULTRASONIC_FLAG=2;

	}

}

ULTRASONIC_READ Ultrasonic_getdistance(transmission_en  transmission_Ultra,uint16_t* DISTANCE){
	uint16_t DIFFERNCE;
	ULTRASONIC_READ STATUS;
switch	(transmission_Ultra){
case Drive:
	HAL_GPIO_WritePin(TRIGER_FORWARD_PORT ,TRIGER_FORWARD_PIN , GPIO_PIN_SET);
	    delay_us(10);
	HAL_GPIO_WritePin(TRIGER_FORWARD_PORT , TRIGER_FORWARD_PIN , GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_CC1);
	while(ULTRASONIC_FLAG<2&&count<time_out ){
	count++;
       }
	break;
  case Reverse:
	HAL_GPIO_WritePin(TRIGER_BACKWARD_PORT , TRIGER_BACKWARD_PIN, GPIO_PIN_SET);
	    delay_us(10);
	HAL_GPIO_WritePin(TRIGER_BACKWARD_PORT ,TRIGER_BACKWARD_PIN, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim1,TIM_IT_CC4);
	while(ULTRASONIC_FLAG<2&&count<time_out ){
	      count++;
           }
	break;
    default:
	break;
}
	    if(ULTRASONIC_FLAG==2){
		if(TIM2_ULTRA>TIM1_ULTRA){
		DIFFERNCE=TIM2_ULTRA-TIM1_ULTRA;
		}
	        else{
		DIFFERNCE=(0xffff-TIM1_ULTRA)+TIM2_ULTRA;
		}
		*DISTANCE=DIFFERNCE* 0.034/2;
		ULTRASONIC_FLAG=0;
		count=0;
		STATUS=SUCCESS_;
		}
		else{
		STATUS=TIME_OUT_;
		count=0;
		}
		
return STATUS;
}
