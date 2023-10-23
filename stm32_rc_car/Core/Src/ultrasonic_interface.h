#ifndef ULTRASONIC_INTERFACE_H_
#define ULTRASONIC_INTERFACE_H_

#include "stm32f1xx_hal.h"
#include "ultrasonic_confg.h"
#define time_out      10000000U

typedef enum{
	
	SUCCESS_=9,
	TIME_OUT_
}ULTRASONIC_READ;

typedef enum{
	Neutral =0X70,
	Reverse=0X77,
	Drive=0x71,
	Parking=0x7F
}transmission_en;

ULTRASONIC_READ Ultrasonic_getdistance(transmission_en  transmission_Ultra,uint16_t* DISTANCE);

#endif //ULTRASONIC_INTERFACE_H_
