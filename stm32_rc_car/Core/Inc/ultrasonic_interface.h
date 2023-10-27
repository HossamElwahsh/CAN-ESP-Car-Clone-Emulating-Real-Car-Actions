#ifndef ULTRASONIC_INTERFACE_H_
#define ULTRASONIC_INTERFACE_H_

#include "stm32f1xx_hal.h"
#include "ultrasonic_confg.h"
#include "app_interface.h"

#define time_out      2000000U

typedef enum{
	
	SUCCESS_=9,
	TIME_OUT_
}ULTRASONIC_READ;

void Ultrasonic_getdistance(transmission_en  transmission_Ultra);

#endif //ULTRASONIC_INTERFACE_H_
