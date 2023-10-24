#ifndef ULTRASONIC_TASK_H_
#define ULTRASONIC_TASK_H_

#include "ultrasonic_interface.h"

typedef enum{
	Green_Flag,
	Red_Flag
}ULTRASONIC_PASS;

void Ultrasonic_Task (void*pvParameter );
void Ultrasonictest_Task(void*pvParameter);
void delay_us (uint16_t us);

#endif //ULTRASONIC_TASK_H_
