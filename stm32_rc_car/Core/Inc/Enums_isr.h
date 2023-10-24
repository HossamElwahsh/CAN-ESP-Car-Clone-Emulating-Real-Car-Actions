/*
 * Enums_isr.h
 *
 *  Created on: Oct 23, 2023
 *      Author: A M
 */

#ifndef INC_ENUMS_ISR_H_
#define INC_ENUMS_ISR_H_


typedef enum {
    Straight = 0xE0,
    right = 0xE2,
    sharp_right = 0xE3,
    left = 0xE8,
    sharp_left = 0xEC
} Steering_en;

typedef enum {
    brake_lights_off = 0x80,
    brake_lights_on = 0x81,
    right_indicators_off = 0x90,
    right_indicators_on = 0x91,
    left_indicators_off = 0xA0,
    left_indicators_on = 0xA1,
    hazard_indicators_off = 0xB0,
    hazard_indicators_on = 0xB1,
    front_lights_off = 0xC0,
    front_lights_on = 0xC1,
    reverse_lights_off = 0xD0,
    reverse_lights_on = 0xD1
} lights_en;


typedef enum {
    throttle_0_percent = 0x10,
    throttle_70_percent = 0x11,
    throttle_80_percent = 0x12,
    throttle_90_percent = 0x13,
    throttle_100_percent = 0x14
} throttle_en;



#endif /* INC_ENUMS_ISR_H_ */
