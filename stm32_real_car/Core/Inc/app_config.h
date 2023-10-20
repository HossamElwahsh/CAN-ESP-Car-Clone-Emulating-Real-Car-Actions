/*
 * app_config.h
 *
 *  Created on: Oct 20, 2023
 *      Author: Hossam Elwahsh
 */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

#include "std.h"

/* CAN CONFIG */
#define APP_TX_DATA_LENGTH	8
#define APP_RX_DATA_LENGTH 	8

/* CAR CAN MSG IDS */
#define APP_CAN_ID_THROTTLE 		0x1F0
#define APP_CAN_ID_LIGHTS			0x73E
#define APP_CAN_ID_TRANSMISSION		0x2EF

#endif /* INC_APP_CONFIG_H_ */
