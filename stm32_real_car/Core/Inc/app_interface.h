/*
 * app_interface.h
 *
 *  Created on: Oct 21, 2023
 *      Author: Hossam Elwahsh
 */

#ifndef INC_APP_INTERFACE_H_
#define INC_APP_INTERFACE_H_

/* Typedefs */

typedef struct
{
	BOOLEAN bool_break_lights		:	SIZE_ONE_BIT;
	BOOLEAN bool_front_lights		:	SIZE_ONE_BIT;
	BOOLEAN bool_reverse_lights		:	SIZE_ONE_BIT;
	BOOLEAN bool_left_indicator		:	SIZE_ONE_BIT;
	BOOLEAN bool_right_indicator	:	SIZE_ONE_BIT;
	BOOLEAN bool_hazard_indicator	:	SIZE_ONE_BIT;
	BOOLEAN bool_reserved_1			:	SIZE_ONE_BIT;
	BOOLEAN bool_reserved_2			:	SIZE_ONE_BIT;
}st_lighting_state_t;

typedef enum
{
	TRANSMISSION_STATE_NONE		=	ZERO	,
	TRANSMISSION_STATE_PARKING				,
	TRANSMISSION_STATE_REVERSE				,
	TRANSMISSION_STATE_NEUTRAL				,
	TRANSMISSION_STATE_DRIVE				,
	TRANSMISSION_STATE_TOTAL				,
}en_transmission_state_t;

typedef enum
{
	STEERING_STATE_NONE			=	ZERO								,
	STEERING_STATE_STRAIGHT		= APP_ESP_DATA_STEERING_STRAIGHT		,
	STEERING_STATE_LEFT			= APP_ESP_DATA_STEERING_LEFT			,
	STEERING_STATE_SHARP_LEFT	= APP_ESP_DATA_STEERING_SHARP_LEFT		,
	STEERING_STATE_RIGHT		= APP_ESP_DATA_STEERING_RIGHT			,
	STEERING_STATE_SHARP_RIGHT	= APP_ESP_DATA_STEERING_SHARP_RIGHT		,
	STEERING_STATE_TOTAL
}en_steering_state_t;

/* Saves last sent data states to prevent duplicate unnecessary sends */
typedef struct
{
	/* lighting value */
	uint32_t u32_lighting_val;

	/* throttle value */
	uint8_t u8_throttle_val;

	/* lighting value */
	st_lighting_state_t st_lighting_state;

	/* transmission state */
	en_transmission_state_t en_transmission_state;

	/* steering state */
	en_steering_state_t en_steering_state;

}st_last_data_state_t;

typedef struct
{
	/* byte 1 */
	uint32_t u32_unused_bit0:1;
	uint32_t u32_unused_bit1:1;
	uint32_t u32_unused_bit2:1;
	uint32_t u32_unused_bit3:1;
	uint32_t u32_unused_bit4:1;
	uint32_t u32_unused_bit5:1;
	uint32_t u32_unused_bit6:1;
	uint32_t u32_unused_bit7:1;

	/* byte 2 */
	uint32_t u32_bit8_left_indicator:1;
	uint32_t u32_unused_bit9:1;
	uint32_t u32_unused_bit10:1;
	uint32_t u32_unused_bit11:1;
	uint32_t u32_unused_bit12:1;
	uint32_t u32_unused_bit13:1;
	uint32_t u32_unused_bit14:1;
	uint32_t u32_unused_bit15:1;


	/* byte 3 */
	uint32_t u32_unused_bit16:1;
	uint32_t u32_unused_bit17:1;
	uint32_t u32_bit18_right_indicator:1;
	uint32_t u32_unused_bit19:1;
	uint32_t u32_unused_bit20:1;
	uint32_t u32_bit21_brake_lights:1;
	uint32_t u32_unused_bit22:1;
	uint32_t u32_unused_bit23:1;

	/* byte 4 */
	uint32_t u32_unused_bit24:1;
	uint32_t u32_unused_bit25:1;
	uint32_t u32_unused_bit26:1;
	uint32_t u32_unused_bit27:1;
	uint32_t u32_unused_bit28:1;
	uint32_t u32_unused_bit29:1;
	uint32_t u32_unused_bit30:1;
	uint32_t u32_unused_bit31:1;
}st_lighting_bits_t;


/* Converter union for RxData */
typedef union
{
	uint8_t 	RxData[APP_RX_DATA_LENGTH]	;	// Receive buffer
	uint8_t		u8_rxNumber					;
	uint16_t 	u16_rxNumber				;
	uint32_t 	u32_rxNumber				;
	st_lighting_bits_t u8_lighting_bits		;
}un_data_converter_t;


typedef struct
{
	uint8_t 			item_id				;
	un_data_converter_t un_data_converter	;
}st_uart_queue_item_t;


#endif /* INC_APP_INTERFACE_H_ */