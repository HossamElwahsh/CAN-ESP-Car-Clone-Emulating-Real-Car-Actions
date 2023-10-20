/* @filename 	: 	std.h
 * @brief		:	Holds standard typedefs/macros/etc. and helping macros
 * @author		:	Hossam Elwahsh
 *
 * */

#ifndef INC_STD_H_
#define INC_STD_H_

typedef unsigned char boolean;

#define STATIC static
#define BOOLEAN boolean

#define TRUE 	(1)
#define FALSE 	(0)
#define ZERO	(0)
#define NULL_PTR ((void *)0)
#define MOD(val, mod_with) (val % mod_with)
#define BYTE_MAX_VAL 0xFF
#define UINT32_MAX_VAL 0xFFFFFFFF

#define MAX_PERCENTAGE 						100UL
#define THROTTLE_READING_REDUCTION_FACTOR 	250 * 64UL

/* LIGHTS MAPPED BYTES */


#endif /* INC_STD_H_ */
