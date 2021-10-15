/**
 * ASIC Firmware Platform Developed by ASIC Team Solution Part
 *
 * Copyright 2008-2009 by System LSI, Samsung Electronics, Inc.,
 * San#24 Nongseo-Dong, Giheung-Gu Yongin-City, Gyeonggi-Do, Korea.
 * All rights reserved.
 *
 * This software is the confidential and proprietary information
 * of Samsung Electronics, Inc. ("Confidential Information").  You
 * shall not disclose such Confidential Information and shall use
 * it only in accordance with the terms of the license agreement
 * you entered into with Samsung.
 */
/**
 * @file		types.h
 * @brief		ASICFramework type declarations
 * @version		v0.0.1 2009/09/05 by Jang Tae Su (taesu.jang@samsung.com)
 *				- Created.
 */

#ifndef __TYPES_H__
#define __TYPES_H__

/*
	Type definition
*/
typedef void					VOID;

typedef unsigned char				BOOL;	/* Logical data type (TRUE or FALSE) */

typedef char					CHAR;	/* char */
typedef unsigned short				WCHAR; 	/* wide char */

typedef signed char				INT8;
typedef unsigned char				UINT8;	/* Unsigned 8 bit value */
typedef signed short				INT16;	/* Signed 16 bit value */
typedef unsigned short				UINT16;	/* Unsigned 16 bit value */
typedef signed long				INT32;	/* Signed 32 bit value ,  same to  signed int	*/
typedef unsigned long				UINT32;	/* Unsigned 32 bit value */
typedef signed long long			INT64;	/* signed 64 bit value (if available)*/
typedef unsigned long long			UINT64;	/* Unsigned 64 bit value (if available)*/

typedef float					FLOAT;	/* 32 bit, single prec. floating-point */
typedef double					DOUBLE;	/* 64 bit, double prec. floating-point */

/*
	Key string definition
*/
typedef void*					HANDLE;

#define TRUE 					1
#define FALSE 					0

#define ENABLE					1
#define DISALBE					0

#define SUCCESS					1
#define ERROR					0

#ifndef NULL
#define NULL					0
#endif

/*
	Macro definition to access hardware registers
*/
#define HW_REG(base, offset)			(*(volatile UINT32 *)(base + (offset)))
#define HW_REGA(base, offset, index)		(*(volatile UINT32 *)((base) + (offset) + (index << 2)))

#define HW_REG8(base, offset)			(*(volatile UINT8 *)(base + (offset)))
#define HW_REG16(base, offset)			(*(volatile UINT16 *)(base + (offset)))
#define HW_REG32(base, offset)			(*(volatile UINT32 *)(base + (offset)))

#endif /* __TYPES_H__ */
