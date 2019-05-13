/********************************************************************************/
/*																				*/
/* @file	types.h																*/
/*																				*/
/* @brief	Defines data types used within this project.						*/
/*																				*/
/* @author	Rick Haltmaier														*/
/*																				*/
/* @date	September 1, 2015													*/
/*																				*/
/* Copyright(c) Analog Devices, Inc.											*/
/*																				*/
/********************************************************************************/
#include <stdio.h>

#ifndef _TYPES_H_
#define _TYPES_H_

typedef enum {
	EC_OK,
	EC_DONE,
	EC_MEM_FAIL,
	EC_FAIL,
	EC_BAD_ARG,
	EC_BUSY
} errcode_t;

/* define unsigned integer types */
typedef char int8;
typedef short int16;
typedef int int32;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;

#endif // _TYPES_H_