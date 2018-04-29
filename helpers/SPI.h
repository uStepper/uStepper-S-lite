/*
 * SPI.h
 *
 *  Created on: 30.09.2016
 *      Author: ed
 */

#ifndef API_SPI_H
#define API_SPI_H

	#include "TypeDefs.h"

	// For 16 bit channels
	#define BIT_0_TO_15   0
	#define BIT_16_TO_31  1

	typedef struct
	{
		u8 address;
		union
		{
			int32 int32;
			u16 word[2];
			u8 byte[4];
		} value;
	} TDatagram;

#endif /* API_SPI_H */
