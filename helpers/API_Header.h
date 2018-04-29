/*
 * tmc_header.h
 *
 *  Created on: 29.09.2016
 *      Author: ed
 */

#ifndef API_TMC_HEADER_H_
#define API_TMC_HEADER_H_

	#include "TypeDefs.h"
	#include "States.h"
	#include "Bits.h"
	#include "SPI.h"
	#include "Debug.h"
	#include "RegisterAccess.h"
	#include <stdlib.h> // todo CHECK 2: As far as i've seen we only use abs() from this - maybe define that ourselves to remove dependency? (LH)

	// structure for configuration mechanism
	typedef struct
	{
		ConfigState       state;
		uint8             configIndex;
		int32             shadowRegister[128];
		uint8 (*reset)    (void);
		uint8 (*restore)  (void);
	} ConfigurationTypeDef;

#endif /* API_TMC_HEADER_H_ */
