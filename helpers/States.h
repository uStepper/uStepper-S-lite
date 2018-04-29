/*
 * States.h
 *
 *  Created on: 29.09.2016
 *      Author: ed
 */

#ifndef API_STATES_H_
#define API_STATES_H_

	typedef enum {
		DRIVER_DISABLE,
		DRIVER_ENABLE,
		DRIVER_USE_GLOBAL_ENABLE
	} DriverState;

	typedef enum {
		CONFIG_READY,
		CONFIG_RESET,
		CONFIG_RESTORE
	} ConfigState;

#endif /* API_STATES_H_ */
