/*
 * TMC2208.h
 *
 *  Created on: 07.07.2017
 *      Author: LK
 */

#ifndef API_IC_TMC2208_H
#define API_IC_TMC2208_H

	#include "../../helpers/API_Header.h"
	#include "TMC2208_Register.h"

	#define REGISTER_COUNT 128

	// Usage note: use 1 TypeDef per IC (LK)
	typedef struct {
		int velocity;
		int oldX;
		uint32 oldTick;
		int32 registerResetState[REGISTER_COUNT];
		uint8 registerAccess[REGISTER_COUNT];
		//bool vMaxModified;
	} TMC2208TypeDef;

	void tmc2208_initConfig(TMC2208TypeDef *TMC2208);
	void tmc2208_periodicJob(u8 motor, uint32 tick, TMC2208TypeDef *TMC2208, ConfigurationTypeDef *TMC2208_config);
	u8 tmc2208_reset(ConfigurationTypeDef *TMC2208_config);
	u8 tmc2208_restore(ConfigurationTypeDef *TMC2208_config);

#endif /* API_IC_TMC2208_H */
