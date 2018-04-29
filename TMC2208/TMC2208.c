/*
 * TMC2208.c
 *
 *  Created on: 07.07.2017
 *      Author: LK
 */

#include "TMC2208.h"

#define R00 0x00000041
#define R10 0x00001F00
#define R6C 0x10000053
#define R70 0xC10D0024

/* Register access permissions:
 * 0: none (reserved)
 * 1: read
 * 2: write
 * 3: read/write
 * 7: read^write (seperate functions/values)
 */
const u8 tmc2208_defaultRegisterAccess[REGISTER_COUNT] =
{
//	0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	3, 3, 1, 2, 2, 1, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, // 0x00 - 0x0F
	2, 2, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x10 - 0x1F
	0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x20 - 0x2F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x30 - 0x3F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x40 - 0x4F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0x50 - 0x5F
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 3, 0, 0, 1, // 0x60 - 0x6F
	3, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  // 0x70 - 0x7F
};
const s32 tmc2208_defaultRegisterResetState[REGISTER_COUNT] =
{
//	0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	R00, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x00 - 0x0F
	R10, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x10 - 0x1F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x20 - 0x2F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x30 - 0x3F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x40 - 0x4F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, // 0x50 - 0x5F
	0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   R6C, 0,   0,   0, // 0x60 - 0x6F
	R70, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0  // 0x70 - 0x7F
};

// => SPI wrapper
extern void tmc2208_writeRegister(uint8 address, int32 value);
extern void tmc2208_readRegister(uint8 address, int32 *value);
// <= SPI wrapper

void tmc2208_initConfig(TMC2208TypeDef *tmc2224)
{
	tmc2224->velocity  = 0;
	tmc2224->oldTick   = 0;
	tmc2224->oldX      = 0;

	for(int i = 0; i < REGISTER_COUNT; i++)
	{
		tmc2224->registerAccess[i]      = tmc2208_defaultRegisterAccess[i];
		tmc2224->registerResetState[i]  = tmc2208_defaultRegisterResetState[i];
	}
};

void tmc2208_writeConfiguration(TMC2208TypeDef *tmc2208, ConfigurationTypeDef *TMC2208_config)
{
	uint8 *ptr = &TMC2208_config->configIndex;
	const int32 *settings = (TMC2208_config->state == CONFIG_RESTORE) ? TMC2208_config->shadowRegister : tmc2208->registerResetState;

	while((*ptr < REGISTER_COUNT) && !IS_WRITEABLE(tmc2208->registerAccess[*ptr]))
		(*ptr)++;

	if(*ptr < REGISTER_COUNT)
	{
		tmc2208_writeRegister(*ptr, settings[*ptr]);
		(*ptr)++;
	}
	else
	{
		TMC2208_config->state = CONFIG_READY;
	}
}

void tmc2208_periodicJob(u8 motor, uint32 tick, TMC2208TypeDef *tmc2208, ConfigurationTypeDef *TMC2208_config)
{
	UNUSED(motor);

	if(TMC2208_config->state != CONFIG_READY && (tick - tmc2208->oldTick) > 2)
	{
		tmc2208_writeConfiguration(tmc2208, TMC2208_config);
		tmc2208->oldTick = tick;
	}
}

uint8 tmc2208_reset(ConfigurationTypeDef *TMC2208_config)
{
	if(TMC2208_config->state != CONFIG_READY)
		return 0;

	TMC2208_config->state        = CONFIG_RESET;
	TMC2208_config->configIndex  = 0;

	return 1;
}

uint8 tmc2208_restore(ConfigurationTypeDef *TMC2208_config)
{
	if(TMC2208_config->state != CONFIG_READY)
		return 0;

	TMC2208_config->state        = CONFIG_RESTORE;
	TMC2208_config->configIndex  = 0;

	return 1;
}
