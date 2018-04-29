#include "TMC2208.h"

/* Register access permissions:
 * 0: none (reserved)
 * 1: read
 * 2: write
 * 3: read/write
 * 7: read^write (seperate functions/values)
 */
const uint8_t tmc2208DefaultRegisterAccess[REGISTER_COUNT] PROGMEM =
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
const int32_t tmc2208DefaultRegisterResetState[REGISTER_COUNT] PROGMEM =
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

Tmc2208::Tmc2208(void)
{

}

void Tmc2208::setup(void)
{
	this->tmc2208InitConfig();
}

void Tmc2208::tmc2208InitConfig(void)
{
	this->tmc2208.velocity  = 0;
	this->tmc2208.oldTick   = 0;
	this->tmc2208.oldX      = 0;

	for(int i = 0; i < REGISTER_COUNT; i++)
	{
		this->tmc2208.registerAccess[i]      = pgm_read_byte(&tmc2208DefaultRegisterAccess[i]);
		this->tmc2208.registerResetState[i]  = pgm_read_dword(&tmc2208DefaultRegisterResetState[i]);
	}
}

void Tmc2208::tmc2208PeriodicJob(uint32_t tick)
{
	if(this->tmc2208Config.state != CONFIG_READY && (tick - this->tmc2208.oldTick) > 2)
	{
		tmc2208WriteConfiguration();
		this->tmc2208.oldTick = tick;
	}
}

void Tmc2208::tmc2208WriteConfiguration(void)
{
	uint8_t *ptr = &this->tmc2208Config.configIndex;
	const int32_t *settings = (this->tmc2208Config.state == CONFIG_RESTORE) ? this->tmc2208Config.shadowRegister : this->tmc2208.registerResetState;

	while((*ptr < REGISTER_COUNT) && !IS_WRITEABLE(this->tmc2208.registerAccess[*ptr]))
		(*ptr)++;

	if(*ptr < REGISTER_COUNT)
	{
		tmc2208WriteRegister(*ptr, settings[*ptr]);
		(*ptr)++;
	}
	else
	{
		this->tmc2208Config.state = CONFIG_READY;
	}
}

void Tmc2208::tmc2208WriteRegister(uint8_t address, int32_t value)
{

}

void Tmc2208::tmc2208ReadRegister(uint8_t address, int32_t *value)
{

}

uint8_t Tmc2208::tmc2208Reset(void)
{

}

uint8_t Tmc2208::tmc2208Restore(void)
{

}