/*
 * Debug.h
 *
 *  Created on: 28.04.2014
 *      Author: ed
 */

#ifndef DEBUG_H
#define DEBUG_H

	#include "TypeDefs.h"

	#define RAM_BUFFER_ELEMENTS	1000
	#define ITEMS_PER_ELEMENT	4

	// getter/setter for test variables
	int debug_getTestVar0();
	int debug_getTestVar1();
	int debug_getTestVar2();
	int debug_getTestVar3();
	int debug_getTestVar4();
	int debug_getTestVar5();
	int debug_getTestVar6();
	int debug_getTestVar7();
	int debug_getTestVar8();
	int debug_getTestVar9();

	void debug_setTestVar0(int value);
	void debug_setTestVar1(int value);
	void debug_setTestVar2(int value);
	void debug_setTestVar3(int value);
	void debug_setTestVar4(int value);
	void debug_setTestVar5(int value);
	void debug_setTestVar6(int value);
	void debug_setTestVar7(int value);
	void debug_setTestVar8(int value);
	void debug_setTestVar9(int value);

	typedef struct
	{
		s32 item[ITEMS_PER_ELEMENT];
	} RamBufferElement;

	#define RAM_DEBUG_MEASURE_TORQUE    0
	#define RAM_DEBUG_MEASURE_VELOCITY  1
	#define RAM_DEBUG_MEASURE_POSITION  2
	#define RAM_DEBUG_MEASURE_DEFINED   3

	#define TRIGGER_ALWAYS       0
	#define TRIGGER_POSITION_LT  1
	#define TRIGGER_POSITION_GT  2
	#define TRIGGER_VELOCITY_LT  3
	#define TRIGGER_VELOCITY_GT  4
	#define TRIGGER_TORQUE_LT    5
	#define TRIGGER_TORQUE_GT    6

	void debug_startRAMDebugging(u8 triggerType, s32 triggerValue);
	void debug_stopRamDebugging();
	bool debug_isRamDebuggingEnabled();
	bool debug_isWaitingForTrigger();
	void debug_checkTriggerCondition(s32 torque, s32 velocity, s32 position);

	int debug_getRamDebugCounter();
	void debug_incRamDebugCounter();

	int debug_getRamMeasureMode();
	void debug_setRamMeasureMode(int mode);

	int debug_getRamMeasureDelay();
	void debug_setRamMeasureDelay(int delay);

	void debug_addRamDebugElement(RamBufferElement element);
	int debug_getRamBufferItem(int elementIndex, int valueIndex, int *value);

	int debug_getRamBufferElements();
	int debug_getRamBufferSize();
	int debug_getRamBufferElementItems();

#endif /* DEBUG_H_ */
