/*
 * Debug.c
 *
 *  Created on: 28.04.2014
 *      Author: ed
 */
#include "Debug.h"

// test/debug variables
int gTestVar0, gTestVar1, gTestVar2, gTestVar3, gTestVar4, gTestVar5, gTestVar6, gTestVar7, gTestVar8, gTestVar9;

bool isRAMDebuggingEnabled = FALSE;
int ramDebugCounter = 0;
int ramMeasureMode = RAM_DEBUG_MEASURE_TORQUE;
int ramMeasureDelay = 1;
int actualDelay = 0;

u8 triggerType = TRIGGER_ALWAYS;
int triggerValue = 0;
bool waitingForTrigger = FALSE;

RamBufferElement ramBuffer[RAM_BUFFER_ELEMENTS];

int debug_getTestVar0()
{
	return gTestVar0;
}

int debug_getTestVar1()
{
	return gTestVar1;
}

int debug_getTestVar2()
{
	return gTestVar2;
}

int debug_getTestVar3()
{
	return gTestVar3;
}

int debug_getTestVar4()
{
	return gTestVar4;
}

int debug_getTestVar5()
{
	return gTestVar5;
}

int debug_getTestVar6()
{
	return gTestVar6;
}

int debug_getTestVar7()
{
	return gTestVar7;
}

int debug_getTestVar8()
{
	return gTestVar8;
}

int debug_getTestVar9()
{
	return gTestVar9;
}

void debug_setTestVar0(int value)
{
	gTestVar0 = value;
}

void debug_setTestVar1(int value)
{
	gTestVar1 = value;
}

void debug_setTestVar2(int value)
{
	gTestVar2 = value;
}

void debug_setTestVar3(int value)
{
	gTestVar3 = value;
}

void debug_setTestVar4(int value)
{
	gTestVar4 = value;
}

void debug_setTestVar5(int value)
{
	gTestVar5 = value;
}

void debug_setTestVar6(int value)
{
	gTestVar6 = value;
}

void debug_setTestVar7(int value)
{
	gTestVar7 = value;
}

void debug_setTestVar8(int value)
{
	gTestVar8 = value;
}

void debug_setTestVar9(int value)
{
	gTestVar9 = value;
}

// == RAM debugging ===

void debug_startRAMDebugging(u8 type, s32 value)
{
	triggerType = type;
	triggerValue = value;
	isRAMDebuggingEnabled = FALSE;
	waitingForTrigger = TRUE;
	ramDebugCounter = 0;
}

void debug_stopRamDebugging()
{
	waitingForTrigger = FALSE;
	isRAMDebuggingEnabled = FALSE;
	ramDebugCounter = 0;
}

bool debug_isRamDebuggingEnabled()
{
	return isRAMDebuggingEnabled;
}

bool debug_isWaitingForTrigger()
{
	return waitingForTrigger;
}

int debug_getRamDebugCounter()
{
	return ramDebugCounter;
}

void debug_incRamDebugCounter()
{
	ramDebugCounter++;

	// reset delay
	actualDelay = 0;

	if(ramDebugCounter == (int)RAM_BUFFER_ELEMENTS)
		debug_stopRamDebugging();
}

void debug_addRamDebugElement(RamBufferElement element)
{
	actualDelay++;
	if(actualDelay >= ramMeasureDelay)
	{
		if((isRAMDebuggingEnabled) && (ramDebugCounter < RAM_BUFFER_ELEMENTS))
		{
			int j;
			for(j = 0; j < ITEMS_PER_ELEMENT; j++)
			{
				ramBuffer[ramDebugCounter].item[j] = element.item[j];
			}
			debug_incRamDebugCounter();
		}
	}
}

int debug_getRamBufferElements()
{
	return RAM_BUFFER_ELEMENTS;
}

int debug_getRamBufferSize()
{
	return sizeof(ramBuffer);
}

int debug_getRamBufferElementItems()
{
	return ITEMS_PER_ELEMENT;
}

int debug_getRamBufferItem(int elementIndex, int valueIndex, int *value)
{
	if(elementIndex >= RAM_BUFFER_ELEMENTS)
		return 1;
	if(valueIndex >= ITEMS_PER_ELEMENT)
		return 2;

	*value = ramBuffer[elementIndex].item[valueIndex];
	return 0;
}

void debug_setRamMeasureMode(int mode)
{
	switch(mode)
	{
	case 0:
		ramMeasureMode = RAM_DEBUG_MEASURE_TORQUE;
		break;
	case 1:
		ramMeasureMode = RAM_DEBUG_MEASURE_VELOCITY;
		break;
	case 2:
		ramMeasureMode = RAM_DEBUG_MEASURE_POSITION;
		break;
	case 3:
		ramMeasureMode = RAM_DEBUG_MEASURE_DEFINED;
		break;
	case 4:
	case 5:
		ramMeasureMode = mode; // allow different unknown modes
		break;
	default:
		ramMeasureMode = RAM_DEBUG_MEASURE_TORQUE;
		break;
	}
}

int debug_getRamMeasureMode()
{
	return ramMeasureMode;
}

void debug_setRamMeasureDelay(int delay)
{
	ramMeasureDelay = delay;
}

int debug_getRamMeasureDelay()
{
	return ramMeasureDelay;
}

void debug_checkTriggerCondition(s32 torque, s32 velocity, s32 position)
{
	switch(triggerType)
	{
	case TRIGGER_ALWAYS:
		isRAMDebuggingEnabled = TRUE;
		break;
	case TRIGGER_POSITION_LT:
		if(position < triggerValue)
			isRAMDebuggingEnabled = TRUE;
		break;
	case TRIGGER_POSITION_GT:
		if(position > triggerValue)
			isRAMDebuggingEnabled = TRUE;
		break;
	case TRIGGER_VELOCITY_LT:
		if(velocity < triggerValue)
			isRAMDebuggingEnabled = TRUE;
		break;
	case TRIGGER_VELOCITY_GT:
		if(velocity > triggerValue)
			isRAMDebuggingEnabled = TRUE;
		break;
	case TRIGGER_TORQUE_LT:
		if(torque < triggerValue)
			isRAMDebuggingEnabled = TRUE;
		break;
	case TRIGGER_TORQUE_GT:
		if(torque > triggerValue)
			isRAMDebuggingEnabled = TRUE;
		break;
	default:	// default behaves the same way as TRIGGER_ALWAYS
		isRAMDebuggingEnabled = TRUE;
		break;
	}

	if(isRAMDebuggingEnabled == TRUE)
		waitingForTrigger = FALSE;
}
