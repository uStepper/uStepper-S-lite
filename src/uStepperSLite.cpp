/********************************************************************************************
* 	 	File: 		uStepper.cpp															*
*		Version:    1.3.0                                           						*
*      	date: 		January 10th, 2018 	                                    				*
*      	Author: 	Thomas Hørring Olsen                                   					*
*                                                   										*	
*********************************************************************************************
*			            uStepper class 					   									*
* 																							*
*	This file contains the implementation of the class methods, incorporated in the  		*
*	uStepper arduino library. The library is used by instantiating an uStepper object 		*
*	by calling either of the two overloaded constructors: 									*
*																							*
*		example:																			*
*																							*
*		uStepper stepper; 																	*
*																							*
*		OR 																					*
*																							*
*		uStepper stepper(500, 2000);														*
*																							*
*	The first instantiation above creates a uStepper object with default acceleration 		*
*	and maximum speed (1000 steps/s^2 and 1000steps/s respectively).						*
*	The second instantiation overwrites the default settings of acceleration and 			*
*	maximum speed (in this case 500 steps/s^2 and 2000 steps/s, respectively);				*
*																							*
*	after instantiation of the object, the object setup function should be called within 	*
*	arduino's setup function:																*
*																							*
*		example:																			*
*																							*
*		uStepper stepper;																	*
*																							*
*		void setup()																		*
*		{																					*
*			stepper.setup();																*
*		} 																					*
*																							*
*		void loop()																			*
*		{																					*
*																							*
*		}																					*
*																							*
*	After this, the library is ready to control the motor!									*
*																							*
*********************************************************************************************
*	(C) 2018																				*
*																							*
*	uStepper ApS																			*
*	www.ustepper.com 																		*
*	administration@ustepper.com 															*
*																							*
*	The code contained in this file is released under the following open source license:	*
*																							*
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International			*
* 																							*
* 	The code in this file is provided without warranty of any kind - use at own risk!		*
* 	neither uStepper ApS nor the author, can be held responsible for any damage				*
* 	caused by the use of the code contained in this file ! 									*
*                                                                                           *
********************************************************************************************/
/**
 * @file uStepper.cpp
 * @brief      Class implementations for the uStepper library
 *
 *             This file contains the implementations of the classes defined in
 *             uStepper.h
 *
 * @author     Thomas Hørring Olsen (thomas@ustepper.com)
 */
#include <uStepperSLite.h>
#include <math.h>

uStepperSLite *pointer;
volatile int32_t *p __attribute__((used));
volatile float debugOutput[5] = {0.0};
volatile uint32_t debugStepCounter = 0;
volatile uint32_t *pp __attribute__((used));
i2cMaster I2C(1);

extern "C" {

	void interrupt1(void)
	{
		if(PIND & 0x04)
		{
			PORTD |= (1 << 4);
		}
		else
		{
			PORTD &= ~(1 << 4);
		}
	}

	void interrupt0(void)
	{
		if(PIND & 0x04)
		{
			PORTD |= (1 << 4);
		}
		else
		{
			PORTD &= ~(1 << 4);
		}
		if((PINB & (0x08)))			//CCW
		{
			if(pointer->control == 0)
			{
				PORTB |= (1 << 2);	//Set dir to CCW
				PORTD |= (1 << 7);		//generate step pulse
				PORTD &= ~(1 << 7);		//pull step pin low again
			}		
			pointer->stepCnt--;				//DIR is set to CCW, therefore we subtract 1 step from step count (negative values = number of steps in CCW direction from initial postion)
		}
		else						//CW
		{
			if(pointer->control == 0)
			{
				
				PORTB &= ~(1 << 2);	//Set dir to CW
				PORTD |= (1 << 7);		//generate step pulse
				PORTD &= ~(1 << 7);		//pull step pin low again
			}
			pointer->stepCnt++;			//DIR is set to CW, therefore we add 1 step to step count (positive values = number of steps in CW direction from initial postion)	
		}
	}

	void PCINT2_vect(void)
	{
		asm volatile("push r16 \n\t");
		asm volatile("in r16,0x3F \n\t");
		asm volatile("push r16 \n\t");
		asm volatile("push r17 \n\t");
		asm volatile("push r30 \n\t");
		asm volatile("push r31 \n\t");

		asm volatile("lds r30,pp \n\t");
		asm volatile("lds r31,pp+1 \n\t");

		asm volatile("ldi r17,1 \n\t");

		asm volatile("ldd r16,z+0 \n\t");
		asm volatile("add r16,r17 \n\t");
		asm volatile("std z+0,r16 \n\t");
		
		asm volatile("brcc NOCARRY \n\t");

		asm volatile("ldd r16,z+1 \n\t");
		asm volatile("add r16,r17 \n\t");
		asm volatile("std z+1,r16 \n\t");

		asm volatile("brcc NOCARRY \n\t");

		asm volatile("ldd r16,z+2 \n\t");
		asm volatile("add r16,r17 \n\t");
		asm volatile("std z+2,r16 \n\t");

		asm volatile("brcc NOCARRY \n\t");

		asm volatile("ldd r16,z+3 \n\t");
		asm volatile("add r16,r17 \n\t");
		asm volatile("std z+3,r16 \n\t");

asm volatile("NOCARRY: \n\t");

		asm volatile("pop r31 \n\t");
		asm volatile("pop r30 \n\t");
		asm volatile("pop r17 \n\t");
		asm volatile("pop r16 \n\t");
		asm volatile("out 0x3F,r16 \n\t");
		asm volatile("pop r16 \n\t");	
		asm volatile("reti \n\t");
		//debugStepCounter++;
	}

	void TIMER2_COMPA_vect(void)
	{	
		asm volatile("push r16 \n\t");
		asm volatile("in r16,0x3F \n\t");
		asm volatile("push r16 \n\t");
		asm volatile("push r30 \n\t");
		asm volatile("push r31 \n\t");
		asm volatile("lds r30,p \n\t");
		asm volatile("lds r31,p+1 \n\t");		

		asm volatile("subi r30,79 \n\t");
		asm volatile("sbci r31,0 \n\t");

		asm volatile("jmp _AccelerationAlgorithm \n\t");	//Execute the acceleration profile algorithm
	}

	void TIMER1_COMPA_vect(void)
	{
		uint8_t data[2];
		uint16_t curAngle;
		int16_t deltaAngle;
		sei();

		if(I2C.getStatus() != I2CFREE)
		{
			return;
		}

		TIMSK1 &= ~(1 << OCIE1A);
		I2C.read(ENCODERADDR, ANGLE, 2, data);
		TIMSK1 |= (1 << OCIE1A);
		curAngle = (((uint16_t)data[0]) << 8 ) | (uint16_t)data[1];
		pointer->encoder.angle = curAngle;
		curAngle -= pointer->encoder.encoderOffset;
		if(curAngle > 4095)
		{
			curAngle -= 61440;
		}

		deltaAngle = (int16_t)pointer->encoder.oldAngle - (int16_t)curAngle;

		if(deltaAngle < -2047)
		{
			deltaAngle += 4096;
		}
		
		else if(deltaAngle > 2047)
		{
			deltaAngle -= 4096;
		}

		pointer->encoder.angleMoved += deltaAngle;

		pointer->encoder.curSpeed *= 0.999;
		pointer->encoder.curSpeed += 0.001 * (ENCODERINTFREQ * (float)deltaAngle) * 0.01875;

		pointer->encoder.oldAngle = curAngle;

		if(pointer->mode == DROPIN || pointer->mode == PID)
		{
			pointer->pid(deltaAngle);
			pointer->detectStall((float)deltaAngle, pointer->getMotorState());
		}
		else
		{
			pointer->detectStall((float)deltaAngle, pointer->getMotorState());
		}
	}
}

float2::float2(void)
{

}

float float2::getFloatValue(void)
{
	union
	{
		float f;
		uint32_t i;
	} a;

	a.i = (uint32_t)(this->value >> 25);

	return a.f;
}

uint64_t float2::getRawValue(void)
{
	return this->value;
}

void float2::setValue(float val)
{
	union
	{
		float f;
		uint32_t i;
	} a;

	a.f = val;

	this->value = ((uint64_t)a.i) << 25;
}

bool float2::operator<=(const float &value)
{
	if(this->getFloatValue() > value)
	{
		return 0;
	}

	if(this->getFloatValue() == value)
	{
		if((this->value & 0x0000000000007FFF) > 0)
		{
			return 0;
		}
	}

	return 1;
}

bool float2::operator<=(const float2 &value)
{
	if((this->value >> 56) > (value.value >> 56))				// sign bit of "this" bigger than sign bit of "value"?
	{
		return 1;												//"This" is negative while "value" is not. ==> "this" < "value"
	}

	if((this->value >> 56) == (value.value >> 56))				//Sign bit of "this" == sign bit of "value"?
	{
		if( (this->value >> 48) < (value.value >> 48) )			//Exponent of "this" < exponent of "value"?
		{
			return 1;											//==> "this" is smaller than "value"
		}

		if( (this->value >> 48) == (value.value >> 48) )		//Exponent of "this" == exponent of "value"?
		{
			if((this->value & 0x0000FFFFFFFFFFFF) <= (value.value & 0x0000FFFFFFFFFFFF))		//mantissa of "this" <= mantissa of "value"?
			{
				return 1;										//==> "this" <= "value"
			}
		}
	}

	return 0;													//"this" > "value"
}

float2 & float2::operator=(const float &value)
{
	this->setValue(value);

	return *this;
}

float2 & float2::operator+=(const float2 &value)
{
	float2 temp = value;
	uint64_t tempMant, tempExp;
	uint8_t cnt;	//how many times should we shift the mantissa of the smallest number to add the two mantissa's

	if((this->value >> 56) == (temp.value >> 56))
	{
		if(*this <= temp)
		{
			cnt = (temp.value >> 48) - (this->value >> 48);
			if(cnt < 48)
			{
				tempExp = (temp.value >> 48);

				this->value &= 0x0000FFFFFFFFFFFF;
				this->value |= 0x0001000000000000;
				this->value >>= cnt;

				tempMant = (temp.value & 0x0000FFFFFFFFFFFF) | 0x0001000000000000;
				tempMant += this->value;

				while(tempMant > 0x2000000000000)
				{
					tempMant >>= 1;
					tempExp++;
				}

				tempMant &= 0x0000FFFFFFFFFFFF;
				this->value = (tempExp << 48) | tempMant;
			}
			else
			{
				this->value = temp.value;
			}
		}

		else
		{
			cnt = (this->value >> 48) - (temp.value >> 48);

			if(cnt < 48)
			{
				tempExp = (this->value >> 48);

				temp.value &= 0x0000FFFFFFFFFFFF;
				temp.value |= 0x0001000000000000;
				temp.value >>= cnt;

				tempMant = (this->value & 0x0000FFFFFFFFFFFF) | 0x0001000000000000;
				tempMant += temp.value;

				while(tempMant > 0x2000000000000)
				{
					tempMant >>= 1;
					tempExp++;
				}

				tempMant &= 0x0000FFFFFFFFFFFF;
				this->value = (tempExp << 48) | tempMant;
			}
		}
	}	

	else if((this->value >> 56) == 1)
	{
		this->value &= 0x00FFFFFFFFFFFFFF;	//clear sign bit, to consider absolute value

		if(*this <= temp)
		{
			cnt = (temp.value >> 48) - (this->value >> 48);

			if(cnt < 48)
			{
				tempExp = (temp.value >> 48);

				this->value &= 0x0000FFFFFFFFFFFF;
				this->value |= 0x0001000000000000;
				this->value >>= cnt;

				tempMant = (temp.value & 0x0000FFFFFFFFFFFF) | 0x0001000000000000;

				tempMant -= this->value;

				if(tempMant > 0x8000000000000000)
				{

					tempMant &= 0x0000FFFFFFFFFFFF;
					tempExp--;
				}

				while(tempMant < 0x1000000000000)
				{
					tempMant <<= 1;
					tempExp--;
				}

				tempMant &= 0x0000FFFFFFFFFFFF;

				this->value = (tempExp << 48) | tempMant;
			}

			else
			{
				this->value = temp.value;
			}
		}

		else
		{
			cnt = (this->value >> 48) - (temp.value >> 48);
			if(cnt < 48)
			{
				tempExp = (this->value >> 48);

				temp.value &= 0x0000FFFFFFFFFFFF;
				temp.value |= 0x0001000000000000;
				temp.value >>= cnt;

				tempMant = (this->value & 0x0000FFFFFFFFFFFF) | 0x0001000000000000;

				tempMant -= temp.value;

				if(tempMant > 0x8000000000000000)
				{
					tempMant &= 0x0000FFFFFFFFFFFF;
					tempExp--;
				}

				while(tempMant < 0x1000000000000)
				{
					tempMant <<= 1;
					tempExp--;
				}

				tempMant &= 0x0000FFFFFFFFFFFF;

				this->value = (tempExp << 48) | tempMant;
				this->value |= 0x0100000000000000;				
			}
		}
	}

	else
	{
		temp.value &= 0x00FFFFFFFFFFFFFF;	//clear sign bit, to consider absolute value

		if(temp <= *this)
		{
			cnt = (this->value >> 48) - (temp.value >> 48);
			if(cnt < 48)
			{
				tempExp = (this->value >> 48);

				temp.value &= 0x0000FFFFFFFFFFFF;
				temp.value |= 0x0001000000000000;
				temp.value >>= cnt;

				tempMant = (this->value & 0x0000FFFFFFFFFFFF) | 0x0001000000000000;

				tempMant -= temp.value;

				if(tempMant > 0x8000000000000000)
				{
					tempMant &= 0x0000FFFFFFFFFFFF;
					tempExp--;
				}

				while(tempMant < 0x1000000000000)
				{
					tempMant <<= 1;
					tempExp--;
				}

				tempMant &= 0x0000FFFFFFFFFFFF;

				this->value = (tempExp << 48) | tempMant;
			}
		}

		else
		{
			cnt = (temp.value >> 48) - (this->value >> 48);
			if(cnt < 48)
			{
				tempExp = (temp.value >> 48);

				this->value &= 0x0000FFFFFFFFFFFF;
				this->value |= 0x0001000000000000;
				this->value >>= cnt;

				tempMant = (temp.value & 0x0000FFFFFFFFFFFF) | 0x0001000000000000;

				tempMant -= this->value;

				if(tempMant > 0x8000000000000000)
				{
					tempMant &= 0x0000FFFFFFFFFFFF;
					tempExp--;
				}

				while(tempMant < 0x1000000000000)
				{
					tempMant <<= 1;
					tempExp--;
				}

				tempMant &= 0x0000FFFFFFFFFFFF;

				this->value = (tempExp << 48) | tempMant;
				this->value |= 0x0100000000000000;				
			}

			else
			{
				this->value = temp.value;
				this->value |= 0x0100000000000000;
			}
		}
	}

	return *this;

}

uStepperEncoder::uStepperEncoder(void)
{
	I2C.begin();
}

float uStepperEncoder::getAngleMoved(void)
{
	return (float)this->angleMoved*0.087890625;
}

float uStepperEncoder::getAngleMovedRaw(void)
{
	return (float)this->angleMovedRaw*0.087890625;
}

float uStepperEncoder::getSpeed(void)
{
	return this->curSpeed;
}

void uStepperEncoder::setup()
{
	TCNT1 = 0;
	ICR1 = 16000;
	TIFR1 = 0;
	TIMSK1 = (1 << OCIE1A);
	TCCR1A = (1 << WGM11);
	TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);
}

void uStepperEncoder::setHome(void)
{
	cli();
	uint8_t data[2];
	TIMSK1 &= ~(1 << OCIE1A);
	I2C.read(ENCODERADDR, ANGLE, 2, data);
	TIMSK1 |= (1 << OCIE1A);
	this->encoderOffset = (((uint16_t)data[0]) << 8 ) | (uint16_t)data[1];

	pointer->stepsSinceReset = 0;
	this->angle = 0;
	this->oldAngle = 0;
	this->angleMoved = 0;
	this->angleMovedRaw = 0;
	this->revolutions = 0;
}

float uStepperEncoder::getAngle()
{
	return (float)this->angle*0.087890625;
}

uint16_t uStepperEncoder::getStrength()
{
	uint8_t data[2];

	TIMSK1 &= ~(1 << OCIE1A);
	I2C.read(ENCODERADDR, MAGNITUDE, 2, data);
	TIMSK1 |= (1 << OCIE1A);

	return (((uint16_t)data[0]) << 8 )| (uint16_t)data[1];
}

uint8_t uStepperEncoder::getAgc()
{
	uint8_t data;
	TIMSK1 &= ~(1 << OCIE1A);
	I2C.read(ENCODERADDR, AGC, 1, &data);
	TIMSK1 |= (1 << OCIE1A);
	return data;
}

uint8_t uStepperEncoder::detectMagnet()
{
	uint8_t data;
	TIMSK1 &= ~(1 << OCIE1A);
	I2C.read(ENCODERADDR, STATUS, 1, &data);
	TIMSK1 |= (1 << OCIE1A);
	data &= 0x38;					//For some reason the encoder returns random values on reserved bits. Therefore we make sure reserved bits are cleared before checking the reply !

	if(data == 0x08)
	{
		return 1;					//magnet too strong
	}

	else if(data == 0x10)
	{
		return 2;					//magnet too weak
	}

	else if(data == 0x20)
	{
		return 0;					//magnet detected and within limits
	}

	return 3;						//Something went horribly wrong !
}

uStepperSLite::uStepperSLite(float accel, float vel)
{
	this->state = STOP;

	this->setMaxVelocity(vel);
	this->setMaxAcceleration(accel);

	p = &(this->control);
	pp = &debugStepCounter;
	pointer = this;

	DDRB |= (1 << 2);		//set direction pin to output
	DDRD |= (1 << 7);		//set step pin to output
	DDRD |= (1 << 4);		//set enable pin to output
}

void uStepperSLite::setMaxAcceleration(float accel)
{
	this->acceleration = accel;

	this->stopTimer();			//Stop timer so we dont fuck up stuff !
	this->multiplier.setValue((this->acceleration/(INTFREQ*INTFREQ)));	//Recalculate multiplier variable, used by the acceleration algorithm since acceleration has changed!
	
	if(this->state != STOP)
	{
		if(this->continous == 1)	//If motor was running continously
		{
			this->runContinous(this->direction);	//We should make it run continously again
		}
		else						//If motor still needs to perform some steps
		{
			this->moveSteps(this->totalSteps - this->currentStep + 1, this->direction, this->hold);	//we should make sure the motor gets to execute the remaining steps				
		}
	}
}

float uStepperSLite::getMaxAcceleration(void)
{
	return this->acceleration;
}

void uStepperSLite::setMaxVelocity(float vel)
{
	if(this->mode == PID)
	{
		if(vel < 0.5005)
		{
			this->velocity = 0.5005;			//Limit velocity in order to not overflow delay variable
		}

		else if(vel > 20000.0)
		{
			this->velocity = 20000.0;			//limit velocity in order to not underflow delay variable
		}

		else
		{
			this->velocity = vel;
		}
	}
	else
	{
		if(vel < 0.5005)
		{
			this->velocity = 0.5005;			//Limit velocity in order to not overflow delay variable
		}

		else if(vel > 28000.0)
		{
			this->velocity = 28000.0;			//limit velocity in order to not underflow delay variable
		}

		else
		{
			this->velocity = vel;
		}
	}
	

	this->stopTimer();			//Stop timer so we dont fuck up stuff !
	this->cruiseDelay = (uint16_t)((INTFREQ/this->velocity) - 0.5);	//Calculate cruise delay, so we dont have to recalculate this in the interrupt routine
	
	if(this->state != STOP)		//If motor was running, we should make sure it runs again
	{
		if(this->continous == 1)	//If motor was running continously
		{
			this->runContinous(this->direction);	//We should make it run continously again
		}
		else					//If motor still needs to perform some steps
		{
			this->moveSteps(this->totalSteps - this->currentStep + 1, this->direction, this->hold);	//we should make sure it gets to execute these steps	
		}
	}
}

float uStepperSLite::getMaxVelocity(void)
{
	return this->velocity;
}

void uStepperSLite::runContinous(bool dir)
{
	float curVel;

	if(this->mode == DROPIN)
	{
		return;		//Drop in feature is activated. just return since this function makes no sense with drop in activated!
	}
	
	this->stopTimer();				//Stop interrupt timer, so we don't fuck up stuff !
	this->continous = 1;			//Set continous variable to 1, in order to let the interrupt routine now, that the motor should run continously

	if(state != STOP)										//if the motor is currently running and we want to move the opposite direction, we need to decelerate in order to change direction.
	{
		curVel = INTFREQ/this->exactDelay.getFloatValue();								//Use this to calculate current velocity
		if(dir != digitalRead(DIR))							//If motor is currently running the opposite direction as desired
		{
			this->direction = dir;
			this->state = INITDECEL;							//We should decelerate the motor to full stop before accelerating the speed in the opposite direction
			this->initialDecelSteps = (uint32_t)(((curVel*curVel))/(2.0*this->acceleration));		//the amount of steps needed to bring the motor to full stop. (S = (V^2 - V0^2)/(2*-a)))
			this->accelSteps = (uint32_t)((this->velocity*this->velocity)/(2.0*this->acceleration));			//Number of steps to bring the motor to max speed (S = (V^2 - V0^2)/(2*a)))

			this->exactDelay.setValue(INTFREQ/sqrt((curVel*curVel) + 2.0*this->acceleration));	//number of interrupts before the first step should be performed.

			if(this->exactDelay.getFloatValue() >= 65535.5)
			{
				this->delay = 0xFFFF;
			}
			else
			{
				this->delay = (uint16_t)(this->exactDelay.getFloatValue() - 0.5);		//Truncate the exactDelay variable, since we cant perform fractional steps
			}
		}
		else												//If the motor is currently rotating the same direction as the desired direction
		{
			if(curVel > this->velocity)						//If current velocity is greater than desired velocity
			{
				this->state = INITDECEL;						//We need to decelerate the motor to desired velocity
				this->initialDecelSteps = (uint32_t)(((this->velocity*this->velocity) - (curVel*curVel))/(-2.0*this->acceleration));		//Number of steps to bring the motor down from current speed to max speed (S = (V^2 - V0^2)/(2*-a)))
				this->accelSteps = 0;						//No acceleration phase is needed
			}

			else if(curVel < this->velocity)					//If the current velocity is less than the desired velocity
			{
				this->state = ACCEL;							//Start accelerating
				this->accelSteps = (uint32_t)(((this->velocity*this->velocity) - (curVel*curVel))/(2.0*this->acceleration));	//Number of Steps needed to accelerate from current velocity to full speed
			}

			else											//If motor is currently running at desired speed
			{
				this->state = CRUISE;						//We should just run at cruise speed
			}
		}
	}

	else																						//If motor is currently stopped (state = STOP)
	{
		this->direction = dir;
		this->state = ACCEL;																	//Start accelerating
		if(dir)																	//Set the motor direction pin to the desired setting
		{
			PORTB |= (1 << 2);
		}
		else
		{
			PORTB &= ~(1 << 2);
		}
		this->accelSteps = (velocity*velocity)/(2.0*acceleration);								//Number of steps to bring the motor to max speed (S = (V^2 - V0^2)/(2*a)))
		
		this->exactDelay.setValue(INTFREQ/sqrt(2.0*this->acceleration));	//number of interrupts before the first step should be performed.
		
		if(this->exactDelay.getFloatValue() > 65535.0)
		{
			this->delay = 0xFFFF;
		}
		else
		{
			this->delay = (uint16_t)(this->exactDelay.getFloatValue() - 0.5);		//Truncate the exactDelay variable, since we cant perform fractional steps
		}
	}
	
	this->startTimer();																			//start timer so we can perform steps
	this->enableMotor();																			//Enable motor
}

void uStepperSLite::moveSteps(int32_t steps, bool dir, bool holdMode)
{
	float curVel;

	if(this->mode == DROPIN)
	{
		return;		//Drop in feature is activated. just return since this function makes no sense with drop in activated!
	}

	if(steps < 1)
	{
		if(holdMode == HARD)
		{
			this->enableMotor();
		}
		else if(holdMode == SOFT)
		{
			this->disableMotor();
		}
		return;
	}

	this->stopTimer();					//Stop interrupt timer so we dont fuck stuff up !
	
	steps--;
	
	this->direction = dir;				//Set direction variable to the desired direction of rotation for the interrupt routine

	this->hold = holdMode;				//Set the hold variable to desired hold mode (block motor or release motor after end movement) for the interrupt routine
	this->totalSteps = steps;			//Load the desired number of steps into the totalSteps variable for the interrupt routine
	this->continous = 0;				//Set continous variable to 0, since the motor should not run continous

	if(state != STOP)					//if the motor is currently running and we want to move the opposite direction, we need to decelerate in order to change direction.
	{
		curVel = INTFREQ/this->exactDelay.getFloatValue();								//Use this to calculate current velocity

		if(dir != digitalRead(DIR))									//If current direction is different from desired direction
		{
			this->state = INITDECEL;									//We should decelerate the motor to full stop
			this->initialDecelSteps = (uint32_t)((curVel*curVel)/(2.0*this->acceleration));		//the amount of steps needed to bring the motor to full stop. (S = (V^2 - V0^2)/(2*-a)))
			this->accelSteps = (uint32_t)((this->velocity * this->velocity)/(2.0*this->acceleration));									//Number of steps to bring the motor to max speed (S = (V^2 - V0^2)/(2*a)))
			this->totalSteps += this->initialDecelSteps;				//Add the steps used for initial deceleration to the totalSteps variable, since we moved this number of steps, passed the initial position, and therefore need to move this amount of steps extra, in the desired direction

			if(this->accelSteps > (this->totalSteps >> 1))			//If we need to accelerate for longer than half of the total steps, we need to start decelerating before we reach max speed
			{
				this->accelSteps = this->decelSteps = (this->totalSteps >> 1);	//Accelerate and decelerate for the same amount of steps (half the total steps)
				this->accelSteps += this->totalSteps - this->accelSteps - this->decelSteps;				//If there are still a step left to perform, due to rounding errors, do this step as an acceleration step	
			}
			else
			{
				this->decelSteps = this->accelSteps;					//If top speed is reached before half the total steps are performed, deceleration period should be same length as acceleration period
				this->cruiseSteps = this->totalSteps - this->accelSteps - this->decelSteps; 			//Perform remaining steps, as cruise steps
			}

			this->exactDelay.setValue(INTFREQ/sqrt((curVel*curVel) + 2.0*this->acceleration));	//number of interrupts before the first step should be performed.

			if(this->exactDelay.getFloatValue() >= 65535.5)
			{
				this->delay = 0xFFFF;
			}
			else
			{
				this->delay = (uint16_t)(this->exactDelay.getFloatValue() - 0.5);		//Truncate the exactDelay variable, since we cant perform fractional steps
			}
		}
		else							//If the motor is currently rotating the same direction as desired, we dont necessarily need to decelerate
		{
			if(curVel > this->velocity)	//If current velocity is greater than desired velocity
			{
				this->state = INITDECEL;	//We need to decelerate the motor to desired velocity
				this->initialDecelSteps = (uint32_t)(((this->velocity*this->velocity) - (curVel*curVel))/(-2.0*this->acceleration));		//Number of steps to bring the motor down from current speed to max speed (S = (V^2 - V0^2)/(2*-a)))
				this->accelSteps = 0;	//No acceleration phase is needed
				this->decelSteps = (uint32_t)((this->velocity*this->velocity)/(2.0*this->acceleration));	//Number of steps needed to decelerate the motor from top speed to full stop
				this->exactDelay.setValue((INTFREQ/sqrt((curVel*curVel) + 2*this->acceleration)));

				if(this->totalSteps <= (this->initialDecelSteps + this->decelSteps))
				{
					this->cruiseSteps = 0;
				}
				else
				{
					this->cruiseSteps = steps - this->initialDecelSteps - this->decelSteps;					//Perform remaining steps as cruise steps
				}

				
			}

			else if(curVel < this->velocity)	//If current velocity is less than desired velocity
			{
				this->state = ACCEL;			//Start accelerating
				this->accelSteps = (uint32_t)(((this->velocity*this->velocity) - (curVel*curVel))/(2.0*this->acceleration));	//Number of Steps needed to accelerate from current velocity to full speed

				if(this->accelSteps > (this->totalSteps >> 1))			//If we need to accelerate for longer than half of the total steps, we need to start decelerating before we reach max speed
				{
					this->accelSteps = this->decelSteps = (this->totalSteps >> 1);	//Accelerate and decelerate for the same amount of steps (half the total steps)
					this->accelSteps += this->totalSteps - this->accelSteps - this->decelSteps;				//If there are still a step left to perform, due to rounding errors, do this step as an acceleration step	
					this->cruiseSteps = 0;
				}
				else
				{
					this->decelSteps = this->accelSteps;					//If top speed is reached before half the total steps are performed, deceleration period should be same length as acceleration period
					this->cruiseSteps = this->totalSteps - this->accelSteps - this->decelSteps; 			//Perform remaining steps, as cruise steps
				}

				this->cruiseSteps = steps - this->accelSteps - this->decelSteps;	//Perform remaining steps as cruise steps
				this->initialDecelSteps = 0;								//No initial deceleration phase needed
			}

			else						//If current velocity is equal to desired velocity
			{
				this->state = CRUISE;	//We are already at desired speed, therefore we start at cruise phase
				this->decelSteps = (uint32_t)((this->velocity*this->velocity)/(2.0*this->acceleration));	//Number of steps needed to decelerate the motor from top speed to full stop
				this->accelSteps = 0;	//No acceleration phase needed
				this->initialDecelSteps = 0;		//No initial deceleration phase needed

				if(this->decelSteps >= this->totalSteps)
				{
					this->cruiseSteps = 0;
				}
				else
				{
					this->cruiseSteps = steps - this->decelSteps;	//Perform remaining steps as cruise steps
				}
			}
		}
	}
	
	else								//If motor is currently at full stop (state = STOP)
	{
		if(dir)																	//Set the motor direction pin to the desired setting
		{
			PORTB |= (1 << 2);
		}
		else
		{
			PORTB &= ~(1 << 2);
		}
		this->state = ACCEL;
		this->accelSteps = (uint32_t)((this->velocity * this->velocity)/(2.0*this->acceleration));	//Number of steps to bring the motor to max speed (S = (V^2 - V0^2)/(2*a)))
		this->initialDecelSteps = 0;		//No initial deceleration phase needed

		if((int32_t)this->accelSteps > (steps >> 1))	//If we need to accelerate for longer than half of the total steps, we need to start decelerating before we reach max speed
		{
			this->cruiseSteps = 0; 		//No cruise phase needed
			this->accelSteps = this->decelSteps = (steps >> 1);				//Accelerate and decelerate for the same amount of steps (half the total steps)
			this->accelSteps += steps - this->accelSteps - this->decelSteps;	//if there are still a step left to perform, due to rounding errors, do this step as an acceleration step	
		}

		else								
		{
			this->decelSteps = this->accelSteps;	//If top speed is reached before half the total steps are performed, deceleration period should be same length as acceleration period
			this->cruiseSteps = steps - this->accelSteps - this->decelSteps;	//Perform remaining steps as cruise steps
		}
		this->exactDelay.setValue(INTFREQ/sqrt(2.0*this->acceleration));	//number of interrupts before the first step should be performed.

		if(this->exactDelay.getFloatValue() > 65535.0)
		{
			this->delay = 0xFFFF;
		}
		else
		{
			this->delay = (uint16_t)(this->exactDelay.getFloatValue() - 0.5);		//Truncate the exactDelay variable, since we cant perform fractional steps
		}
	}

	this->startTimer();									//start timer so we can perform steps
	this->enableMotor();									//Enable motor driver
}

void uStepperSLite::hardStop(bool holdMode)
{
	#warning "Function only here for compatibility with old code. this function is replaced by 'stop(bool brake)'"

	this->stop(holdMode);
}

void uStepperSLite::stop(bool brake)
{
	if(this->mode == DROPIN)
	{
		return;		//Drop in feature is activated. just return since this function makes no sense with drop in activated!
	}

	this->stall = 0;

	this->stepsSinceReset = (int32_t)(this->encoder.getAngleMoved()*this->angleToStep);

	this->stopTimer();			//Stop interrupt timer, since we shouldn't perform more steps
	this->hold = brake;
	
	if(state != STOP && this->mode == NORMAL)
	{
		this->startTimer();
	}

	else
	{
		if(brake == BRAKEOFF)
		{
			this->disableMotor();
		}
		
		else if (brake == BRAKEON)
		{
			this->enableMotor();
		}
	}
	this->state = STOP;			//Set current state to STOP
}

void uStepperSLite::softStop(bool holdMode)
{
	#warning "Function only here for compatibility with old code. this function is replaced by 'stop(bool brake)'"

	this->stop(holdMode);
}

void uStepperSLite::setup(	uint8_t mode, 
							uint8_t microStepping, 
							float faultTolerance,
							float faultHysteresis, 
							float pTerm, 
							float iTerm, 
							float dTerm,
							bool setHome)
{
	uint8_t data[2], i;
	uint16_t angle;
	int16_t angleDiff[3];

	this->mode = mode;
	this->encoder.setup();
	
	I2C.read(ENCODERADDR, ANGLE, 2, data);
	angle = (((uint16_t)data[0]) << 8 ) | (uint16_t)data[1];

	this->driver.setup();
	this->driver.enableDriver();
	_delay_ms(2000);
	this->encoder.setHome();
	PORTB |= (1 << 2);

	for(i = 0; i < 50; i++)
	{
		PORTD |= (1 << 7);
		delayMicroseconds(1);
		PORTD &= ~(1 << 7);
		_delay_ms(10);
	}

	angleDiff[0] = (int16_t)angle;

	I2C.read(ENCODERADDR, ANGLE, 2, data);
	angle = (((uint16_t)data[0]) << 8 ) | (uint16_t)data[1];

	angleDiff[0] -= (int16_t)angle;

	Serial.print("CCW: ");
	Serial.println(angleDiff[0]);

	PORTB &= ~(1 << 2);

	for(i = 0; i < 50; i++)
	{
		PORTD |= (1 << 7);
		delayMicroseconds(1);
		PORTD &= ~(1 << 7);
		_delay_ms(10);
	}

	angleDiff[1] = (int16_t)angle;

	I2C.read(ENCODERADDR, ANGLE, 2, data);
	angle = (((uint16_t)data[0]) << 8 ) | (uint16_t)data[1];

	angleDiff[1] -= (int16_t)angle;

	Serial.print("CW: ");
	Serial.println(angleDiff[1]);

	PORTB |= (1 << 2);

	for(i = 0; i < 50; i++)
	{
		PORTD |= (1 << 7);
		delayMicroseconds(1);
		PORTD &= ~(1 << 7);
		_delay_ms(10);
	}

	angleDiff[2] = (int16_t)angle;

	I2C.read(ENCODERADDR, ANGLE, 2, data);
	angle = (((uint16_t)data[0]) << 8 ) | (uint16_t)data[1];

	angleDiff[2] -= (int16_t)angle;

	for(i = 0; i < 3; i++)
	{
		if(angleDiff[i] > 2048)
		{
			angleDiff[i] -= 4096;
		}
		else if(angleDiff[i] < -2048)
		{
			angleDiff[i] += 4096;
		}
	}

	if(!(angleDiff[0] < -3 && angleDiff[1] > 3 && angleDiff[2] < -3))
	{
		this->driver.invertDirection();
	}

	this->encoder.setHome();	
	this->stepConversion = ((float)(200*microStepping))/4096.0;	//Calculate conversion coefficient from raw encoder data, to actual moved steps
	this->angleToStep = ((float)(200*microStepping))/360.0;	//Calculate conversion coefficient from angle to corresponding number of steps
	if(setHome)
	{
		this->encoder.setHome();	
	}
	else
	{
		pointer->stepsSinceReset = ((float)this->encoder.angleMoved * this->stepConversion) + 0.5;
	}

	if(this->mode)
	{
		if(this->mode == DROPIN)
		{
			_delay_ms(4000);
			//Set Enable, Step and Dir signal pins from 3dPrinter controller as inputs
			pinMode(2,INPUT);		
			pinMode(3,INPUT);
			pinMode(4,INPUT);
			//Enable internal pull-up resistors on the above pins
			digitalWrite(2,HIGH);
			digitalWrite(3,HIGH);
			digitalWrite(4,HIGH);
			attachInterrupt(0, interrupt0, FALLING);
			attachInterrupt(1, interrupt1, CHANGE);
		}		
		this->tolerance = faultTolerance;		//Number of steps missed before controller kicks in
		this->hysteresis = faultHysteresis;
		
		//Scale supplied controller coefficents. This is done to enable the user to use easier to manage numbers for these coefficients.
	    this->pTerm = pTerm;    
	    this->iTerm = iTerm*ENCODERINTSAMPLETIME;
	    this->dTerm = dTerm/ENCODERINTSAMPLETIME;
	}
	
	TCCR2B &= ~((1 << CS20) | (1 << CS21) | (1 << CS22) | (1 << WGM22));
	TCCR2A &= ~((1 << WGM20) | (1 << WGM21));
	TCCR2B |= (1 << CS21)| (1 << WGM22);				//Enable timer with prescaler 8 - interrupt base frequency ~ 2MHz
	TCCR2A |= (1 << WGM21) | (1 << WGM20);				//Switch timer 2 to Fast PWM mode, to enable adjustment of interrupt frequency, while being able to use PWM
	OCR2A = 70;											//Change top value to 70 in order to obtain an interrupt frequency of 28.571kHz
	OCR2B = 70;

	sei();
}

void uStepperSLite::startTimer(void)
{
	while(TCNT2);						//Wait for timer to overflow, to ensure correct timing.
	TIFR2 |= (1 << OCF2A);				//Clear compare match interrupt flag, if it is set.
	TIMSK2 |= (1 << OCIE2A);			//Enable compare match interrupt
	sei();
}

void uStepperSLite::stopTimer(void)
{
	TIMSK2 &= ~(1 << OCIE2A);			//disable compare match interrupt
}

void uStepperSLite::enableMotor(void)
{
	PORTD &= ~(1 << 4);				//Enable motor driver
}

void uStepperSLite::disableMotor(void)
{
	PORTD |= (1 << 4);			//Disable motor driver
}

bool uStepperSLite::getCurrentDirection(void)
{
	return this->direction;
}

bool uStepperSLite::getMotorState(void)
{
	if(this->mode == PID)
	{
		if(this->control)
		{
			return 1;
		}
		else if(this->state != STOP)
		{
			return 1;		//Motor running
		}
		return 0;
	}
	else
	{
		if(this->state != STOP)
		{
			return 1;		//Motor running
		}

		return 0;			//Motor not running
	}
}

int32_t uStepperSLite::getStepsSinceReset(void)
{
	return this->stepsSinceReset;
}

void uStepperSLite::setCurrent(uint8_t runCurrent, uint8_t holdCurrent)
{
	this->driver.setCurrent(runCurrent, holdCurrent);
}

void uStepperSLite::setHoldCurrent(uint8_t holdCurrent)
{
	this->driver.setHoldCurrent(holdCurrent);
}

void uStepperSLite::setRunCurrent(uint8_t runCurrent)
{
	this->driver.setHoldCurrent(runCurrent);
}

float uStepperSLite::moveToEnd(bool dir)
{
	uint8_t checks = 0;
  	float pos = 0.0;
  	float lengthMoved;

  	lengthMoved = this->encoder.getAngleMoved();
  	
  	this->stop(HARD);
	_delay_ms(50);
  	this->runContinous(dir);

  	if(this->mode == PID)
  	{
  		while(!this->isStalled());
  		this->stop(SOFT);//stop motor without brake
  	}
  	else
  	{
  		
		while(checks < 5)//allows for 2 checks on movement error
		{
			pos = abs(this->encoder.getAngleMoved() - (this->getStepsSinceReset()*0.1125));//see current position error
			if(pos < 5.0)//if position error is less than 5 steps it is okay...
			{
				checks = 0;
			}
			else //if position error is 5 steps or more, count up checks
			{
		  		checks++;
			}
		}

	  	this->stop(SOFT);//stop motor without brake
  	}
  	
	this->moveSteps(20, !dir, SOFT);
	while(this->getMotorState())
	{
		_delay_ms(1);
	}
	_delay_ms(100);
	if(dir == CW)
	{
		lengthMoved = this->encoder.getAngleMoved() - lengthMoved;
	}
	else
	{
		lengthMoved -= this->encoder.getAngleMoved();
	}
  	this->encoder.setHome();//set new home position

  	return lengthMoved;
}

void uStepperSLite::moveToAngle(float angle, bool holdMode)
{
	float diff;
	uint32_t steps;

	diff = angle - this->encoder.getAngleMoved();
	steps = (uint32_t)((abs(diff)*angleToStep) + 0.5);
	
	if(diff < 0.0)
	{
		this->moveSteps(steps, CCW, holdMode);
	}
	else
	{
		this->moveSteps(steps, CW, holdMode);
	}
}

void uStepperSLite::moveAngle(float angle, bool holdMode)
{
	int32_t steps;

	if(angle < 0.0)
	{
		steps = -(int32_t)((angle*angleToStep) - 0.5);
		this->moveSteps(steps, CCW, holdMode);
	}
	else
	{
		steps = (int32_t)((angle*angleToStep) + 0.5);
		this->moveSteps(steps, CW, holdMode);
	}
}

void uStepperSLite::pid(int16_t deltaAngle)
{
	static float oldError;
	float integral;
	float output;
	static float accumError;
	float error;
	static float currentSpeed;
	static float temp;
	static uint8_t checks = 0;

	if(this->mode == DROPIN)
	{
		cli();
			error = (((float)this->stepCnt - ((float)this->encoder.angleMoved * this->stepConversion))); 
		sei();
	}
	else
	{
		cli();
			error = (((float)this->stepsSinceReset - ((float)this->encoder.angleMoved * this->stepConversion)));
		sei();
	}
/*
	if((currentSpeed >= 10.0 || currentSpeed <= -10.0) && (error > 2.0 || error < -2.0))
	{
		temp += (error - oldError);
		checks++;

		if( checks >= 100)
		{
			temp *= 0.01;

			if(temp < 1.0 && temp > -1.0)
			{
				this->stall = 1;
				if(currentSpeed < 0.0)
				{
					currentSpeed = -10.0;
				}
				else
				{
					currentSpeed = 10.0;
				}
			}
			else
			{
				this->stall = 0;
			}
			temp = 0.0;
			checks = 0;
		}
	}
	else
	{
		this->stall = 0;
		temp = 0.0;
		checks = 0;
	}
*/
	integral = error*this->iTerm;	//Multiply current error by integral term
	accumError += integral;			//And accumulate, to get integral action
	
	output = this->pTerm*error;		
	output += accumError;
	output += this->dTerm*(error - oldError);
	
	debugOutput[1] = error;//this->dTerm*(error - oldError);

	output *= 0.01875;		//60/3200

	if(output > 1000.0 )		//If stepSpeed is lower than possible
	{
		output = 1000.0;		//Set stepSpeed to lowest possible
		accumError -= integral;	//and subtract current integral part from accumerror (anti-windup)
	}
	else if(output < -1000.0 )		//If stepSpeed is lower than possible
	{
		output = -1000.0;		//Set stepSpeed to lowest possible
		accumError -= integral;	//and subtract current integral part from accumerror (anti-windup)
	}

	if(accumError > 300)
	{
		accumError = 300;
	}
	else if(accumError < -300)
	{
		accumError = -300;
	}

	oldError = error;		//Save current error for next sample, for use in differential part	
	
	if(output != 0.0)
	{
		temp = 10.0/abs(output);
	}
	else
	{
		temp = 10.0;
	}

	if(output > currentSpeed)
	{
		if(currentSpeed + temp < output)
		{
			currentSpeed += temp; 
		}
		else
		{
			currentSpeed = output;
		}
	}
	else if(output < currentSpeed)
	{
		if(currentSpeed - temp > output)
		{
			currentSpeed -= temp; 
		}
		else
		{
			currentSpeed = output;
		}
	}
	else
	{
		currentSpeed = output;
	}


	this->driver.setVelocity(currentSpeed);
	this->control = 1;
}

bool uStepperSLite::detectStall(float diff, bool running)
{
	static uint16_t checks = 0;
	static float temp = 0.0;
	uint32_t treshold;

	treshold = (uint32_t)this->exactDelay.getFloatValue();
	if(treshold == 0)
	{
		return 0;
	}
	treshold *= treshold;
	if(treshold >= 5000)
	{
		treshold = 5000;
	}

	if(running)
	{
		temp += diff;
		checks++;

		if( checks >= treshold)
		{
			temp /= (float)checks;

			if(temp < 0.3 && temp > -0.3)
			{
				this->stall = 1;
			}
			else
			{
				this->stall = 0;
			}
			temp = 0.0;
			checks = 0;
		}
	}
	else
	{
		this->stall = 0;
		temp = 0.0;
		checks = 0;
	}
	return 0;
}

bool uStepperSLite::isStalled(void)
{
	return this->stall;
}