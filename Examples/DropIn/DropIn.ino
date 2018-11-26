/*
*			DropIn example sketch!
*
*	Pin connections:
*	-----------------------------
*	| Controller | uStepperS Lite|
*	|----------------------------|
*	|	Enable   |		D2		 |
*	|	Step     |		D3		 |
*	|	Dir      |		D4		 |
*	|	GND      |		GND		 |
*	------------------------------	
*/

#include <uStepperSLite.h>

uStepperSLite stepper;

void setup(void)
{
	stepper.setup(DROPIN,3200.0,0.75,10.0,0.0,true);
}

void loop(void)
{
}
