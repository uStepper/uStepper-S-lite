#include "i2cSlave.h"

static void (*I2CRecv)(uint8_t);
static void (*I2CReq)(uint8_t);
static void (*I2CFinished)(uint8_t);

extern "C" {
	void TWI_vect(void)
	{
		static uint8_t address = 0;
		static uint8_t bytesRecieved = 0;

	  	switch(TW_STATUS)
	  	{
	    case TW_SR_DATA_ACK:
	      // received data from master, call the receive callback
	      if(!bytesRecieved)
	      {
	      	address = TWDR0;
	      	I2CRecv(address);
	      }
	      else
	      {
	      	I2CRecv(TWDR0);
	      }
	       
	      bytesRecieved++;
	      TWCR0 = (1<<TWIE0) | (1<<TWINT0) | (1<<TWEA0) | (1<<TWEN0);
	      break;
	    case TW_ST_SLA_ACK:
	      // master is requesting data, call the request callback
	      bytesRecieved = 0;

	      I2CReq(address++);
	      TWCR0 = (1<<TWIE0) | (1<<TWINT0) | (1<<TWEA0) | (1<<TWEN0);
	      break;
	    case TW_ST_DATA_ACK:
	      // master is requesting data, call the request callback
	      bytesRecieved = 0;
	      I2CReq(address++);
	      TWCR0 = (1<<TWIE0) | (1<<TWINT0) | (1<<TWEA0) | (1<<TWEN0);
	      break;
	    case TW_BUS_ERROR:
	      // some sort of erroneous state, prepare TWI to be readdressed
	      bytesRecieved = 0;
	      TWCR0 = 0;
	      TWCR0 = (1<<TWIE0) | (1<<TWINT0) | (1<<TWEA0) | (1<<TWEN0); 
	      break;
	    case TW_SR_STOP:
	    	I2CFinished(bytesRecieved);
	    	bytesRecieved = 0;
	    default:
	      TWCR0 = (1<<TWIE0) | (1<<TWINT0) | (1<<TWEA0) | (1<<TWEN0);
	      break;
	  	}
	}
}

I2CSlave::I2CSlave(void (*recv)(uint8_t), void (*req)(uint8_t), void (*finished)(uint8_t))
{
	I2CSetCallbacks(recv, req, finished);
}

void I2CSlave::setup(uint8_t channel __attribute__((unused)), uint8_t address)
{
	cli();
	// load address into TWI address register
	TWAR0 = address << 1;
	// set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
	TWCR0 = (1<<TWIE0) | (1<<TWEA0) | (1<<TWINT0) | (1<<TWEN0);
	sei();
}

void I2CSlave::I2CSetCallbacks(void (*recv)(uint8_t), void (*req)(uint8_t), void (*finished)(uint8_t))
{
	I2CRecv = recv;
	I2CReq = req;
	I2CFinished = finished;
}

void I2CSlave::I2CStop(void)
{
	// clear acknowledge and enable bits
	cli();
	TWCR0 = 0;
	TWAR0 = 0;
	sei();
}

void* I2CSlave::operator new(size_t size)
{
	void *object = malloc(size);
	return object;
}