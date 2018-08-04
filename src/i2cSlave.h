#ifndef _I2CSLAVE_H_
#define _I2CSLAVE_H_

#include "queue.h"
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/io.h>
#include <inttypes.h>

#define TW_STATUS_MASK (_BV(TWS07)|_BV(TWS06)|_BV(TWS05)|_BV(TWS04)|_BV(TWS03))
#define TW_STATUS (TWSR0 & TW_STATUS_MASK) 

/** I2C bus is not currently in use */
#define I2CFREE 0						

/** Value for RW bit in address field, to request a read */
#define READ  1							

/** Value for RW bit in address field, to request a write */
#define WRITE 0							

/** start condition transmitted */
#define START  0x08						

/** repeated start condition transmitted */
#define REPSTART 0x10					

/** slave address plus write bit transmitted, ACK received */
#define TXADDRACK  0x18					

/** data transmitted, ACK received */
#define TXDATAACK 0x28					

/** slave address plus read bit transmitted, ACK received */
#define RXADDRACK 0x40	

extern "C" void TWI_vect(void) __attribute__ ((signal,used));

class I2CSlave
{
public:
	void* operator new(size_t size);
	
	I2CSlave(void (*recv)(uint8_t), void (*req)(uint8_t), void (*finished)(uint8_t));
	void setup(uint8_t channel, uint8_t address);
	inline void __attribute__((always_inline)) I2CTransmitByte(uint8_t data)
	{
  		TWDR0 = data;
	}

private:
	void I2CSetCallbacks(void (*recv)(uint8_t), void (*req)(uint8_t), void (*finished)(uint8_t));
	void I2CStop(void);
};

#endif