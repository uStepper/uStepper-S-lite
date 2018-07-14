#include "TMC2208.h"

uint8_t Tmc2208::calcCRC(uint8_t datagram[], uint8_t len) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) {
		uint8_t currentByte = datagram[i];
		for (uint8_t j = 0; j < 8; j++) {
			if ((crc >> 7) ^ (currentByte & 0x01)) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc = (crc << 1);
			}
			crc &= 0xff;
			currentByte = currentByte >> 1;
		} 
	}
	return crc;
}

void Tmc2208::writeRegister(uint8_t address, int32_t value)
{
	uint8_t writeData[8];
	cli();
	writeData[0] = 0x05;                         // Sync byte
	writeData[1] = 0x00;                         // Slave address
	writeData[2] = address | TMC2208_WRITE_BIT;  // Register address with write bit set
	writeData[3] = value >> 24;                  // Register Data
	writeData[4] = value >> 16;                  // Register Data
	writeData[5] = value >> 8;                   // Register Data
	writeData[6] = value & 0xFF;                 // Register Data
	writeData[7] = calcCRC(writeData, 7);     // Cyclic redundancy check

	for(uint32_t i = 0; i < ARRAY_SIZE(writeData); i++)
	{
		this->uartSendByte(writeData[i]);	
	}
	sei();
}

void Tmc2208::readRegister(uint8_t address, int32_t *value)
{
	uint8_t readData[8], dataRequest[4];
	
cli();
	// Clear write bit
	address &= ~TMC2208_WRITE_BIT;

	dataRequest[0] = 0x05;                  // Sync byte
	dataRequest[1] = 0x00;                  // Slave address
	dataRequest[2] = address;               // Register address
	dataRequest[7] = calcCRC(dataRequest, 3);     // Cyclic redundancy check

	for(uint32_t i = 0; i < ARRAY_SIZE(dataRequest); i++)
	{
		this->uartSendByte(dataRequest[i]);	
	}

	//this->uartReceivePacket(readData, 8);
	_delay_ms(1);
	return;

	// Check if the received data is correct (CRC, Sync, Slave address, Register address)
	// todo CHECK 2: Only keep CRC check? Should be sufficient for wrong transmissions (LH) #1
	if(readData[7] != calcCRC(readData, 7) || readData[0] != 0x05 || readData[1] != 0xFF || readData[2] != address)
		return;

	*value = readData[3] << 24 | readData[4] << 16 | readData[5] << 8 | readData[6];
	return;
}

Tmc2208::Tmc2208(void)
{

}

void Tmc2208::setup(void)
{
	int32_t registerSetting;

	DDRD |= (1 << 4);			//Set Enable as output
	DDRD |= (1 << 7);			//Set Step pin as output
	DDRB |= (1 << 2);			//Set Dir pin as Output

	this->uartInit();
	registerSetting = R00;
	registerSetting |= TMC2208_PDN_DISABLE_MASK;
	this->writeRegister(TMC2208_GCONF, registerSetting);
	registerSetting = 5000;
	this->writeRegister(TMC2208_TPWMTHRS, registerSetting);
	this->setCurrent(25);
	this->setVelocity(200);	
}

void Tmc2208::enableDriver(void)
{
	PORTD &= ~(1 << 4);				//Enable motor driver
}

void Tmc2208::disableDriver(void)
{
	PORTD |= (1 << 4);				//Disable motor driver
}

void Tmc2208::uartInit(void)
{
	UARTRXDDR &= ~(1 << UARTRXPIN);		//Set RX pin as input
	UARTTXDDR |= (1 << UARTTXPIN);		//Set TX pin as Output

	UARTRXPORT |= (1 << UARTRXPIN);		//Set RX pin pullup enable
	UARTTXPORT |= (1 << UARTTXPIN);		//Set TX pin high
}

void Tmc2208::uartSendByte(uint8_t value)
{
	uint8_t mask = 1;
	

	//Start bit
	UARTTXPORT &= ~(1 << UARTTXPIN); UARTCLKDELAY();
	while(mask)
	{
		if (mask & value)
			UARTTXPORT |= (1 << UARTTXPIN);
		else
			UARTTXPORT &= ~(1 << UARTTXPIN);
		UARTCLKDELAY();
		mask <<= 1;
	}

	// Stop bit
	UARTTXPORT |= (1 << UARTTXPIN);	UARTCLKDELAY();
	
}

bool Tmc2208::uartReceivePacket(uint8_t *packet, uint8_t size)
{
	uint32_t timeout = millis();

	while(size--)
	{
		while ( !(UCSR0A & (1<<RXC0)) )
		{
			if((millis() - timeout) > TIMEOUT_VALUE) // Timeout
			{
				return;
			}
		}
		*packet++ = UDR0;
	}
}

void Tmc2208::setCurrent(uint8_t percent)
{
	int32_t registerSetting = 0;

	if(percent >= 100)
	{
		percent = 31;
	}
	else if(percent == 0)
	{
		//Do nothing
	}
	else
	{
		percent = ((uint8_t)(float)percent * 0.32) - 1; 
	}

	registerSetting |= ((int32_t)(percent & 0x1F));
	registerSetting |= (((int32_t)(percent & 0x1F)) << 8);

	this->writeRegister(TMC2208_IHOLD_IRUN, registerSetting);
}

void Tmc2208::setVelocity(int32_t RPM)
{
	float dummy;

	dummy = (float)RPM;
	dummy *= 0.01666667;

	dummy *= 3200;

	dummy *= 1.0486;

	RPM = (int32_t)(dummy + 0.5);

	this->writeRegister(TMC2208_VACTUAL, RPM);
}