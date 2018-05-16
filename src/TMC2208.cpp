#include "TMC2208.h"

static const uint8_t reflectedCRCTable[256] PROGMEM = {
//	   0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
	0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B, // 0x00 - 0x0F
	0x1C, 0x8D, 0xFF, 0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, // 0x10 - 0x1F
	0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE, 0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43, // 0x20 - 0x2F
	0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A, 0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F, // 0x30 - 0x3F
	0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C, 0x79, 0xE8, 0x9A, 0x0B, // 0x40 - 0x4F
	0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86, 0x17, // 0x50 - 0x5F
	0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, // 0x60 - 0x6F
	0x54, 0xC5, 0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, // 0x70 - 0x7F
	0xE0, 0x71, 0x03, 0x92, 0xE7, 0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, // 0x80 - 0x8F
	0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89, 0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, // 0x90 - 0x9F
	0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35, 0xA4, 0xD1, 0x40, 0x32, 0xA3, // 0xA0 - 0xAF
	0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C, 0x2E, 0xBF, // 0xB0 - 0xBF
	0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, // 0xC0 - 0xCF
	0x8C, 0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, // 0xD0 - 0xDF
	0xA8, 0x39, 0x4B, 0xDA, 0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3, // 0xE0 - 0xEF
	0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50, 0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF  // 0xF0 - 0xFF
};

uint8_t Tmc2208::crc8(const uint8_t *data, uint32_t bytes)
{
	uint8_t reflectedCRC = 0, CRCResult = 0;

	// Use a reflected CRC-LUT to calculate the (reflected) CRC8 of the *data buffer
	while(bytes--)
		reflectedCRC = pgm_read_byte(reflectedCRCTable[reflectedCRC ^ *data++]);

	// Reflect reflectedCRC to get the actual CRC value
	for(uint8_t i = 0; i < 8; i++) {
		CRCResult <<= 1;
		CRCResult += reflectedCRC & 0x01;
		reflectedCRC >>= 1;
	}

	return CRCResult;
}

void Tmc2208::writeRegister(uint8_t address, int32_t value)
{
	uint8_t writeData[8];

	writeData[0] = 0x05;                         // Sync byte
	writeData[1] = 0x00;                         // Slave address
	writeData[2] = address | TMC2208_WRITE_BIT;  // Register address with write bit set
	writeData[3] = value >> 24;                  // Register Data
	writeData[4] = value >> 16;                  // Register Data
	writeData[5] = value >> 8;                   // Register Data
	writeData[6] = value & 0xFF;                 // Register Data
	writeData[7] = this->crc8(writeData, 7);     // Cyclic redundancy check

	for(uint32_t i = 0; i < ARRAY_SIZE(writeData); i++)
	{
		this->uartSendByte(writeData[i]);	
	}
}

void Tmc2208::readRegister(uint8_t address, int32_t *value)
{
	uint8_t readData[8], dataRequest[4];
	

	// Clear write bit
	address &= ~TMC2208_WRITE_BIT;

	dataRequest[0] = 0x05;                  // Sync byte
	dataRequest[1] = 0x00;                  // Slave address
	dataRequest[2] = address;               // Register address
	dataRequest[3] = crc8(dataRequest, 3);  // Cyclic redundancy check

	for(uint32_t i = 0; i < ARRAY_SIZE(dataRequest); i++)
	{
		this->uartSendByte(dataRequest[i]);	
	}

	this->uartReceivePacket(readData, 8);

	// Check if the received data is correct (CRC, Sync, Slave address, Register address)
	// todo CHECK 2: Only keep CRC check? Should be sufficient for wrong transmissions (LH) #1
	if(readData[7] != crc8(readData, 7) || readData[0] != 0x05 || readData[1] != 0xFF || readData[2] != address)
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
	DDRB |= (1 << 1);			//Set Step pin as output
	DDRB |= (1 << 2);			//Set Dir pin as Output
	
	this->uartInit();
	registerSetting = R00;
	registerSetting |= TMC2208_PDN_DISABLE_MASK;
	//registerSetting |= TMC2208_MSTEP_REG_SELECT_MASK;
	this->writeRegister(TMC2208_GCONF, registerSetting);
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
	UBRR0H = (uint8_t)(MYUBRR >> 8);
	UBRR0L = (uint8_t)MYUBRR;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0A = 0;
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void Tmc2208::uartSendByte(uint8_t value)
{
		while ( !( UCSR0A & (1<<UDRE0)) );
		UDR0 = value;
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