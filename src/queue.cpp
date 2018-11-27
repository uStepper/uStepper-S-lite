#include "queue.h"

CommandPacket & CommandPacket::operator=(const CommandPacket& a)
{
	uint8_t i;
	
	this->command = a.command;

	for(i = 0; i < bytes; i++)
	{
		this->data[i] = a.data[i];	
	}

	this->bytes = a.bytes;

	return *this;
}