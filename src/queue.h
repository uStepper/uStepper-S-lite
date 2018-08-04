#ifndef _QUEUE_H_
#define _QUEUE_H_

#include <inttypes.h>
#include <Arduino.h>

#define QUEUESIZE 12

class CommandPacket
{
public:

	uint8_t command;
	uint8_t data[10];
	uint8_t bytes;
	
	CommandPacket(){}
	CommandPacket & operator=(const CommandPacket& a);
};

class ByteQueue{
public:	
    ByteQueue(){}
	bool popMsg(uint8_t *msg)
	{ 	
		if(this->readPointer != this->writePointer)
		{	
			*msg = queue[this->readPointer];
			
			this->readPointer++;

			this->readPointer %= QUEUESIZE;
			return 1;
		}
		
		return 0;

	}
	
	void pushMsg(uint8_t *msg)
	{
		queue[this->writePointer] = *msg;
		this->writePointer++;

		this->writePointer %= QUEUESIZE;

		return;
	}

private:
	uint8_t readPointer;
	uint8_t writePointer;
	uint8_t queue[QUEUESIZE];
};
/*
class CommandQueue{
public:	
    CommandQueue(){}
	bool popMsg(CommandPacket *msg)
	{ 	delay(1);
		if(this->readPointer != this->writePointer)
		{	
			*msg = queue[this->readPointer];
			
			this->readPointer++;

			this->readPointer %= QUEUESIZE;
			return 1;
		}
		
		return 0;

	}
	
	void pushMsg(CommandPacket *msg)
	{
		queue[this->writePointer] = *msg;
		this->writePointer++;

		this->writePointer %= QUEUESIZE;

		return;
	}

private:
	uint8_t readPointer;
	uint8_t writePointer;
	CommandPacket queue[4];
};*/

#endif
