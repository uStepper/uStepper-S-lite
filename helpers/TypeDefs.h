/*
 * Types.h
 *
 *  Created on: 29.09.2016
 *      Author: ed
 */

#ifndef API_TYPEDEFS_H
#define API_TYPEDEFS_H

	// unsigned types
	typedef unsigned char       u8;
	typedef unsigned short      u16;
	typedef unsigned int        u32;
	typedef unsigned long long  u64;

	typedef unsigned char       uint8;
	typedef unsigned short int  uint16;
	typedef unsigned long int   uint32;
	typedef unsigned long long  uint64;

	#define u8_MAX   (u8)   255
	#define u10_MAX  (u16)  1023
	#define u12_MAX  (u16)  4095
	#define u15_MAX  (u16)  32767
	#define u16_MAX  (u16)  65535
	#define u18_MAX  (u32)  262143uL
	#define u20_MAX  (u32)  1048575uL
	#define u22_MAX  (u32)  4194303uL
	#define u24_MAX  (u32)  16777215uL
	#define u32_MAX  (u32)  4294967295uL

	// signed types
	typedef signed char       s8;
	typedef signed short      s16;
	typedef signed int        s32;
	typedef signed long long  s64;

	typedef signed char       int8;
	typedef signed short int  int16;
	typedef signed long int   int32;
	typedef signed long long  int64;

	#define s8_MAX   (s8)   127
	#define s8_MIN   (s8)   -128
	#define s16_MAX  (s16)  32767
	#define s16_MIN  (s16)  -32768
	#define s24_MAX  (s32)  8388607
	#define s24_MIN  (s32)  -8388608
	#define s32_MAX  (s32)  2147483647
	#define s32_MIN  (s32)  -2147483648

	/* Helper Macro: Cast a n bit signed int to a 32 bit signed int
	 * This is done by checking the MSB of the signed int (Bit n).
	 * If it is 1, the value is negative and the Bits 32 to n+1 are set to 1
	 * If it is 0, the value remains unchanged
	 */
	#define CAST_Sn_TO_S32(value, n) ((value) | (((value) & (1<<((n)-1)))? ~((0x1<<(n))-1) : 0 ))

	// bool type
	typedef unsigned char  bool;

	#define FALSE  0
	#define TRUE   1

	// parameter access
	#define READ   0
	#define WRITE  1

	// Min/Max macros
	#ifndef MIN
		#define MIN(a,b) ((a<b) ? (a) : (b))
	#endif
	#ifndef MAX
		#define MAX(a,b) ((a>b) ? (a) : (b))
	#endif

	// Static Array length
	#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

	// Register read/write/update macros using Mask/Shift:
	#define FIELD_READ(read, motor, address, mask, shift) \
		((read(motor, address) & (mask)) >> (shift))
	#define FIELD_WRITE(write, motor, address, mask, shift, value) \
		(write(motor, address, ((value)<<(shift)) & (mask)))
	#define FIELD_UPDATE(read, write, motor, address, mask, shift, value) \
		(write(motor, address, (read(motor, address) & (~(mask))) | (((value)<<(shift))&(mask))))

	#ifndef UNUSED
		#define UNUSED(x) (void)(x)
	#endif

	#ifndef NULL
		#define NULL ((void *) 0)
	#endif

#endif /* API_TYPEDEFS_H */
