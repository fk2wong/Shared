/*
 * FMemory.c
 *
 * Created: 2016-09-08 8:14:23 AM
 *  Author: Felix
 */ 

#include "FMemory.h"

extern struct never_defined __bss_end;
extern void *__brkval;

void _FMemoryFreeAndNULLPtr( void** ioDoublePtr )
{
	if( ioDoublePtr && *ioDoublePtr )
	{
		free( *ioDoublePtr );
		
		*ioDoublePtr = NULL;
	}
}

int16_t FMemory_GetNumFreeBytes()
{
	int free_memory;
	
	if((int16_t)__brkval == 0)
	{
		free_memory = ((int16_t)&free_memory) - ((int16_t)&__bss_end);
	}
	else
	{
		free_memory = ((int16_t)&free_memory) - ((int16_t)__brkval);
	}
	
	return free_memory;
}