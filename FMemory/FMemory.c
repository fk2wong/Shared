/*
 * FMemory.c
 *
 * Created: 2016-09-08 8:14:23 AM
 *  Author: Felix
 */ 

#include "FMemory.h"

void _FMemoryFreeAndNULLPtr( void** ioDoublePtr )
{
	if( ioDoublePtr && *ioDoublePtr )
	{
		free( *ioDoublePtr );
		
		*ioDoublePtr = NULL;
	}
}