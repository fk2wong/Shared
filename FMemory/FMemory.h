/*
 * FMemory.h
 *
 * Created: 2016-09-08 8:08:33 AM
 *  Author: Felix
 */ 


#ifndef FMEMORY_H_
#define FMEMORY_H_

#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>

// Public functions
#define FMemoryAlloc( X )          malloc( X )
#define FMemoryFreeAndNULLPtr( X ) _FMemoryFreeAndNULLPtr(( void** ) X )

/*!
 *\brief    Returns the number of free bytes left in RAM.
 */
int16_t FMemory_GetNumFreeBytes();

// Should not be called explicitly
void _FMemoryFreeAndNULLPtr( void** ioDoublePtr );

#endif /* FMEMORY_H_ */