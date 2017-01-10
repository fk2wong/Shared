// FStatus.h

#pragma once

#include "FUtilities.h"

F_ENUM( uint8_t, FStatus )
{
	FStatus_Success            = 0,
	FStatus_Failed             = 1,
	FStatus_AlreadyInitialized = 2,
	FStatus_NotInitialized     = 3,
};