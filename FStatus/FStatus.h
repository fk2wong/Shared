// FStatus.h

#pragma once

#include "FUtilities.h"

F_ENUM( uint8_t, FStatus )
{
	FStatusSuccess            = 0,
	FStatusFailed             = 1,
	FStatusAlreadyInitialized = 2,
	FStatusNotInitialized     = 3,
};