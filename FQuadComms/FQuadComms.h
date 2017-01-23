/*
 * FQuadComms.h
 *
 * Created: 2017-01-20 12:54:52 AM
 *  Author: Felix
 */ 


#ifndef FQUADCOMMS_H_
#define FQUADCOMMS_H_

#include "FStatus.h"
#include "FQuadDefs.h"
#include "PlatformGPIO.h"

F_ENUM( uint8_t, FQuadCommsType_t )
{
	FQuadCommsType_Flight,
	FQuadCommsType_Controller,
};

FStatus FQuadComms_Init( const PlatformGPIO_t inSleepPin, FQuadCommsType_t inCommsType );

FStatus FQuadComms_SendControls( const FQuadAxisValue inPitch, const FQuadAxisValue inRoll, const FQuadAxisValue inYaw, const FQuadThrustValue inThrust );

FStatus FQuadComms_ReceiveControls( FQuadAxisValue *const   outPitch,
									FQuadAxisValue *const   outRoll,
									FQuadAxisValue *const   outYaw,
									FQuadThrustValue *const outThrust,
									const uint16_t          inTimeoutMs );
// Battery, RSSI
FStatus FQuadComms_ReceiveStatus( );

FStatus FQuadComms_SendStatus( );

FStatus FQuadComms_Sleep();

FStatus FQuadComms_Wake();

#endif /* FQUADCOMMS_H_ */