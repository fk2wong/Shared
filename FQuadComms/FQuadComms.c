/*
 * FQuadComms.c
 *
 * Created: 2017-01-20 12:55:26 AM
 *  Author: Felix
 */ 

#include "FQuadComms.h"
#include "require_macros.h"
#include "PlatformRingBuffer.h"
#include "PlatformUART.h"
#include <stdbool.h>

#define FQUAD_COMMS_UART_BAUD_RATE ( 19200 )
#define FQUAD_COMMS_UART_RING_BUFFER_SIZE ( 32 )

struct FQuadCommsInfoStruct
{
	bool isInitialized;
	PlatformRingBuffer *uartBuffer;
};

static struct FQuadCommsInfoStruct mCommsInfoStruct;

FStatus FQuadComms_Init( const PlatformGPIO_t inSleepPin )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	
	// Check if already initialized
	require_action( !mCommsInfoStruct.isInitialized, exit, status = FStatus_AlreadyInitialized );
	
	// Initialize ring buffer for UART
	mCommsInfoStruct.uartBuffer = PlatformRingBuffer_Create( FQUAD_COMMS_UART_RING_BUFFER_SIZE );
	require( mCommsInfoStruct.uartBuffer, exit );
	
	// Initialize UART with ring buffer
	platformStatus = PlatformUART_Init( FQUAD_COMMS_UART_BAUD_RATE, mCommsInfoStruct.uartBuffer );
	require_noerr( platformStatus, exit );
	
	// Configure sleep pin, deasserted
	platformStatus = PlatformGPIO_Configure( inSleepPin, PlatformGPIOConfig_Output );
	require_noerr( platformStatus, exit );
	
	platformStatus = PlatformGPIO_OutputLow( inSleepPin );
	require_noerr( platformStatus, exit );
	
	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadComms_SendControls( const FQuadAxisValue inPitch, const FQuadAxisValue inRoll, const FQuadAxisValue inYaw, const FQuadThrustValue inThrust )
{
	FStatus status = FStatus_Failed;
	
	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadComms_ReceiveControls( FQuadAxisValue *const outPitch, FQuadAxisValue *const outRoll, FQuadAxisValue *const outYaw, FQuadThrustValue *const outThrust )
{
	FStatus status = FStatus_Failed;
	
	status = FStatus_Success;
exit:
	return status;
}
