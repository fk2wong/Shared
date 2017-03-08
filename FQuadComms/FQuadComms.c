/*
 * FQuadComms.c
 *
 * Created: 2017-01-20 12:55:26 AM
 *  Author: Felix
 */ 

#include "FQuadComms.h"
#include "FQuadRF.h"
#include "require_macros.h"
#include "Platform_FQuadTX.h"
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>

#define FQUAD_COMMS_CONTROLS_MSG_LEN ( 5 )
#define FQUAD_COMMS_FLIGHT_STATUS_MSG_LEN ( 3 )
#define FQUAD_COMMS_MAX_MSG_LEN      ( 5 )

#define FQUAD_ADDRH   ( 0x13A200 )
#define FQUAD_ADDRL   ( 0x40B39D9C )
#define FQUADTX_ADDRH ( 0x13A200 )
#define FQUADTX_ADDRL ( 0x40B39D9D )

#define FQUAD_COMMS_START_FRAME_ID ( 0x01 )
#define FQUAD_COMMS_MAX_FRAME_ID   ( 0xFF )

#define FQUAD_COMMS_ACK_TIMEOUT_MS ( 1000 ) // XBee Datasheet: (200 + 48ms wait) * 4 

//===============================//
//           Typedefs            //
//===============================//

F_ENUM( uint8_t, FQuadCommsDataType_t )
{
	FQuadCommsDataType_Controls,
	FQuadCommsDataType_FlightStatus,
};

typedef struct
{
	bool                isInitialized;
	
	FQuadAxisValue      latestPitch;
	FQuadAxisValue      latestRoll;
	FQuadAxisValue      latestYaw;
	FQuadThrustValue    latestThrust;
	
	FQuadBatteryLevel   latestBatteryLevel;
	FQuadRSSI           latestFlightRSSI;
	FQuadRSSI           latestControllerRSSI;
	
	uint8_t             lastFrameID;          // Keep track of the frame #, increments per msg.
	uint8_t             latestACKStatus;
	volatile bool       ackReceived;
	
} FQuadCommsInfoStruct_t;

typedef struct __attribute__ (( packed ))
{
	FQuadAxisValue       pitch;
	FQuadAxisValue       roll;
	FQuadAxisValue       yaw;
	FQuadThrustValue     thrust;
} FQuadCommsControlsData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadBatteryLevel batteryLevel;
	FQuadRSSI         RSSI;
} FQuadCommsFlightStatusData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadCommsDataType_t type;
	union
	{
		FQuadCommsControlsData_t     controls;
		FQuadCommsFlightStatusData_t flightStatus;
	} msgData;
} FQuadCommsMsg_t;

static FQuadCommsInfoStruct_t mCommsInfoStruct;

//================================//
// Internal Function Declarations //
//================================//

static FStatus _FQuadComms_WaitForAck( uint16_t inTimeoutMs );

void _FQuadComms_ACKReceivedISR( const uint8_t inFrameID, const FQuadRFTXStatus inACKStatus );
void _FQuadComms_MsgReceivedISR( uint8_t *const inMsg, const size_t inMsgLen, const FQuadRSSI inRSSI );

//=============================//
// Public Function Definitions //
//=============================//

FStatus FQuadComms_Init( const PlatformGPIO_t inSleepPin )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	
	// Check if already initialized
	require_action( !mCommsInfoStruct.isInitialized, exit, status = FStatus_AlreadyInitialized );
	
	// Initialize the RF module
	platformStatus = FQuadRF_Init( inSleepPin, _FQuadComms_MsgReceivedISR, _FQuadComms_ACKReceivedISR );
	require_noerr( platformStatus, exit );
	
	// Initialize timer for timeouts. May be already intialized by other modules.
	platformStatus = PlatformTimer_Init();
	require(( platformStatus == PlatformStatus_Success ) || ( platformStatus == PlatformStatus_AlreadyInitialized ), exit );
	
	// Initialize frame IDs, for matching ACKs
	mCommsInfoStruct.lastFrameID = FQUAD_COMMS_START_FRAME_ID;
	
	// This must be initialized as zero, since it will act as binary semaphore when we wait for an ACK
	mCommsInfoStruct.ackReceived = false;
	
	// Initialization complete
	mCommsInfoStruct.isInitialized = true;
	
	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadComms_SendControls( const FQuadAxisValue inPitch, const FQuadAxisValue inRoll, const FQuadAxisValue inYaw, const FQuadThrustValue inThrust )

{
	FStatus status = FStatus_Failed;
	FQuadCommsMsg_t msg;
	
	// Make sure the module is initialized
	require_action( mCommsInfoStruct.isInitialized, exit, status = FStatus_NotInitialized );
	
	// Structure the data into a message, with the first byte as the type of message 
	msg.type                    = FQuadCommsDataType_Controls;
	msg.msgData.controls.pitch  = inPitch;
	msg.msgData.controls.roll   = inRoll;
	msg.msgData.controls.yaw    = inYaw;
	msg.msgData.controls.thrust = inThrust;
	
	// Handle overflow of the frame ID
	if ( mCommsInfoStruct.lastFrameID == FQUAD_COMMS_MAX_FRAME_ID )
	{
		mCommsInfoStruct.lastFrameID = FQUAD_COMMS_START_FRAME_ID;
	}
	else
	{
		// Increment the frame ID
		mCommsInfoStruct.lastFrameID++;
	}
	
	// Clear ACK status before we send the message
	mCommsInfoStruct.ackReceived = false;
	
	// Send message
	status = FQuadRF_SendMessage(( uint8_t* )&msg, FQUAD_COMMS_CONTROLS_MSG_LEN, mCommsInfoStruct.lastFrameID, HTONL( FQUAD_ADDRH ), HTONL( FQUAD_ADDRL ));
	require_noerr( status, exit );
	
	// Wait for ACK
	status = _FQuadComms_WaitForAck( FQUAD_COMMS_ACK_TIMEOUT_MS );
	require_noerr( status, exit );
	
exit:
	return status;
}


FStatus FQuadComms_SendFlightBatteryLevelAndRSSI( const FQuadBatteryLevel inBatteryLevel, const FQuadRSSI inFlightRSSI )
{
	FStatus status = FStatus_Failed;
	FQuadCommsMsg_t msg;
	
	// Make sure the module is initialized
	require_action( mCommsInfoStruct.isInitialized, exit, status = FStatus_NotInitialized );
	
	// Structure the data into a message, with the first byte as the type of message
	msg.type                              = FQuadCommsDataType_FlightStatus;
	msg.msgData.flightStatus.batteryLevel = inBatteryLevel;
	msg.msgData.flightStatus.RSSI         = inFlightRSSI;
	
	// Handle overflow of the frame ID
	if ( mCommsInfoStruct.lastFrameID == FQUAD_COMMS_MAX_FRAME_ID )
	{
		mCommsInfoStruct.lastFrameID = FQUAD_COMMS_START_FRAME_ID;
	}
	else
	{
		// Increment the frame ID
		mCommsInfoStruct.lastFrameID++;
	}
	
	// Clear ACK status before we send the message
	mCommsInfoStruct.ackReceived = false;
	
	// Send message
	status = FQuadRF_SendMessage(( uint8_t* )&msg, FQUAD_COMMS_FLIGHT_STATUS_MSG_LEN, mCommsInfoStruct.lastFrameID, HTONL( FQUAD_ADDRH ), HTONL( FQUAD_ADDRL ));
	require_noerr( status, exit );
	
	// Wait for ACK
	status = _FQuadComms_WaitForAck( FQUAD_COMMS_ACK_TIMEOUT_MS );
	require_noerr( status, exit );
	
exit:
	return status;
}

FStatus FQuadComms_ReceiveControls( FQuadAxisValue *const   outPitch, 
                                    FQuadAxisValue *const   outRoll, 
									FQuadAxisValue *const   outYaw, 
									FQuadThrustValue *const outThrust )
{
	FStatus status = FStatus_InvalidArgument;
	
	// Make sure the module is initialized
	require_action( mCommsInfoStruct.isInitialized, exit, status = FStatus_NotInitialized );
	
	// Output battery level
	*outPitch   = mCommsInfoStruct.latestPitch;
	*outRoll    = mCommsInfoStruct.latestRoll;
	*outYaw     = mCommsInfoStruct.latestYaw;
	*outThrust  = mCommsInfoStruct.latestThrust;

	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadComms_GetLatestFlightBatteryLevel( FQuadBatteryLevel *const outBatteryLevel )
{
	FStatus status = FStatus_InvalidArgument;
	
	// Make sure the module is initialized
	require_action( mCommsInfoStruct.isInitialized, exit, status = FStatus_NotInitialized );
	
	// Output battery level
	*outBatteryLevel  = mCommsInfoStruct.latestBatteryLevel;

	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadComms_GetLatestRSSI( FQuadRSSI *const outControllerRSSI, FQuadRSSI *const outFlightRSSI )
{
	FStatus status = FStatus_InvalidArgument;
	
	// Make sure the module is initialized
	require_action( mCommsInfoStruct.isInitialized, exit, status = FStatus_NotInitialized );
	
	if ( outControllerRSSI != NULL )
	{
		*outControllerRSSI = mCommsInfoStruct.latestControllerRSSI;
	}
	if ( outFlightRSSI != NULL )
	{
		*outFlightRSSI = mCommsInfoStruct.latestFlightRSSI;
	}

	status = FStatus_Success;
exit:
	return status;
}

inline FStatus FQuadComms_Sleep( void )
{	
	return FQuadRF_Sleep();
}

inline FStatus FQuadComms_Wake( void )
{	
	return FQuadRF_Wake();
}

//===============================//
// Internal Function Definitions //
//===============================//

static FStatus _FQuadComms_WaitForAck( uint16_t inTimeoutMs )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	
	uint32_t startTime;
	uint32_t currentTime;
	
	platformStatus = PlatformTimer_GetTime( &currentTime );
	require_noerr( platformStatus, exit );
	
	startTime = currentTime;
	
	// Loop until timed out, or ACK received
	while ( ( uint16_t )( currentTime - startTime ) <= inTimeoutMs )
	{		
		if ( mCommsInfoStruct.ackReceived )
		{
			// return status of the ACK
			status = ( mCommsInfoStruct.latestACKStatus == FQuadRFTXStatus_Success ) ? FStatus_Success : FStatus_Failed;
			
			// Reset ackReceived flag
			mCommsInfoStruct.ackReceived = false;
			break;
		} 
		
		// Check current time, for timeout purposes
		platformStatus = PlatformTimer_GetTime( &currentTime );
		require_noerr( platformStatus, exit );
	}
	
exit:
	return status;
}

void _FQuadComms_MsgReceivedISR( uint8_t *const inMsg, const size_t inMsgLen, const FQuadRSSI inRSSI )
{
	FQuadCommsMsg_t *msg;
	
	require_noerr_quiet( inMsg, exit );
	require_noerr_quiet( inMsgLen, exit );
	
	// Cast to appropriate type
	msg = ( FQuadCommsMsg_t* )inMsg;
	
	switch ( msg->type )
	{
		case FQuadCommsDataType_Controls:
		{
			// Message sent from controller to flight
			mCommsInfoStruct.latestPitch      = msg->msgData.controls.pitch;
			mCommsInfoStruct.latestRoll       = msg->msgData.controls.roll;
			mCommsInfoStruct.latestYaw        = msg->msgData.controls.yaw;
			mCommsInfoStruct.latestThrust     = msg->msgData.controls.thrust;
			mCommsInfoStruct.latestFlightRSSI = inRSSI;
			
			break;
		}
		case FQuadCommsDataType_FlightStatus:
		{
			// Message sent from flight to controller
			mCommsInfoStruct.latestBatteryLevel   = msg->msgData.flightStatus.batteryLevel;
			mCommsInfoStruct.latestFlightRSSI     = msg->msgData.flightStatus.RSSI;
			mCommsInfoStruct.latestControllerRSSI = inRSSI;
		}
	}
	
exit:
	return;
}

void _FQuadComms_ACKReceivedISR( const uint8_t inFrameID, const FQuadRFTXStatus inACKStatus )
{
	// If the ack frame matches the last one we sent
	if ( mCommsInfoStruct.lastFrameID == inFrameID )
	{
		mCommsInfoStruct.latestACKStatus = inACKStatus;
		mCommsInfoStruct.ackReceived     = true;
	}
}