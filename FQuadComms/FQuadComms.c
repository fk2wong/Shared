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

#define FQUAD_COMMS_START_FRAME_ID ( 1 )

#define FQUAD_COMMS_ACK_TIMEOUT_MS ( 1000 ) // XBee Datasheet: (200 + 48ms wait) * 4

typedef struct
{
	bool                isInitialized;
	
	uint8_t             lastFrameID;
	
	FQuadAxisValue      latestPitch;
	FQuadAxisValue      latestRoll;
	FQuadAxisValue      latestYaw;
	FQuadThrustValue    latestThrust;
	
	FQuadBatteryLevel   latestBatteryLevel;
	FQuadRSSI           latestFlightRSSI;
	FQuadRSSI           latestControllerRSSI;
	
	uint8_t             latestACKFrameID;
	uint8_t             latestACKStatus;
	
} FQuadCommsInfoStruct_t;

F_ENUM( uint8_t, FQuadCommsDataType_t )
{
	FQuadCommsDataType_Controls,
	FQuadCommsDataType_FlightStatus,
};

static FQuadCommsInfoStruct_t mCommsInfoStruct;

static FStatus _FQuadComms_RetrieveAndSortAllNewPackets( bool *const wasACKReceived, bool *const wasDataReceived );
static FStatus _FQuadComms_SortMessage( uint8_t *const inData, uint16_t inDataLength, FQuadRFCmdID_t inCmdID );
static FStatus _FQuadComms_WaitForAck( uint16_t inTimeoutMs );

FStatus FQuadComms_Init( const PlatformGPIO_t inSleepPin )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	
	// Check if already initialized
	require_action( !mCommsInfoStruct.isInitialized, exit, status = FStatus_AlreadyInitialized );
	
	// Initialize timer for timeouts. May be already intialized by other modules.
	platformStatus = PlatformTimer_Init();
	require(( platformStatus == PlatformStatus_Success ) || ( platformStatus == PlatformStatus_AlreadyInitialized ), exit );
	
	// Initialize frame IDs, for matching ACKs
	mCommsInfoStruct.lastFrameID      = FQUAD_COMMS_START_FRAME_ID;
	mCommsInfoStruct.latestACKFrameID = FQUAD_COMMS_START_FRAME_ID - 1;
	
	mCommsInfoStruct.isInitialized = true;
	
	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadComms_SendControls( const FQuadAxisValue inPitch, const FQuadAxisValue inRoll, const FQuadAxisValue inYaw, const FQuadThrustValue inThrust )
{
	FStatus status = FStatus_Failed;
	uint8_t msgData[FQUAD_COMMS_CONTROLS_MSG_LEN];
	
	// Structure the data into a message, with the first byte as the type of message 
	msgData[0] = FQuadCommsDataType_Controls;
	msgData[1] = inPitch;
	msgData[2] = inRoll;
	msgData[3] = inYaw;
	msgData[4] = inThrust;
	
	// Send message
	status = FQuadRF_SendMessage( msgData, sizeof( msgData ), ++mCommsInfoStruct.lastFrameID, HTONL( FQUAD_ADDRH ), HTONL( FQUAD_ADDRL ));
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
	uint8_t msgData[FQUAD_COMMS_FLIGHT_STATUS_MSG_LEN];
	
	// Structure the data into a message, with the first byte as the type of message
	msgData[0] = FQuadCommsDataType_FlightStatus;
	msgData[1] = inBatteryLevel;
	msgData[2] = inFlightRSSI;
	
	// Send message
	status = FQuadRF_SendMessage( msgData, sizeof( msgData ), ++mCommsInfoStruct.lastFrameID, HTONL( FQUAD_ADDRH ), HTONL( FQUAD_ADDRL ));
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
									FQuadThrustValue *const outThrust,
									const uint16_t          inTimeoutMs )
{
	FStatus status;
	bool controlsReceived = false;
	
	uint64_t startTime;
	uint64_t currentTime;
	
	status = PlatformTimer_GetTime( &currentTime );
	require_noerr( status, exit );
	
	startTime = currentTime;
	
	// Loop until controls received, or until timed out
	while ( ( uint16_t )(( currentTime - startTime ) <= inTimeoutMs ) && !controlsReceived )
	{
		status = _FQuadComms_RetrieveAndSortAllNewPackets( NULL, &controlsReceived );
		require_noerr( status, exit );
		
		if ( controlsReceived )
		{
			*outPitch  = mCommsInfoStruct.latestPitch;
			*outRoll   = mCommsInfoStruct.latestRoll;
			*outYaw    = mCommsInfoStruct.latestYaw;
			*outThrust = mCommsInfoStruct.latestThrust;
		}
		
		status = PlatformTimer_GetTime( &currentTime );
		require_noerr( status, exit );
	}
	
	status = ( controlsReceived ) ? FStatus_Success : FStatus_Failed;
exit:
	return status;
}

FStatus FQuadComms_GetLatestFlightBatteryLevel( FQuadBatteryLevel *const outBatteryLevel )
{
	FStatus status = FStatus_InvalidArgument;
	
	require( outBatteryLevel, exit );

	status = _FQuadComms_RetrieveAndSortAllNewPackets( NULL, NULL );
	require_noerr( status, exit );
	
	*outBatteryLevel  = mCommsInfoStruct.latestBatteryLevel;

exit:
	return status;
}

FStatus FQuadComms_GetLatestRSSI( FQuadRSSI *const outControllerRSSI, FQuadRSSI *const outFlightRSSI )
{
	FStatus status;

	status = _FQuadComms_RetrieveAndSortAllNewPackets( NULL, NULL );
	require_noerr( status, exit );
	
	if ( outControllerRSSI != NULL )
	{
		*outControllerRSSI = mCommsInfoStruct.latestControllerRSSI;
	}
	if ( outFlightRSSI != NULL )
	{
		*outFlightRSSI = mCommsInfoStruct.latestFlightRSSI;
	}

exit:
	return status;
}

FStatus FQuadComms_Sleep()
{	
	return FQuadRF_Sleep();
}

FStatus FQuadComms_Wake()
{	
	return FQuadRF_Wake();
}


static FStatus _FQuadComms_RetrieveAndSortAllNewPackets( bool *const wasACKReceived, bool *const wasDataReceived )
{
	FStatus status = FStatus_Failed;
	bool areThereNewPackets;
	
	uint8_t msgData[FQUAD_COMMS_MAX_MSG_LEN + 1];
	uint8_t msgLen;
	FQuadRFCmdID_t msgCmdID;
	
	// Clear the output arguments, if they exist
	if ( wasACKReceived )
	{
		*wasACKReceived  = false;
	}
	if ( wasDataReceived )
	{
		*wasDataReceived = false;
	}
	
	// Assume there are new packets to be retrieved in the buffer
	areThereNewPackets = true;

	while ( areThereNewPackets )
	{
		// Check for a new packet, timeout of 0ms ( no wait )
		status = FQuadRF_ReceiveMessage( msgData, &msgLen, &msgCmdID );
		
		// If a packet was received, sort it
		if ( status == FStatus_Success )
		{			
			status = _FQuadComms_SortMessage( msgData, msgLen, msgCmdID );
			require_noerr( status, exit );
			
			// Let the calling function know if a new ACK or if new data was received
			if (( msgCmdID = FQuadRFCmd_TXStatus ) && ( wasACKReceived != NULL ))
			{
				*wasACKReceived = true;
			}
			else if (( msgCmdID == FQuadRFCmd_RX64Bit ) && ( wasDataReceived != NULL ))
			{
				*wasDataReceived = true;
			}
		}
		else
		{
			areThereNewPackets = false;
		}
	}
	
	status = FStatus_Success;
exit:
	return status;
}

static FStatus _FQuadComms_SortMessage( uint8_t *const inData, uint16_t inDataLength, FQuadRFCmdID_t inCmdID )
{
	FStatus status = FStatus_Failed;
	
	FQuadRFRXData_t   *rxData;
	FQuadRFTXStatus_t *txACKData;
	
	require_action( inData, exit, status = FStatus_InvalidArgument );
	require_action( inDataLength, exit, status = FStatus_InvalidArgument );
	
	switch ( inCmdID )
	{

		case FQuadRFCmd_TXStatus:
		{
			// Cast data to appropriate type
			txACKData = ( FQuadRFTXStatus_t* )inData;
			
			// Save the frame ID and ACK status
			mCommsInfoStruct.latestACKFrameID = txACKData->frameID;
			mCommsInfoStruct.latestACKStatus  = txACKData->status;
			
			break;
		}
		case FQuadRFCmd_RX64Bit:
		{
			// Cast it to the RX Frame type
			rxData = ( FQuadRFRXData_t* )inData;
			
			// Save the relevant data
			if ( rxData->data[0] == FQuadCommsDataType_Controls )
			{
				mCommsInfoStruct.latestPitch      = rxData->data[1];
				mCommsInfoStruct.latestRoll       = rxData->data[2];
				mCommsInfoStruct.latestYaw        = rxData->data[3];
				mCommsInfoStruct.latestThrust     = rxData->data[4];
				mCommsInfoStruct.latestFlightRSSI = rxData->RSSI; // Since this packet should only be received by the flight side
			}
			else if ( rxData->data[0] == FQuadCommsDataType_FlightStatus )
			{
				mCommsInfoStruct.latestBatteryLevel   = rxData->data[1];
				mCommsInfoStruct.latestFlightRSSI     = rxData->data[2];
				mCommsInfoStruct.latestControllerRSSI = rxData->RSSI; // since this packet should only be received by the controller side
			}
			break;
		}
		default:
		{
			goto exit;
		}
	}
	
	status = FStatus_Success;
exit:
	return status;
}

static FStatus _FQuadComms_WaitForAck( uint16_t inTimeoutMs )
{
	FStatus status;
	bool ackReceived = false;
	bool ackMatches;
	bool ackSuccess;
	
	uint64_t startTime;
	uint64_t currentTime;
	
	status = PlatformTimer_GetTime( &currentTime );
	require_noerr( status, exit );
	
	startTime = currentTime;
	
	// Check for ACK until timed out, or ACK received
	while ( ( uint16_t )(( currentTime - startTime ) <= inTimeoutMs ) && !ackReceived )
	{
		status = _FQuadComms_RetrieveAndSortAllNewPackets( &ackReceived, NULL);
		require_noerr( status, exit );
		
		if ( ackReceived )
		{
			ackMatches = mCommsInfoStruct.latestACKFrameID == mCommsInfoStruct.lastFrameID;
			ackSuccess = mCommsInfoStruct.latestACKStatus;
			// Ensure this ACK matches the current frame
			if ( ackMatches && ackSuccess )
			{
				ackReceived = true;
			}
		} 
		
		status = PlatformTimer_GetTime( &currentTime );
		require_noerr( status, exit );
	}
	
	status = ( ackReceived ) ? FStatus_Success : FStatus_Failed;
exit:
	return status;
}