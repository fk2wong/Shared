/*
 * FQuadComms.c
 *
 * Created: 2017-01-20 12:55:26 AM
 *  Author: Felix
 */ 

#include "FQuadComms.h"
#include "require_macros.h"
#include "Platform_FQuadTX.h"
#include <stdbool.h>
#include <string.h>
#include <util/delay.h>

#define FQUAD_COMMS_UART_BAUD_RATE        ( 19200 )
#define FQUAD_COMMS_UART_RING_BUFFER_SIZE ( 64 )

#define UART_BITS_PER_BYTE                ( 10 )
#define FQUAD_MAX_PACKET_SEND_TIME_MS     (( uint16_t )((( uint32_t )UART_BITS_PER_BYTE * FQUAD_COMMS_MAX_PACKET_SIZE * 1000 ) / ( FQUAD_COMMS_UART_BAUD_RATE )))

#define FQUAD_COMMS_CONTROLS_LEN          ( 4 )
#define FQUAD_COMMS_FLIGHT_STATUS_LEN     ( 1 )
#define FQUAD_COMMS_DATA_TYPE_ID_LEN      ( 1 )
#define FQUAD_COMMS_MAX_TX_DATA_LEN       ( MAX( FQUAD_COMMS_CONTROLS_LEN, FQUAD_COMMS_FLIGHT_STATUS_LEN ) + FQUAD_COMMS_DATA_TYPE_ID_LEN )     
#define FQUAD_COMMS_HEADER_BYTES          ( 3 )
#define FQUAD_COMMS_CHECKSUM_BYTES        ( 1 ) 
#define FQUAD_COMMS_OVERHEAD_BYTES        ( FQUAD_COMMS_HEADER_BYTES + FQUAD_COMMS_CHECKSUM_BYTES )
#define FQUAD_COMMS_ACK_LENGTH_BYTES      ( 3 ) 
#define FQUAD_COMMS_MAX_FRAME_DATA_LEN    ( 11 + FQUAD_COMMS_MAX_TX_DATA_LEN )
#define FQUAD_COMMS_MAX_PACKET_SIZE       ( FQUAD_COMMS_OVERHEAD_BYTES + FQUAD_COMMS_MAX_FRAME_DATA_LEN ) 

#define FQUAD_COMMS_START_BYTE ( 0x7E )

#define FQUAD_COMMS_START_FRAME_ID ( 1 )

#define FQUAD_ADDRH   ( 0x13A200 )
#define FQUAD_ADDRL   ( 0x40B39D9C )
#define FQUADTX_ADDRH ( 0x13A200 )
#define FQUADTX_ADDRL ( 0x40B39D9D )

#define FQUAD_COMMS_ACK_TIMEOUT_MS ( 1000 ) // XBee Datasheet: (200 + 48ms wait) * 4

typedef struct
{
	bool                isInitialized;
	FQuadCommsType_t    commsType;
	
	PlatformRingBuffer *uartBuffer;
	uint8_t             lastFrameID;
	
	FQuadAxisValue      latestPitch;
	FQuadAxisValue      latestRoll;
	FQuadAxisValue      latestYaw;
	FQuadThrustValue    latestThrust;
	
	FQuadBatteryLevel   latestBatteryLevel;
	FStatus             latestFlightRSSI;
	FStatus             latestControllerRSSI;
	
	uint8_t             latestACKFrameID;
	uint8_t             latestACKStatus;
	
} FQuadCommsInfoStruct_t;

F_ENUM( uint8_t, FQuadCommsCmd_t )
{
	FQuadCommsCmd_TX64Bit  = 0x00,
	FQuadCommsCmd_TXStatus = 0x89,
	FQuadCommsCmd_RX64Bit  = 0x80,
};

F_ENUM( uint8_t, FQuadCommsDataType_t )
{
	FQuadCommsDataType_Controls,
	FQuadCommsDataType_FlightStatus,
};

F_ENUM( uint8_t, FQuadCommsTXOptions )
{
	FQuadCommsTXOptions_None           = 0x00,
	FQuadCommsTXOptions_DisableACK     = 0x01,
	FQuadCommsTXOptions_BroadcastPanID = 0x04,
};

F_ENUM( uint8_t, FQuadCommsRXOptions )
{
	FQuadCommsRXOptions_None             = 0x00,
	FQuadCommsRXOptions_AddressBroadcast = 0x02,
	FQuadCommsRXOptions_PanBroadcast     = 0x04,
};

F_ENUM( uint8_t, FQuadCommsTXStatus )
{
	FQuadCommsTXStatus_Success = 0,
	FQuadCommsTXStatus_NACK    = 1,
	FQuadCommsTXStatus_CCAFail = 2,
	FQuadCommsTXStatus_Purged  = 3,
};

typedef struct __attribute__ (( packed ))
{
	FQuadCommsCmd_t     cmd;  
	uint32_t            sourceADDRH;
	uint32_t            sourceADDRL;
	uint8_t             RSSI;
	FQuadCommsRXOptions options;
	uint8_t             data[FQUAD_COMMS_MAX_TX_DATA_LEN];
} FQuadCommsRXFrameData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadCommsCmd_t     cmd;
	uint8_t             frameID;
	uint32_t            destADDRH;
	uint32_t            destADDRL;
	FQuadCommsTXOptions options;
	uint8_t             data[FQUAD_COMMS_MAX_TX_DATA_LEN];
} FQuadCommsTXFrameData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadCommsCmd_t    cmd;
	uint8_t            frameID;
	FQuadCommsTXStatus status;
} FQuadCommsTXACKData_t;

typedef struct __attribute__ (( packed ))
{
	uint8_t  startByte;
	uint16_t frameLength;
	uint8_t  frameData[FQUAD_COMMS_MAX_FRAME_DATA_LEN];
	uint8_t  checksum;
} FQuadCommsPacket_t;

static FQuadCommsInfoStruct_t mCommsInfoStruct;


static FStatus _FQuadComms_SerializePacket( FQuadCommsPacket_t *const outPacket, void *const inData, size_t inDataLen );
static FStatus _FQuadComms_GetChecksum(  void *const inData, size_t inDataLen, uint8_t *const outChecksum );
static FStatus _FQuadComms_SendPacket( FQuadCommsPacket_t *const inPacket );


static FStatus _FQuadComms_RetrieveAndSortAllNewPackets( bool *const wasACKReceived, bool *const wasDataReceived );
static FStatus _FQuadComms_SortPacket( uint8_t *const inFrameData, uint16_t inFrameLength, FQuadCommsCmd_t *const outSortedPacketCmd );
static FStatus _FQuadComms_WaitForAck( uint16_t inTimeoutMs );

FStatus FQuadComms_Init( const PlatformGPIO_t inSleepPin, FQuadCommsType_t inCommsType )
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
	
	// Initialize timer for timeouts. May be already intialized by other modules.
	platformStatus = PlatformTimer_Init();
	require(( platformStatus == PlatformStatus_Success ) || ( platformStatus == PlatformStatus_AlreadyInitialized ), exit );
	
	// Determine whether this is the flight or controller side
	mCommsInfoStruct.commsType = inCommsType;
	
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
	FQuadCommsTXFrameData_t txCmdData;
	FQuadCommsPacket_t    txPacket;
	
	// Fill TX command specific data
	txCmdData.cmd       = FQuadCommsCmd_TX64Bit;
	txCmdData.frameID   = ++mCommsInfoStruct.lastFrameID;
	txCmdData.destADDRH = HTONL( FQUAD_ADDRH );
	txCmdData.destADDRL = HTONL( FQUAD_ADDRL );
	txCmdData.data[0]   = FQuadCommsDataType_Controls;
	txCmdData.data[1]   = inPitch;
	txCmdData.data[2]   = inRoll;
	txCmdData.data[3]   = inYaw;
	txCmdData.data[4]   = inThrust;
	txCmdData.options   = FQuadCommsTXOptions_None;
	
	// Create packet
	status = _FQuadComms_SerializePacket( &txPacket, &txCmdData, sizeof( FQuadCommsTXFrameData_t ));
	require_noerr( status, exit );
	
	// Send the packet
	status = _FQuadComms_SendPacket( &txPacket );
	require_noerr( status, exit );
	
	// Wait for ACK TODO
	status = _FQuadComms_WaitForAck( FQUAD_COMMS_ACK_TIMEOUT_MS );
	
	status = FStatus_Success;
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


static FStatus _FQuadComms_SerializePacket( FQuadCommsPacket_t *const outPacket, void *const inData, size_t inDataLen )
{
	FStatus status = FStatus_Failed;
	
	require_action( outPacket, exit, status = FStatus_InvalidArgument );
	require_action( inData, exit , status = FStatus_InvalidArgument );
	require_action( inDataLen, exit, status = FStatus_InvalidArgument );
	
	// Fill packet info
	outPacket->startByte = FQUAD_COMMS_START_BYTE;
	outPacket->frameLength    = HTONS( inDataLen );
	memcpy( outPacket->frameData, inData, inDataLen ); 
	
	// Get packet checksum
	status = _FQuadComms_GetChecksum( inData, inDataLen, &outPacket->checksum );
	
exit:
	return status;	
}

static FStatus _FQuadComms_GetChecksum( void *const inFrameData, size_t inDataLen, uint8_t *const outChecksum )
{
	FStatus status = FStatus_Failed;
	uint8_t checksum = 0;
	
	require_action( inFrameData, exit , status = FStatus_InvalidArgument );
	require_action( inDataLen, exit, status = FStatus_InvalidArgument );
	require_action( outChecksum, exit, status = FStatus_InvalidArgument );
	
	// Sum all the bytes
	for ( uint8_t i = 0; i < inDataLen; i++ )
	{
		checksum += (( uint8_t* )inFrameData )[i];
	}
	
	// Subtract from 0xFF
	*outChecksum = 0xFF - checksum;
	
	status = FStatus_Success;
exit:
	return status;	
}

static FStatus _FQuadComms_SendPacket( FQuadCommsPacket_t *const inPacket )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	uint8_t header[FQUAD_COMMS_HEADER_BYTES];
	
	require_action( inPacket, exit, status = FStatus_InvalidArgument );
	
	// Send the packet header
	memcpy( header, inPacket, FQUAD_COMMS_HEADER_BYTES );
	
	platformStatus = PlatformUART_Transmit( header, FQUAD_COMMS_HEADER_BYTES );
	require_noerr( platformStatus, exit );
	
	// Send the command data
	platformStatus = PlatformUART_Transmit( inPacket->frameData, inPacket->frameLength );
	require_noerr( platformStatus, exit );
	
	// Send checksum
	platformStatus = PlatformUART_Transmit( &inPacket->checksum, FQUAD_COMMS_CHECKSUM_BYTES );
	require_noerr( platformStatus, exit );
	
	status = FStatus_Success;
exit:
	return status;
}

static FStatus _FQuadComms_GetNextReceivedPacket( FQuadCommsPacket_t *const outPacket )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	uint8_t rawData[FQUAD_COMMS_MAX_PACKET_SIZE];
	uint8_t calculatedChecksum;
	
	uint16_t packetLength;
	uint16_t frameLengthRaw;
	
	require_action( outPacket, exit, status = FStatus_InvalidArgument );
	
	// First get enough data to read the length of packet
	platformStatus = PlatformUART_Receive( rawData, FQUAD_COMMS_HEADER_BYTES );
	require( platformStatus, exit );
		
	// If we received something, the first byte should be a start byte, since nothing else is using the UART
	require( rawData[0] == FQUAD_COMMS_START_BYTE, exit );
			
	// Find the remaining packet length
	frameLengthRaw = *( uint16_t* )&rawData[1];
	packetLength   = NTOHS( frameLengthRaw ) + FQUAD_COMMS_OVERHEAD_BYTES;
			
	// Get the rest of the packet, which is the frame + checksum
	platformStatus = PlatformUART_Receive( &rawData[FQUAD_COMMS_HEADER_BYTES], packetLength - FQUAD_COMMS_HEADER_BYTES );
	
	// If the receive failed, then the packet may still be transmitting over UART. 
	// Tr waiting a sufficient amount of time for the rest of the packet to be received
	if ( platformStatus != PlatformStatus_Success )
	{
		// TODO change this to a timer function if keeping a delay?
		_delay_ms( FQUAD_MAX_PACKET_SEND_TIME_MS * 2 );	
		
		// Try receiving again
		platformStatus = PlatformUART_Receive( &rawData[FQUAD_COMMS_HEADER_BYTES], packetLength - FQUAD_COMMS_HEADER_BYTES );
		require_noerr( platformStatus, exit );
	}
			
	// Check the checksum of the frame (don't include checksum byte in the checksum)
	_FQuadComms_GetChecksum( &rawData[FQUAD_COMMS_HEADER_BYTES], packetLength - FQUAD_COMMS_OVERHEAD_BYTES, &calculatedChecksum );
			
	// Check the checksum matches what we received
	require( calculatedChecksum == rawData[ FQUAD_COMMS_HEADER_BYTES + packetLength ], exit );
			
	// Copy the data into the outPacket
	outPacket->startByte   = rawData[0];
	outPacket->frameLength = packetLength - FQUAD_COMMS_OVERHEAD_BYTES;
	outPacket->checksum    = calculatedChecksum;
	memcpy( outPacket->frameData, &rawData[FQUAD_COMMS_HEADER_BYTES], outPacket->frameLength );
		
	status = FStatus_Success;
exit:
	return status;
}

static FStatus _FQuadComms_RetrieveAndSortAllNewPackets( bool *const wasACKReceived, bool *const wasDataReceived )
{
	FStatus status = FStatus_Failed;
	bool areThereNewPackets;
	
	FQuadCommsPacket_t newPacket;
	FQuadCommsCmd_t    sortedPacketCmd;
	
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
		status = _FQuadComms_GetNextReceivedPacket( &newPacket );
		
		// If a packet was received, sort it
		if ( status == FStatus_Success )
		{			
			status = _FQuadComms_SortPacket( newPacket.frameData, newPacket.frameLength, &sortedPacketCmd );
			require_noerr( status, exit );
			
			if (( sortedPacketCmd == FQuadCommsCmd_TXStatus ) && ( wasACKReceived != NULL ))
			{
				*wasACKReceived = true;
			} 
			else if (( sortedPacketCmd == FQuadCommsCmd_RX64Bit ) && ( wasDataReceived != NULL ))
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

static FStatus _FQuadComms_SortPacket( uint8_t *const inFrameData, uint16_t inFrameLength, FQuadCommsCmd_t *const outSortedPacketCmd )
{
	FStatus status = FStatus_Failed;
	
	FQuadCommsRXFrameData_t *rxFrameData;
	FQuadCommsTXACKData_t   *txACKData;
	
	require_action( inFrameData, exit, status = FStatus_InvalidArgument );
	require_action( inFrameLength, exit, status = FStatus_InvalidArgument );
	require_action( outSortedPacketCmd, exit, status = FStatus_InvalidArgument );
	
	switch ( inFrameData[0] )
	{

		case FQuadCommsCmd_TXStatus:
		{
			txACKData = ( FQuadCommsTXACKData_t* )inFrameData;
			
			mCommsInfoStruct.latestACKFrameID = txACKData->frameID;
			mCommsInfoStruct.latestACKStatus  = txACKData->status;
			
			*outSortedPacketCmd = FQuadCommsCmd_TXStatus;
			break;
		}
		case FQuadCommsCmd_RX64Bit:
		{
			// Cast it to the RX Frame type
			rxFrameData = ( FQuadCommsRXFrameData_t* )inFrameData;
			
			if ( rxFrameData->data[0] == FQuadCommsDataType_Controls )
			{
				mCommsInfoStruct.latestPitch      = rxFrameData->data[1];
				mCommsInfoStruct.latestRoll       = rxFrameData->data[2];
				mCommsInfoStruct.latestYaw        = rxFrameData->data[3];
				mCommsInfoStruct.latestThrust     = rxFrameData->data[4];
				mCommsInfoStruct.latestFlightRSSI = rxFrameData->RSSI; // Since this packet should only be received by the flight side
				
				*outSortedPacketCmd = FQuadCommsCmd_RX64Bit;
			}
			else if ( rxFrameData->data[0] == FQuadCommsDataType_FlightStatus )
			{
				mCommsInfoStruct.latestBatteryLevel   = rxFrameData->data[1];
				mCommsInfoStruct.latestFlightRSSI     = rxFrameData->data[2];
				mCommsInfoStruct.latestControllerRSSI = rxFrameData->RSSI; // since this packet should only be received by the controller side
				
				*outSortedPacketCmd = FQuadCommsCmd_RX64Bit;
			}
			else
			{
				goto exit;
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
			// Ensure this ACK matches the current frame
			if ( mCommsInfoStruct.latestACKFrameID == mCommsInfoStruct.lastFrameID )
			{
				ackReceived = true;
			}
			else
			{
				ackReceived = false;
			}
		} 
		
		status = PlatformTimer_GetTime( &currentTime );
		require_noerr( status, exit );
	}
	
	status = ( ackReceived ) ? FStatus_Success : FStatus_Failed;
exit:
	return status;
}