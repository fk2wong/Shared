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
#include <string.h>

#define FQUAD_COMMS_UART_BAUD_RATE ( 19200 )
#define FQUAD_COMMS_UART_RING_BUFFER_SIZE ( 64 )

#define UART_BITS_PER_BYTE ( 10 )
#define FQUAD_COMMS_PACKET_SEND_TIME_MS (( UART_BITS_PER_BYTE * FQUAD_COMMS_MAX_PACKET_SIZE * 1000 ) / ( FQUAD_COMMS_UART_BAUD_RATE )

#define FQUAD_COMMS_CONTROLS_LENGTH_BYTES ( 4 )
#define FQUAD_COMMS_HEADER_BYTES          ( 3 )
#define FQUAD_COMMS_CHECKSUM_BYTES        ( 1 ) 
#define FQUAD_COMMS_OVERHEAD_BYTES        ( FQUAD_COMMS_HEADER_BYTES + FQUAD_COMMS_CHECKSUM_BYTES )
#define FQUAD_COMMS_ACK_LENGTH_BYTES      ( 3 ) 
#define FQUAD_COMMS_MAX_FRAME_DATA_LEN    ( 11 + FQUAD_COMMS_CONTROLS_LENGTH_BYTES )
#define FQUAD_COMMS_MAX_PACKET_SIZE       ( FQUAD_COMMS_OVERHEAD_BYTES + FQUAD_COMMS_MAX_FRAME_DATA_LEN ) 

#define FQUAD_COMMS_START_BYTE ( 0x7E )

#define FQUAD_ADDRH   ( 0x13A200 )
#define FQUAD_ADDRL   ( 0x40B39D9C )
#define FQUADTX_ADDRH ( 0x13A200 )
#define FQUADTX_ADDRL ( 0x40B39D9D )

#define FQUAD_COMMS_ACK_TIMEOUT_MS ( 1000 ) // XBee Datasheet: (200 + 48ms wait) * 4

typedef struct
{
	bool isInitialized;
	PlatformRingBuffer *uartBuffer;
	uint8_t lastFrameID;
} FQuadCommsInfoStruct_t;

F_ENUM( uint8_t, FQuadCommsCmd_t )
{
	FQuadCommsCmd_TX64Bit  = 0x00,
	FQuadCommsCmd_TXStatus = 0x89,
	FQuadCommsCmd_RX64Bit  = 0x80,
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
	uint8_t             data[FQUAD_COMMS_CONTROLS_LENGTH_BYTES];
} FQuadCommsRXFrameData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadCommsCmd_t     cmd;
	uint8_t             frameID;
	uint32_t            destADDRH;
	uint32_t            destADDRL;
	FQuadCommsTXOptions options;
	uint8_t             data[FQUAD_COMMS_CONTROLS_LENGTH_BYTES];
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
	uint16_t length;
	uint8_t  *frameData;
	uint8_t  checksum;
} FQuadCommsPacket_t;

static FQuadCommsInfoStruct_t mCommsInfoStruct;


static FStatus _FQuadComms_SerializePacket( FQuadCommsPacket_t *const outPacket, void *const inData, size_t inDataLen, const FQuadCommsCmd_t inCmdType );
static FStatus _FQuadComms_GetChecksum(  void *const inData, size_t inDataLen, uint8_t *const outChecksum );
static FStatus _FQuadComms_SendPacket( FQuadCommsPacket_t *const inPacket );

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
	FQuadCommsTXFrameData_t txCmdData;
	FQuadCommsPacket_t    txPacket;
	
	// Fill TX command specific data
	txCmdData.cmd       = FQuadCommsCmd_TX64Bit;
	txCmdData.frameID   = ++mCommsInfoStruct.lastFrameID;
	txCmdData.destADDRH = HTONL( FQUAD_ADDRH );
	txCmdData.destADDRL = HTONL( FQUAD_ADDRL );
	txCmdData.data[0]   = inPitch;
	txCmdData.data[1]   = inRoll;
	txCmdData.data[2]   = inYaw;
	txCmdData.data[3]   = inThrust;
	txCmdData.options   = FQuadCommsTXOptions_None;
	
	// Create packet
	status = _FQuadComms_SerializePacket( &txPacket, &txCmdData, sizeof( FQuadCommsTXFrameData_t ));
	require_noerr( status, exit );
	
	// Send the packet
	status = _FQuadComms_SendPacket( &txPacket );
	require_noerr( status, exit );
	
	// Wait for ACK
	status = _FQuadComms_WaitForAck( &txPacket );
	
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


static FStatus _FQuadComms_SerializePacket( FQuadCommsPacket_t *const outPacket, void *const inData, size_t inDataLen )
{
	FStatus status = FStatus_Failed;
	
	require_action( outPacket, exit, status = FStatus_InvalidArgument );
	require_action( inData, exit , status = FStatus_InvalidArgument );
	require_action( inDataLen, exit, status = FStatus_InvalidArgument );
	
	// Fill packet info
	outPacket->startByte = FQUAD_COMMS_START_BYTE;
	outPacket->length    = HTONS( inDataLen );
	outPacket->frameData = inData; 
	
	// Get packet checksum
	_FQuadComms_GetChecksum( inData, inDataLen,  &outPacket->checksum );
	
	status = FStatus_Success;
exit:
	return status;	
}

static FStatus _FQuadComms_GetChecksum( void *const inData, size_t inDataLen, uint8_t *const outChecksum )
{
	FStatus status = FStatus_Failed;
	uint8_t checksum = 0;
	
	require_action( inData, exit , status = FStatus_InvalidArgument );
	require_action( inDataLen, exit, status = FStatus_InvalidArgument );
	require_action( outChecksum, exit, status = FStatus_InvalidArgument );
	
	// Sum all the bytes
	for ( uint8_t i = 0; i < inDataLen; i++ )
	{
		checksum += (( uint8_t* )inData )[i];
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
	platformStatus = PlatformUART_Transmit( inPacket->frameData, inPacket->length );
	require_noerr( platformStatus, exit );
	
	// Send checksum
	platformStatus = PlatformUART_Transmit( &inPacket->checksum, FQUAD_COMMS_CHECKSUM_BYTES );
	require_noerr( platformStatus, exit );
	
	status = FStatus_Success;
exit:
	return status;
}

static FStatus _FQuadComms_WaitForAck( FQuadCommsPacket_t *const inPacket )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	uint8_t rawData[FQUAD_COMMS_MAX_PACKET_SIZE];
	bool ackReceived = false;
	
	uint16_t packetLength;
	uint16_t frameLengthRaw;
	
	require_action( inPacket, exit, status = FStatus_InvalidArgument );
	
	// Get the raw packet
	while( )
	{
		// First get enough data to read the length of packet
		platformStatus = PlatformUART_Receive( rawData, FQUAD_COMMS_HEADER_BYTES );
		
		// Continue only if we received something
		if ( platformStatus == PlatformStatus_Success )
		{
			// The first byte should be a stat byte, since nothing else is using the UART
			require( rawData[0] == FQUAD_COMMS_START_BYTE, exit );
			
			// Wait for the rest of the packet to be received
			_delay( FQUAD_COMMS_PACKET_SEND_TIME_MS );
			
			// Find the remaining packet length
			frameLengthRaw = *(uint16_t*)&rawData[1]
			packetLength = NTOHS( frameLengthRaw ) + FQUAD_COMMS_OVERHEAD_BYTES;
			
			// Get the rest of the packet TODO 
			while ( )
			platformStatus = PlatformUART_Receive( rawData, packetLength - FQUAD_COMMS_HEADER_BYTES );
			require_noerr( platformStatus, exit );
		}
	}
	
	
	
	status = FStatus_Success;
exit:
	return status;
}