/*
 * FQuadRF.c
 *
 * Created: 2017-01-25 10:03:31 PM
 *  Author: Felix
 */ 

#include "FQuadRF.h"
#include "FUtilities.h"
#include "Platform_FQuadTX.h"
#include "require_macros.h"
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define FQUADRF_UART_BAUD_RATE        ( 19200 )
#define FQUADRF_UART_RING_BUFFER_SIZE ( 64 )

#define UART_BITS_PER_BYTE                ( 10 )
#define FQUADRF_MAX_PACKET_SEND_TIME_MS     (( uint16_t )((( uint32_t )UART_BITS_PER_BYTE * FQUADRF_MAX_PACKET_SIZE * 1000 ) / ( FQUADRF_UART_BAUD_RATE )))



#define FQUADRF_START_BYTE ( 0x7E )
#define FQUADRF_HEADER_BYTES ( 3 )
#define FQUADRF_CHECKSUM_BYTES ( 1 )
#define FQUADRF_OVERHEAD_BYTES ( FQUADRF_HEADER_BYTES + FQUADRF_CHECKSUM_BYTES )
#define FQUADRF_MAX_FRAME_SIZE ( FQUADRF_MAX_MSG_DATA_LEN + 11 )
#define FQUADRF_MAX_PACKET_SIZE ( FQUADRF_MAX_FRAME_SIZE + FQUADRF_OVERHEAD_BYTES )

#define FQUADRF_RX_MSG_NON_DATA_BYTES ( 10 )

#define FQUAD_COMMS_WAKE_TIME_MS ( 4 )

F_ENUM( uint8_t, FQuadRFTXOptions )
{
	FQuadRFTXOptions_None           = 0x00,
	FQuadRFTXOptions_DisableACK     = 0x01,
	FQuadRFTXOptions_BroadcastPanID = 0x04,
};

F_ENUM( uint8_t, FQuadRFRXOptions )
{
	FQuadRFRXOptions_None             = 0x00,
	FQuadRFRXOptions_AddressBroadcast = 0x02,
	FQuadRFRXOptions_PanBroadcast     = 0x04,
};

typedef struct __attribute__ (( packed ))
{
	FQuadRFCmdID_t      cmd;
	uint32_t            sourceADDRH;
	uint32_t            sourceADDRL;
	int8_t              RSSI;
	FQuadRFRXOptions options;
	uint8_t             data[FQUADRF_MAX_MSG_DATA_LEN];
} FQuadRFRXFrameData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadRFCmdID_t      cmd;
	uint8_t             frameID;
	uint32_t            destADDRH;
	uint32_t            destADDRL;
	FQuadRFTXOptions options;
	uint8_t             data[FQUADRF_MAX_MSG_DATA_LEN];
} FQuadRFTXFrameData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadRFCmdID_t     cmd;
	uint8_t            frameID;
	FQuadRFTXStatus status;
} FQuadRFTXACKData_t;

typedef struct __attribute__ (( packed ))
{
	uint8_t  startByte;
	uint16_t frameLength;
	uint8_t  frameData[FQUADRF_MAX_FRAME_SIZE];
	uint8_t  checksum;
} FQuadCommsPacket_t;

typedef struct 
{
	bool                isInitialized;
	PlatformGPIO_t      sleepPin;
	PlatformRingBuffer *uartBuffer;
} FQuadRFInfo_t;

static FQuadRFInfo_t mRFInfoStruct;

static FStatus _FQuadRF_GetChecksum( void *const inFrameData, size_t inFrameLen, uint8_t *const outChecksum );
static FStatus _FQuadRF_SerializePacket( FQuadCommsPacket_t *const outPacket, void *const inFrameData, size_t inFrameLen );
static FStatus _FQuadRF_SendPacket( FQuadCommsPacket_t *const inPacket );

FStatus FQuadRF_Init( const PlatformGPIO_t inSleepPin )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	
	// Initialize ring buffer for UART
	mRFInfoStruct.uartBuffer = PlatformRingBuffer_Create( FQUADRF_UART_RING_BUFFER_SIZE );
	require( mRFInfoStruct.uartBuffer, exit );
		
	// Initialize UART with ring buffer
	platformStatus = PlatformUART_Init( FQUADRF_UART_BAUD_RATE, mRFInfoStruct.uartBuffer );
	require_noerr( platformStatus, exit );
		
	// Configure sleep pin, deasserted
	platformStatus = PlatformGPIO_Configure( inSleepPin, PlatformGPIOConfig_Output );
	require_noerr( platformStatus, exit );
		
	platformStatus = PlatformGPIO_OutputLow( inSleepPin );
	require_noerr( platformStatus, exit );
	
	// Save sleep pin info
	mRFInfoStruct.sleepPin = inSleepPin;
	
	mRFInfoStruct.isInitialized = true;
	
	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadRF_SendMessage( const uint8_t *const inData,
				  			 const size_t         inDataLen,
							 const uint8_t        inMsgID,
							 const uint32_t       inDestAddrH,
							 const uint32_t       inDestAddrL )
{
	FStatus                 status = FStatus_Failed;
	FQuadRFTXFrameData_t txFrameData;
	size_t                  txFrameLength;
	FQuadCommsPacket_t      txPacket;
	
	require_action( inData, exit, status = FStatus_InvalidArgument );
	require_action( inDataLen != 0, exit, status = FStatus_InvalidArgument );
	require_action( inDataLen < FQUADRF_MAX_MSG_DATA_LEN, exit, status = FStatus_InvalidArgument );
	
	// Fill TX command specific data
	txFrameData.cmd       = FQuadRFCmd_TX64Bit;
	txFrameData.frameID   = inMsgID;
	txFrameData.destADDRH = inDestAddrH;
	txFrameData.destADDRL = inDestAddrL;
	txFrameData.options   = FQuadRFTXOptions_None;
	
	memcpy( txFrameData.data, inData, inDataLen );

	// The length of the frame is the size of the struct , but using inDataLen instead of the array max size
	txFrameLength = sizeof( FQuadRFTXFrameData_t ) - FQUADRF_MAX_MSG_DATA_LEN + inDataLen;
	
	// Create packet
	status = _FQuadRF_SerializePacket( &txPacket, &txFrameData, txFrameLength );
	require_noerr( status, exit );
	
	// Send the packet
	status = _FQuadRF_SendPacket( &txPacket );
	require_noerr( status, exit );
	
exit:
	return status;
}

static FStatus _FQuadRF_SerializePacket( FQuadCommsPacket_t *const outPacket, void *const inFrameData, size_t inFrameLen )
{
	FStatus status = FStatus_Failed;
	
	require_action( outPacket, exit, status = FStatus_InvalidArgument );
	require_action( inFrameData, exit , status = FStatus_InvalidArgument );
	require_action( inFrameLen, exit, status = FStatus_InvalidArgument );
	
	// Fill packet info
	outPacket->startByte   = FQUADRF_START_BYTE;
	outPacket->frameLength = HTONS( inFrameLen );
	
	memcpy( outPacket->frameData, inFrameData, inFrameLen );
	
	// Get packet checksum
	status = _FQuadRF_GetChecksum( inFrameData, inFrameLen, &outPacket->checksum );
	
exit:
	return status;
}

static FStatus _FQuadRF_GetChecksum( void *const inFrameData, size_t inFrameLen, uint8_t *const outChecksum )
{
	FStatus status = FStatus_Failed;
	uint8_t checksum = 0;
	
	require_action( inFrameData, exit , status = FStatus_InvalidArgument );
	require_action( inFrameLen, exit, status = FStatus_InvalidArgument );
	require_action( outChecksum, exit, status = FStatus_InvalidArgument );
	
	// Sum all the bytes
	for ( uint8_t i = 0; i < inFrameLen; i++ )
	{
		checksum += (( uint8_t* )inFrameData )[i];
	}
	
	// Subtract from 0xFF
	*outChecksum = 0xFF - checksum;
	
	status = FStatus_Success;
exit:
	return status;
}

static FStatus _FQuadRF_SendPacket( FQuadCommsPacket_t *const inPacket )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	
	require_action( inPacket, exit, status = FStatus_InvalidArgument );
	
	// Send start byte
	platformStatus = PlatformUART_Transmit(( uint8_t* )&inPacket->startByte, sizeof( inPacket->startByte ));
	require_noerr( platformStatus, exit );
	
	// Send frame length byte
	platformStatus = PlatformUART_Transmit(( uint8_t* )&inPacket->frameLength, sizeof( inPacket->frameLength ));
	require_noerr( platformStatus, exit );
	
	// Send the frame data
	platformStatus = PlatformUART_Transmit(( uint8_t* )&inPacket->frameData, inPacket->frameLength );
	require_noerr( platformStatus, exit );
	
	// Send checksum
	platformStatus = PlatformUART_Transmit(( uint8_t* )&inPacket->checksum, sizeof( inPacket->checksum ));
	require_noerr( platformStatus, exit );
	
	status = FStatus_Success;
exit:
	return status;
}


FStatus FQuadRF_ReceiveMessage( uint8_t *const outData, uint8_t *const outDataLen, FQuadRFCmdID_t *const outCmdID )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	uint8_t rawData[FQUADRF_MAX_PACKET_SIZE];
	uint8_t calculatedChecksum;
	
	uint16_t packetLength;
	uint16_t frameLengthRaw;
	
	FQuadCommsPacket_t *rawPacket;
	
	FQuadRFRXFrameData_t *rawRXData;
	FQuadRFTXACKData_t   *rawTXStatusData;
	
	FQuadRFTXStatus_t *txStatus;
	FQuadRFRXData_t   *rxData;
	
	require_action( outData, exit, status = FStatus_InvalidArgument );
	require_action( outDataLen, exit, status = FStatus_InvalidArgument );
	require_action( outCmdID, exit, status = FStatus_InvalidArgument );
	
	// First get enough data to read the length of packet
	platformStatus = PlatformUART_Receive( rawData, FQUADRF_HEADER_BYTES );
	require( platformStatus, exit );
	
	// If we received something, the first byte should be a start byte, since nothing else is using the UART
	require( rawData[0] == FQUADRF_START_BYTE, exit );
	
	// Find the remaining packet length
	frameLengthRaw = *( uint16_t* )&rawData[1];
	packetLength   = NTOHS( frameLengthRaw ) + FQUADRF_OVERHEAD_BYTES;
	
	// Get the rest of the packet, which is the frame + checksum
	platformStatus = PlatformUART_Receive( &rawData[FQUADRF_HEADER_BYTES], packetLength - FQUADRF_HEADER_BYTES );
	
	// If the receive failed, then the packet may still be transmitting over UART.
	// Tr waiting a sufficient amount of time for the rest of the packet to be received
	if ( platformStatus != PlatformStatus_Success )
	{
		// TODO change this to a timer function if keeping a delay?
		_delay_ms( FQUADRF_MAX_PACKET_SEND_TIME_MS * 2 );
		
		// Try receiving again
		platformStatus = PlatformUART_Receive( &rawData[FQUADRF_HEADER_BYTES], packetLength - FQUADRF_HEADER_BYTES );
		require_noerr( platformStatus, exit );
	}
	
	// Check the checksum of the frame (don't include checksum byte in the checksum)
	_FQuadRF_GetChecksum( &rawData[FQUADRF_HEADER_BYTES], packetLength - FQUADRF_OVERHEAD_BYTES, &calculatedChecksum );
	
	// Check the checksum matches what we received
	require( calculatedChecksum == rawData[ FQUADRF_HEADER_BYTES + packetLength ], exit );
	
	// Return the relevant data to the upper layer
	*outCmdID = rawData[FQUADRF_HEADER_BYTES];
	
	if ( *outCmdID == FQuadRFCmd_TXStatus )
	{
		// Cast the raw data into packet structure for easier access
		rawPacket       = ( FQuadCommsPacket_t* ) rawData;
		rawTXStatusData = ( FQuadRFTXACKData_t* ) rawPacket->frameData;
		
		// Cast output data to the appropriate type
		txStatus = ( FQuadRFTXStatus_t* ) outData;
		
		// Copy raw data into output
		txStatus->frameID = rawTXStatusData->frameID;
		txStatus->status  = rawTXStatusData->status;
	}
	else if ( *outCmdID == FQuadRFCmd_RX64Bit )
	{
		// Cast the raw data into packet structure for easier access
		rawPacket = ( FQuadCommsPacket_t* ) rawData;
		rawRXData = ( FQuadRFRXFrameData_t* ) rawPacket->frameData;
				
		// Cast output data to the appropriate type
		rxData = ( FQuadRFRXData_t* ) outData;
				
		// Copy RSSI into output
		rxData->RSSI = rawRXData->RSSI;
		
		// Copy message data into output
		memcpy( rxData->data, rawRXData->data, NTOHS( rawPacket->frameLength ) - FQUADRF_RX_MSG_NON_DATA_BYTES );
	}
	else
	{
		status = FStatus_Failed;
		goto exit;
	}
	
	status = FStatus_Success;
exit:
	return status;
}

FStatus FQuadRF_Wake()
{
	PlatformStatus platformStatus;
	
	platformStatus = PlatformGPIO_OutputLow( mRFInfoStruct.sleepPin );
	
	// Wait enough time for the module to exit sleep mode
	// TODO change this to timer?
	_delay_ms( FQUAD_COMMS_WAKE_TIME_MS );
	
	return ( platformStatus == PlatformStatus_Success ) ? FStatus_Success : FStatus_Failed;
}

FStatus FQuadRF_Sleep()
{
	PlatformStatus platformStatus;
	
	platformStatus = PlatformGPIO_OutputHigh( mRFInfoStruct.sleepPin );
	
	return ( platformStatus == PlatformStatus_Success ) ? FStatus_Success : FStatus_Failed;
}