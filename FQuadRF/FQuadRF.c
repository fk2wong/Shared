/*
 * FQuadRF.c
 *
 * Created: 2017-01-25 10:03:31 PM
 *  Author: Felix
 */ 

#include "FQuadRF.h"
#include "FUtilities.h"
#include "PlatformRingBuffer.h"
#include "PlatformUART.h"
#include "PlatformClock.h"
#include "require_macros.h"
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>

#define FQUADRF_UART_BAUD_RATE        ( 19200 )
#define FQUADRF_UART_RING_BUFFER_SIZE ( 32 )

#define UART_BITS_PER_BYTE                ( 10 )
#define FQUADRF_MAX_PACKET_SEND_TIME_MS     (( uint16_t )((( uint32_t )UART_BITS_PER_BYTE * FQUADRF_MAX_PACKET_SIZE * 1000 ) / ( FQUADRF_UART_BAUD_RATE )))

#define FQUADRF_START_BYTE ( 0x7E )
#define FQUADRF_HEADER_BYTES ( 3 )
#define FQUADRF_CHECKSUM_BYTES ( 1 )
#define FQUADRF_PACKET_OVERHEAD_BYTES ( FQUADRF_HEADER_BYTES + FQUADRF_CHECKSUM_BYTES )
#define FQUADRF_DATA_FRAME_OVERHEAD_BYTES ( 11 )
#define FQUADRF_MAX_FRAME_SIZE ( FQUADRF_MAX_MSG_DATA_LEN + FQUADRF_DATA_FRAME_OVERHEAD_BYTES )
#define FQUADRF_MAX_PACKET_SIZE ( FQUADRF_MAX_FRAME_SIZE + FQUADRF_PACKET_OVERHEAD_BYTES )

#define FQUADRF_RX_MSG_NON_DATA_BYTES ( 10 )

#define FQUAD_COMMS_WAKE_TIME_MS ( 4 )

//===============================//
//           Typedefs            //
//===============================//

F_ENUM( uint8_t, FQuadRFCmdID_t )
{
	FQuadRFCmd_TX64Bit  = 0x00,
	FQuadRFCmd_TXStatus = 0x89,
	FQuadRFCmd_RX64Bit  = 0x80,
};

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
	FQuadRFRXOptions    options;
	uint8_t             data[FQUADRF_MAX_MSG_DATA_LEN];
} FQuadRFRXFrameData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadRFCmdID_t      cmd;
	uint8_t             frameID;
	uint32_t            destADDRH;
	uint32_t            destADDRL;
	FQuadRFTXOptions    options;
	uint8_t             data[FQUADRF_MAX_MSG_DATA_LEN];
} FQuadRFTXFrameData_t;

typedef struct __attribute__ (( packed ))
{
	FQuadRFCmdID_t   cmd;
	uint8_t          frameID;
	FQuadRFTXStatus  status;
} FQuadRFTXACKData_t;

typedef struct __attribute__ (( packed ))
{
	uint8_t  startByte;
	uint16_t frameLength;
	uint8_t  frameData[FQUADRF_MAX_FRAME_SIZE];
	uint8_t  checksum;
} FQuadRFPacket_t;

typedef struct 
{
	bool                isInitialized;
	PlatformGPIO_t      sleepPin;
	PlatformRingBuffer *uartBuffer;
	
	// For managing ring buffer data and assembling packets
	uint8_t bytesLeftInPacket;
	
	// Callback info
	FQuadRF_ACKReceivedISR ackReceivedISR;
	FQuadRF_MsgReceivedISR msgReceivedISR;
	
} FQuadRFInfo_t;

static FQuadRFInfo_t mRFInfoStruct;

//================================//
// Internal Function Declarations //
//================================//

static FStatus _FQuadRF_GetChecksum( void *const inFrameData, size_t inFrameLen, uint8_t *const outChecksum );
static FStatus _FQuadRF_SerializePacket( FQuadRFPacket_t *const outPacket, void *const inFrameData, size_t inFrameLen );
static FStatus _FQuadRF_SendPacket( FQuadRFPacket_t *const inPacket );

void _FQuadRF_ByteReceivedISR( PlatformRingBuffer *const inRingBuffer,
                                                             const uint8_t* const      inDataReceived,
                                                             const size_t              inDataLen,
                                                             const size_t              inBufferBytesUsed );
															 
static FStatus _FQuadRF_ExtractDataAndSendCallback( uint8_t *const inData, const size_t inDataLen );

//=============================//
// Public Function Definitions //
//=============================//

FStatus FQuadRF_Init( const PlatformGPIO_t inSleepPin, FQuadRF_MsgReceivedISR inMsgReceivedISR, FQuadRF_ACKReceivedISR inACKReceivedISR )
{
	FStatus status = FStatus_Failed;
	PlatformStatus platformStatus;
	
	require_action( inMsgReceivedISR, exit, status = FStatus_InvalidArgument );
	require_action( inACKReceivedISR, exit, status = FStatus_InvalidArgument );
	
	// Initialize ring buffer for UART
	// We will get an ISR whenever a byte is received from the UART.
	// In this ISR we will assemble the bytes into packets and callback the upper layer with relevant data.
	mRFInfoStruct.uartBuffer = PlatformRingBuffer_Create( FQUADRF_UART_RING_BUFFER_SIZE, _FQuadRF_ByteReceivedISR );
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
	
	// Save callback info
	mRFInfoStruct.msgReceivedISR = inMsgReceivedISR;
	mRFInfoStruct.ackReceivedISR = inACKReceivedISR;
	
	// Indicate we are not waiting for any bytes in an ongoing packet RX transmission
	mRFInfoStruct.bytesLeftInPacket = 0;
	
	// Initialization complete
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
	FStatus              status = FStatus_Failed;
	FQuadRFTXFrameData_t txFrameData;
	size_t               txFrameLength;
	FQuadRFPacket_t      txPacket;
	
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
	
	PlatformUART_Transmit("here2\n", 6);
	
	// Create packet
	status = _FQuadRF_SerializePacket( &txPacket, &txFrameData, txFrameLength );
	require_noerr( status, exit );
	
	PlatformUART_Transmit((uint8_t*)&txPacket, sizeof(txPacket));
	
	// Send the packet
	status = _FQuadRF_SendPacket( &txPacket );
	require_noerr( status, exit );
	
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

//===============================//
// Internal Function Definitions //
//===============================//

static FStatus _FQuadRF_SerializePacket( FQuadRFPacket_t *const outPacket, void *const inFrameData, size_t inFrameLen )
{
	FStatus status = FStatus_Failed;
	
	require_action( outPacket, exit, status = FStatus_InvalidArgument );
	require_action( inFrameData, exit , status = FStatus_InvalidArgument );
	require_action( inFrameLen, exit, status = FStatus_InvalidArgument );
	
	PlatformUART_Transmit("here3\n", 6);
	
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
	uint8_t i;
	
	require_action( inFrameData, exit , status = FStatus_InvalidArgument );
	require_action( inFrameLen, exit, status = FStatus_InvalidArgument );
	require_action( outChecksum, exit, status = FStatus_InvalidArgument );
	
	PlatformUART_Transmit("here4\n", 6);
	
	// Sum all the bytes
	for ( i = 0; i < inFrameLen; i++ )
	{
		
		checksum += (( uint8_t* )inFrameData )[i];
	}
	
	// Subtract from 0xFF
	*outChecksum = 0xFF - checksum;
	
	status = FStatus_Success;
exit:
	return status;
}

static FStatus _FQuadRF_SendPacket( FQuadRFPacket_t *const inPacket )
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
	platformStatus = PlatformUART_Transmit(( uint8_t* )&inPacket->frameData, NTOHS( inPacket->frameLength ));
	require_noerr( platformStatus, exit );
	
	// Send checksum
	platformStatus = PlatformUART_Transmit(( uint8_t* )&inPacket->checksum, sizeof( inPacket->checksum ));
	require_noerr( platformStatus, exit );
	
	status = FStatus_Success;
exit:
	return status;
}

void _FQuadRF_ByteReceivedISR( PlatformRingBuffer *const inRingBuffer,
                                                            const uint8_t* const      inDataReceived,
                                                            const size_t              inDataLen,
                                                            const size_t              inBufferBytesUsed )
{
	// NOTE: This should only be called with a data length of one, 
	// since the only source writing to the ring buffer is the UART,
	// which is one byte at a time.
	PlatformStatus status;
	uint8_t data[FQUADRF_MAX_PACKET_SIZE];
	uint16_t rawLength;
	
	require_quiet( inRingBuffer, exit );
	require_quiet( inDataReceived, exit );
	require_quiet( inDataLen == 1, exit );
	
	// If this is the start of a new packet ( there are no more bytes in the previous packet, and we received a new byte )
	if ( mRFInfoStruct.bytesLeftInPacket == 0 )
	{
		// If this is the first byte in the packet, then ensure this is the start byte. 
		// If not, something went wrong, so flush this byte.
		if ( inBufferBytesUsed == 1 )
		{
			if ( *inDataReceived != FQUADRF_START_BYTE )
			{
				status = PlatformRingBuffer_Consume( inRingBuffer, 1 );
				require_noerr_quiet( status, exit );
			}
		}
		
		// If we have the complete header, we will know how much data is left in the packet.
		if ( inBufferBytesUsed == FQUADRF_HEADER_BYTES )
		{
			// Peek the header
			status = PlatformRingBuffer_Peek( inRingBuffer, data, FQUADRF_HEADER_BYTES );
			require_noerr_quiet( status, exit );
			
			// Get the length of the packet's frame
			rawLength = ( data[1] << 8 ) | data[2];
			
			mRFInfoStruct.bytesLeftInPacket = NTOHS( rawLength ) + FQUADRF_CHECKSUM_BYTES;
		}
	}
	// Otherwise we are waiting for bytes in the packet, after a header has already been received. 
	else
	{
		// Decrement the number of bytes we are waiting for
		mRFInfoStruct.bytesLeftInPacket--;
		
		// If the packet is now complete, package the relevant data and send it to upper layers.
		if ( mRFInfoStruct.bytesLeftInPacket == 0 )
		{
			status = PlatformRingBuffer_ReadBuffer( inRingBuffer, data, inBufferBytesUsed );
			require_noerr_quiet( status, exit );
			
			status = _FQuadRF_ExtractDataAndSendCallback( data, inBufferBytesUsed );
			require_noerr_quiet( status, exit );
		}
	}
	
exit:
	return;
}

static FStatus _FQuadRF_ExtractDataAndSendCallback( uint8_t *const inData, const size_t inDataLen )
{
	FStatus status = FStatus_Failed;
	
	FQuadRFPacket_t *rawPacket;
	FQuadRFRXFrameData_t *rxFrame;
	FQuadRFTXACKData_t   *ackFrame;
	
	require_action_quiet( inData, exit, status = FStatus_InvalidArgument );
	
	// Cast raw data into packet
	rawPacket = ( FQuadRFPacket_t* )inData;
	
	// Determine the type of packet
	switch ( rawPacket->frameData[0] )
	{
		case FQuadRFCmd_TXStatus:
		{
			// Cast frame to ACK frame
			ackFrame = ( FQuadRFTXACKData_t* ) rawPacket->frameData;
			
			// Call the registered ack received ISR
			mRFInfoStruct.ackReceivedISR( ackFrame->frameID, ackFrame->status );
			break;
		}
		case FQuadRFCmd_RX64Bit:
		{
			// Cast frame to RX Frame
			rxFrame = ( FQuadRFRXFrameData_t* ) rawPacket->frameData;
			
			// Call the registered msg received ISR
			mRFInfoStruct.msgReceivedISR( rxFrame->data, inDataLen - FQUADRF_DATA_FRAME_OVERHEAD_BYTES - FQUADRF_PACKET_OVERHEAD_BYTES, rxFrame->RSSI );
			break;
		}
	}
	
	status = FStatus_Success;
exit:
	return status;
}