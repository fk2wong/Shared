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
#define FQUADRF_UART_RING_BUFFER_SIZE ( 64 )

#define UART_BITS_PER_BYTE                ( 10 )
#define FQUADRF_MAX_PACKET_SEND_TIME_MS   (( uint16_t )((( uint32_t )UART_BITS_PER_BYTE * FQUADRF_MAX_PACKET_SIZE * 1000 ) / ( FQUADRF_UART_BAUD_RATE )))

#define FQUADRF_START_BYTE  ( 0x7E )
#define FQUADRF_HEADER_BYTES ( 3 )
#define FQUADRF_CHECKSUM_BYTES ( 1 )
#define FQUADRF_PACKET_OVERHEAD_BYTES ( FQUADRF_HEADER_BYTES + FQUADRF_CHECKSUM_BYTES )
#define FQUADRF_DATA_FRAME_OVERHEAD_BYTES ( 11 )

// Worst case, every character needs an escape character
#define FQUADRF_MAX_MSG_DATA_LEN ( FQUADRF_MAX_MSG_RAW_DATA_LEN * 2 ) 

#define FQUADRF_MAX_FRAME_SIZE ( FQUADRF_MAX_MSG_DATA_LEN + FQUADRF_DATA_FRAME_OVERHEAD_BYTES )
#define FQUADRF_MAX_PACKET_SIZE ( FQUADRF_MAX_FRAME_SIZE + FQUADRF_PACKET_OVERHEAD_BYTES )

#define FQUADRF_ESCAPE_CHAR ( 0x7D )
#define FQUADRF_XOR_CHAR    ( 0x20 )
#define FQUADRF_CHAR_MUST_BE_ESCAPED( X ) ((( X ) == mEscapeChars[0] ) || (( X ) == mEscapeChars[1] ) || (( X ) == mEscapeChars[2] ) || (( X ) == mEscapeChars[3] ))

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
	
	uint8_t numEscapedCharsInTXPacket;
	
	// For managing ring buffer data and assembling packets
	uint16_t bytesLeftInRXPacket;
	
	// Callback info
	FQuadRF_ACKReceivedISR ackReceivedISR;
	FQuadRF_MsgReceivedISR msgReceivedISR;
	
} FQuadRFInfo_t;

//================================//
//         Local Variables        //
//================================//

static FQuadRFInfo_t mRFInfoStruct;

static const char mEscapeChars[] = { 0x7E, 0x7D, 0x11, 0x13 };
	
//================================//
// Internal Function Declarations //
//================================//

static FStatus _FQuadRF_GetRawChecksum( void *const inFrameData, size_t inFrameLen, uint8_t *const outChecksum );
static FStatus _FQuadRF_SerializePacket( FQuadRFPacket_t *const outPacket, void *const inFrameData, uint8_t inFrameLen );
static FStatus _FQuadRF_SendPacket( FQuadRFPacket_t *const inPacket );

void _FQuadRF_ByteReceivedISR( PlatformRingBuffer *const inRingBuffer,
                                                             const uint8_t* const      inDataReceived,
                                                             const size_t              inDataLen,
                                                             const size_t              inBufferBytesUsed );

static FStatus _FQuadRF_UnescapeChars( uint8_t * ioData, const size_t inEscapedLen );
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
	mRFInfoStruct.bytesLeftInRXPacket = 0;
	
	mRFInfoStruct.numEscapedCharsInTXPacket = 0;
	
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
	uint8_t              txFrameLength; //NOTE: Originally used size_t, but compiling with -O1 caused a bug where the arithmetic of the high and low bytes are not related.
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
	txFrameLength = ( uint8_t )( sizeof( FQuadRFTXFrameData_t ) - FQUADRF_MAX_MSG_DATA_LEN + inDataLen );
	
	// Create packet
	status = _FQuadRF_SerializePacket( &txPacket, &txFrameData, txFrameLength );
	require_noerr( status, exit );
	
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

static FStatus _FQuadRF_SerializePacket( FQuadRFPacket_t *const outPacket, void *const inFrameData, uint8_t inFrameLen )
{
	FStatus status = FStatus_Failed;
	uint8_t packetDataIndex = 0;
	
 	require_action( outPacket, exit, status = FStatus_InvalidArgument );
 	require_action( inFrameData, exit , status = FStatus_InvalidArgument );
 	require_action( inFrameLen, exit, status = FStatus_InvalidArgument );
		
	// Set start byte
	outPacket->startByte   = FQUADRF_START_BYTE;
	
	// Set the frame length
	outPacket->frameLength = HTONS( inFrameLen );
	
	// Reset number of escaped characters
	mRFInfoStruct.numEscapedCharsInTXPacket = 0;
	
	// Copy frame data into the packet, escaping characters when necessary
	for ( uint8_t i = 0; i < inFrameLen; i++ )
	{
		if ( FQUADRF_CHAR_MUST_BE_ESCAPED( (( uint8_t* )inFrameData )[i] ))
		{
			outPacket->frameData[packetDataIndex++] = FQUADRF_ESCAPE_CHAR;
			outPacket->frameData[packetDataIndex++] = (( uint8_t* )inFrameData )[i] ^ FQUADRF_XOR_CHAR;
			mRFInfoStruct.numEscapedCharsInTXPacket++;
		}
		else
		{
			// Copy the raw character
			outPacket->frameData[packetDataIndex++] = (( uint8_t* )inFrameData )[i];
		}
	}
	
	// Get packet checksum, using data before being escaped
	status = _FQuadRF_GetRawChecksum( inFrameData, inFrameLen, &outPacket->checksum );
	
	// If this checksum must be escaped, then add another escape character to the packet data, and use the escaped checksum
	if ( FQUADRF_CHAR_MUST_BE_ESCAPED( outPacket->checksum ))
	{
		mRFInfoStruct.numEscapedCharsInTXPacket++;
		outPacket->frameData[packetDataIndex++] = FQUADRF_ESCAPE_CHAR;
		outPacket->checksum ^= FQUADRF_XOR_CHAR;
	}
	
exit:
	return status;
}

static FStatus _FQuadRF_GetRawChecksum( void *const inFrameData, size_t inFrameLen, uint8_t *const outChecksum )
{
	FStatus status = FStatus_Failed;
	uint8_t checksum = 0;
	uint8_t i;
	
	require_action( inFrameData, exit , status = FStatus_InvalidArgument );
	require_action( inFrameLen, exit, status = FStatus_InvalidArgument );
	require_action( outChecksum, exit, status = FStatus_InvalidArgument );
	
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
	platformStatus = PlatformUART_Transmit(( uint8_t* )&inPacket->frameData, NTOHS( inPacket->frameLength ) + mRFInfoStruct.numEscapedCharsInTXPacket );
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
	uint16_t rawLength;
	
	uint8_t data[FQUADRF_MAX_PACKET_SIZE];
	uint8_t calcedChecksum;
	
	require_quiet( inRingBuffer, exit );
	require_quiet( inDataReceived, exit );
	require_quiet( inDataLen == 1, exit );
	
	// If this is the start of a new packet ( there are no more bytes in the previous packet, and we received a new byte )
	if ( mRFInfoStruct.bytesLeftInRXPacket == 0 )
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
			
			// Get the number of remaining bytes for this RX packet, including checksum
			mRFInfoStruct.bytesLeftInRXPacket = rawLength + FQUADRF_CHECKSUM_BYTES;
		}
	}
	// Otherwise we are waiting for bytes in the packet, after a header has already been received. 
	else
	{
		// Decrement the number of bytes we are waiting for, since we just received one, as long as it's not an escape character
		// If this is an escape character, then don't count this as part of the frame length or bytesLeftInRXPacket, and wait for next byte 
		if ( !FQUADRF_CHAR_MUST_BE_ESCAPED( *inDataReceived ))
		{
			mRFInfoStruct.bytesLeftInRXPacket--;
		}
				
		// If the packet is now complete with this byte, package the relevant data and send it to upper layers.
		if ( mRFInfoStruct.bytesLeftInRXPacket == 0 )
		{
			// Get the entire packet
			status = PlatformRingBuffer_ReadBuffer( inRingBuffer, data, inBufferBytesUsed );
			require_noerr_quiet( status, exit );
					
			// Remove escaped chars
			status = _FQuadRF_UnescapeChars( data, inBufferBytesUsed );
			require_noerr_quiet( status, exit );
			
			// Verify checksum
			status = _FQuadRF_GetRawChecksum( (( FQuadRFPacket_t* )data)->frameData, (( FQuadRFPacket_t* )data)->frameLength, &calcedChecksum );
			require_noerr_quiet( status, exit );
			
			// Verify that checksum == current byte
			require_quiet( calcedChecksum == (( FQuadRFPacket_t* )data)->checksum, exit );
			
			status = _FQuadRF_ExtractDataAndSendCallback( data, inBufferBytesUsed );
			require_noerr_quiet( status, exit );
		}
	}
	
exit:
	return;
}

static FStatus _FQuadRF_UnescapeChars( uint8_t * ioData, const size_t inEscapedLen )
{
	FStatus status = FStatus_Failed;
	
	FQuadRFPacket_t *rawPacket;
	
	FQuadRFPacket_t unescapedPacket;
	uint8_t         unescapedIndex = 0;
	uint8_t         numCumulatedEscapedChars = 0;
	
	uint8_t *checksumByte;
	
	require_action_quiet( ioData, exit, status = FStatus_InvalidArgument );
	require_action_quiet( inEscapedLen, exit, status = FStatus_InvalidArgument );
	
	// Cast raw data into packet
	rawPacket = ( FQuadRFPacket_t* )ioData;
	
	// Copy all except frame data
	unescapedPacket.startByte = rawPacket->startByte;
	unescapedPacket.frameLength = NTOHS( rawPacket->frameLength );
	
	// Loop through frame data and unescape chars
	for ( unescapedIndex = 0; unescapedIndex < unescapedPacket.frameLength; unescapedIndex++ )
	{		
		if ( FQUADRF_CHAR_MUST_BE_ESCAPED( rawPacket->frameData[unescapedIndex + numCumulatedEscapedChars] ))
		{
			// If this character is an escape character, skip it and XOR the next byte with 0x20
			numCumulatedEscapedChars++;
			unescapedPacket.frameData[unescapedIndex] = rawPacket->frameData[unescapedIndex + numCumulatedEscapedChars] ^ FQUADRF_XOR_CHAR ;
		}
		else
		{
			// Copy the raw character
			unescapedPacket.frameData[unescapedIndex] = rawPacket->frameData[unescapedIndex + numCumulatedEscapedChars];
		}
	}
	
	// Checksum must be found since the packet structure over the wire is not the same as ours ( no padding for max size packets )
	// Note: it is not this function's duty to verify the checksum, only extract it from the raw data.
	checksumByte = &((( uint8_t* )rawPacket )[FQUADRF_HEADER_BYTES + unescapedPacket.frameLength + numCumulatedEscapedChars]);
	
	// If the checksum must be escaped, find the next byte and XOR it
	if ( FQUADRF_CHAR_MUST_BE_ESCAPED( *checksumByte ))
	{
		checksumByte++;
		*checksumByte ^= FQUADRF_XOR_CHAR;
	}
	unescapedPacket.checksum = *checksumByte;
	
	// Now overwrite the old packet with the new unescaped one
	memcpy( rawPacket, &unescapedPacket, sizeof( FQuadRFPacket_t ));
	
	status = FStatus_Success;
exit:
	return status;
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