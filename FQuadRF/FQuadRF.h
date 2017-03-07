/*
 * FQuadRF.h
 *
 * Created: 2017-01-25 10:02:55 PM
 *  Author: Felix
 */ 


#ifndef FQUADRF_H_
#define FQUADRF_H_

#include "FStatus.h"
#include "FQuadDefs.h"
#include "PlatformGPIO.h"
#include <stdint.h>
#include <stddef.h>

#define FQUADRF_MAX_MSG_RAW_DATA_LEN    ( 16 )

F_ENUM( uint8_t, FQuadRFTXStatus )
{
	FQuadRFTXStatus_Success = 0,
	FQuadRFTXStatus_NACK    = 1,
	FQuadRFTXStatus_CCAFail = 2,
	FQuadRFTXStatus_Purged  = 3,
};

typedef void ( *FQuadRF_MsgReceivedISR )( uint8_t *const inMsg, const size_t inMsgLen, const FQuadRSSI inRSSI );
typedef void ( *FQuadRF_ACKReceivedISR )( const uint8_t inFrameID, const FQuadRFTXStatus inACKStatus );

FStatus FQuadRF_Init( const PlatformGPIO_t inSleepPin, FQuadRF_MsgReceivedISR inMsgReceivedISR, FQuadRF_ACKReceivedISR inACKReceivedISR );

FStatus FQuadRF_SendMessage( const uint8_t *const inData, 
                             const size_t         inDataLen, 
							 const uint8_t        inMsgID, 
							 const uint32_t       inDestAddrH, 
							 const uint32_t       inDestAddrL );

FStatus FQuadRF_Wake();

FStatus FQuadRF_Sleep();


#endif /* FQUADRF_H_ */