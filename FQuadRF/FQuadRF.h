/*
 * FQuadRF.h
 *
 * Created: 2017-01-25 10:02:55 PM
 *  Author: Felix
 */ 


#ifndef FQUADRF_H_
#define FQUADRF_H_

#include "FStatus.h"
#include "PlatformGPIO.h"
#include <stdint.h>
#include <stddef.h>

#define FQUADRF_MAX_MSG_DATA_LEN    ( 16 )

F_ENUM( uint8_t, FQuadRFTXStatus )
{
	FQuadRFTXStatus_Success = 0,
	FQuadRFTXStatus_NACK    = 1,
	FQuadRFTXStatus_CCAFail = 2,
	FQuadRFTXStatus_Purged  = 3,
};

F_ENUM( uint8_t, FQuadRFCmdID_t )
{
	FQuadRFCmd_TX64Bit  = 0x00,
	FQuadRFCmd_TXStatus = 0x89,
	FQuadRFCmd_RX64Bit  = 0x80,
};

typedef struct
{
	uint8_t            frameID;
	FQuadRFTXStatus status;
} FQuadRFTXStatus_t;

typedef struct
{
	int8_t  RSSI;
	uint8_t data[FQUADRF_MAX_MSG_DATA_LEN];
} FQuadRFRXData_t;

FStatus FQuadRF_Init( const PlatformGPIO_t inSleepPin );

FStatus FQuadRF_SendMessage( const uint8_t *const inData, 
                             const size_t         inDataLen, 
							 const uint8_t        inMsgID, 
							 const uint32_t       inDestAddrH, 
							 const uint32_t       inDestAddrL );

FStatus FQuadRF_ReceiveMessage( uint8_t *const outData, uint8_t *const outDataLen, FQuadRFCmdID_t *const outCmdID );

FStatus FQuadRF_Wake();

FStatus FQuadRF_Sleep();


#endif /* FQUADRF_H_ */