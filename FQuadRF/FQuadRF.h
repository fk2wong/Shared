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

/*!
 *\brief    ISR called back when a message is received.
 *
 *\param    inMsg - Data contained in the message.
 *\param    inMsgLen - Length of the message.
 *\param    inRSSI - RSSI of the received message.
 */
typedef void ( *FQuadRF_MsgReceivedISR )( uint8_t *const inMsg, const size_t inMsgLen, const FQuadRSSI inRSSI );

/*!
 *\brief    ISR called back when an ACK is received for a previously TX'd packet.
 *
 *\param    inFrameID - ID of the message that this ACK corresponds to.
 *\param    inACKStatus - Status of the ACK, see FQuadRFTXStatus for details.
 */
typedef void ( *FQuadRF_ACKReceivedISR )( const uint8_t inFrameID, const FQuadRFTXStatus inACKStatus );

/*!
 *\brief   Initializes the RF module, and anything related to receiving/sending RF data. 
 *
 *\param   inSleepPin - Pin controlling the sleep on the RF module. Sleep mode active high.
 *\param   inMsgReceivedISR - ISR for when a new message has been received. See FQuadRF_MsgReceivedISR for details.
 *\param   inACKReceivedISR - ISR for when an ACK has been received for previously sent message. See FQuadRF_ACKReceivedISR for details.
 */
FStatus FQuadRF_Init( const PlatformGPIO_t inSleepPin, FQuadRF_MsgReceivedISR inMsgReceivedISR, FQuadRF_ACKReceivedISR inACKReceivedISR );

/*!
 *\brief   Sends a message over RF.
 *
 *\param   inData - Data to be sent.
 *\param   inDataLen - Length of data.
 *\param   inMsgID - ID of the message being sent. After the data is sent, an ACK will be returned in an FQuadRF_ACKReceivedISR, with the ID of the sent message.
 *\param   inDestAddrH - High 4 bytes of the destination address.
 *\param   inDestAddrL - Low 4 bytes of the destination address.
 */
FStatus FQuadRF_SendMessage( const uint8_t *const inData, 
                             const size_t         inDataLen, 
							 const uint8_t        inMsgID, 
							 const uint32_t       inDestAddrH, 
							 const uint32_t       inDestAddrL );

/*!
 *\brief   Wakes the RF module from sleep mode.
 */
FStatus FQuadRF_Wake();

/*!
 *\brief   Puts the RF module into sleep mode.
 */
FStatus FQuadRF_Sleep();


#endif /* FQUADRF_H_ */