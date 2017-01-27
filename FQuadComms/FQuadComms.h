/*
 * FQuadComms.h
 *
 * Created: 2017-01-20 12:54:52 AM
 *  Author: Felix
 */ 


#ifndef FQUADCOMMS_H_
#define FQUADCOMMS_H_

#include "FStatus.h"
#include "FQuadDefs.h"
#include "PlatformGPIO.h"

F_ENUM( uint8_t, FQuadCommsType_t )
{
	FQuadCommsType_Flight,
	FQuadCommsType_Controller,
};

/*!
 *\brief    Initializes the FQuadComms module.
 *
 *\param    inSleepPin  - Pin used to control the sleepRQ on the RF chip.
 */
FStatus FQuadComms_Init( const PlatformGPIO_t inSleepPin );

/*!
 *\brief    Sends controls from the controller to the quadcopter.
 */
FStatus FQuadComms_SendControls( const FQuadAxisValue inPitch, const FQuadAxisValue inRoll, const FQuadAxisValue inYaw, const FQuadThrustValue inThrust );

/*!
 *\brief    Receives controls from the controller.
 */
FStatus FQuadComms_ReceiveControls( FQuadAxisValue *const   outPitch,
									FQuadAxisValue *const   outRoll,
									FQuadAxisValue *const   outYaw,
									FQuadThrustValue *const outThrust );

/*!
 *\brief    Sends the battery level and RSSI from the quadcopter to the controller.
 */									
FStatus FQuadComms_SendFlightBatteryLevelAndRSSI( const FQuadBatteryLevel inBatteryLevel, const FQuadRSSI inFlightRSSI );
									
/*!
 *\brief    Gets the latest RSSI on the controller, as well as latest RSSI info sent from the quadcopter.
 */		
FStatus FQuadComms_GetLatestRSSI( FQuadRSSI *const outControllerRSSI, FQuadRSSI *const outFlightRSSI );

/*!
 *\brief    Gets the latest battery level sent from the quadcopter to the controller.
 */		
FStatus FQuadComms_GetLatestFlightBatteryLevel( FQuadBatteryLevel *const outBatteryLevel );

/*!
 *\brief    Puts the RF chip into sleep mode.
 */	
FStatus FQuadComms_Sleep( void );

/*!
 *\brief    Wakes the RF chip from sleep mode.
 */	
FStatus FQuadComms_Wake( void );

#endif /* FQUADCOMMS_H_ */