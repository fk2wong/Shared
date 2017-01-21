/*
 * FUtilities.h
 *
 * Created: 2016-09-08 9:00:18 AM
 *  Author: Felix
 */ 


#ifndef FUTILITIES_H_
#define FUTILITIES_H_

#include <avr/io.h>
#include "stdint.h"

#define F_ENUM( TYPE, NAME ) typedef TYPE NAME; enum

#define MAX( X, Y )   ((( X ) > ( Y )) ? ( X ) : ( Y ))
#define MIN( X, Y )   ((( X ) < ( Y )) ? ( X ) : ( Y ))

#define HTONS( X ) (((( uint32_t )( X ) & 0xFF00 ) >> 8 ) | ((( uint32_t )( X ) & 0xFF ) << 8 ))  
#define HTONL( X ) (((( uint32_t )( X ) & 0xFF000000 ) >> 24 ) | \
				   ((( uint32_t )( X ) & 0xFF0000 ) >> 8 ) | \
				   ((( uint32_t )( X ) & 0xFF00 ) << 8 ) | \
				   ((( uint32_t )( X ) & 0xFF ) << 24 ))  

#define NTOHS( X ) HTONS( X )
#define NTOHL( X ) HTONL( X )



#endif /* FUTILITIES_H_ */