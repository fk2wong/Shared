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



#endif /* FUTILITIES_H_ */