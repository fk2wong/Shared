/*
 * require_macros.h
 *
 * Created: 2016-09-07 4:46:14 PM
 *  Author: Felix
 */ 


#ifndef REQUIRE_MACROS_H_
#define REQUIRE_MACROS_H_

#define USE_PRODUCTION_CODE 1

#if USE_PRODUCTION_CODE
#define require( assertation, label )                 \
	do                                                \
	{                                                 \
		if ( __builtin_expect( !( assertation ), 0 )) \
		{                                             \
			goto label;                               \
		}                                             \
	}                                                 \
	while ( 0 )
#else
#define require( assertation, label ) //TODO                   
#endif // USE_PRODUCTION_CODE

#if USE_PRODUCTION_CODE
#define require_quiet( assertation, label )       \
do                                                \
{                                                 \
	if ( __builtin_expect( !( assertation ), 0 )) \
	{                                             \
		goto label;                               \
	}                                             \
}                                                 \
while ( 0 )
#else
#define require_quiet( err, label ) //TODO
#endif // USE_PRODUCTION_CODE

#if USE_PRODUCTION_CODE
#define require_noerr( err, label )              \
	do                                           \
	{                                            \
		if ( __builtin_expect(( err ) != 0, 0 )) \
		{                                        \
			goto label;                          \
		}                                        \
	}                                            \
	while ( 0 )
#else
#define require_noerr( err, label ) //TODO
#endif // USE_PRODUCTION_CODE

#if USE_PRODUCTION_CODE
#define require_noerr_quiet( err, label )        \
	do                                           \
	{                                            \
		if ( __builtin_expect(( err ) != 0, 0 )) \
		{                                        \
			goto label;                          \
		}                                        \
	}                                            \
	while ( 0 )
#else
#define require_noerr_quiet( err, label ) //TODO
#endif // USE_PRODUCTION_CODE

#if USE_PRODUCTION_CODE
#define require_action( assertation, label, action ) \
do                                                   \
{                                                    \
	if ( __builtin_expect( !( assertation ), 0 ))    \
	{                                                \
		action;                                      \
		goto label;                                  \
	}                                                \
}                                                    \
while ( 0 )
#else
#define require_action( err, label, action ) //TODO
#endif // USE_PRODUCTION_CODE

#if USE_PRODUCTION_CODE
#define require_action_quiet( assertation, label, action ) \
do                                                         \
{                                                          \
	if ( __builtin_expect( !( assertation ), 0 ))          \
	{                                                      \
		action;                                            \
		goto label;                                        \
	}                                                      \
}                                                          \
while ( 0 )
#else
#define require_action_quiet( err, label, action ) //TODO
#endif // USE_PRODUCTION_CODE

#endif /* REQUIRE_MACROS_H_ */