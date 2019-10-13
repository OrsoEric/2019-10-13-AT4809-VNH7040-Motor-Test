

#ifndef GLOBAL_H
	//header environment variable, is used to detect multiple inclusion
	//of the same header, and can be used in the c file to detect the
	//included library
	#define GLOBAL_H

	/****************************************************************************
	**	ENVROIMENT VARIABILE
	****************************************************************************/

	#define F_CPU 20000000

	/****************************************************************************
	**	GLOBAL INCLUDE
	**	TIPS: you can put here the library common to all source file
	****************************************************************************/

	//type definition using the bit width and signedness
	#include <stdint.h>
	//define the ISR routune, ISR vector, and the sei() cli() function
	#include <avr/interrupt.h>
	//name all the register and bit
	#include <avr/io.h>
	//hard delay
	#include <util/delay.h>
	//General purpose macros
	#include "at_utils.h"
	//AT4809 PORT macros definitions
	#include "at4809_port.h"

	/****************************************************************************
	**	DEFINE
	****************************************************************************/

	/****************************************************************************
	**	MACRO
	****************************************************************************/

		///----------------------------------------------------------------------
		///	LEDS
		///----------------------------------------------------------------------

	#define LED0_TOGGLE()	\
		TOGGLE_BIT( PORTB, PB6 )

	/****************************************************************************
	**	TYPEDEF
	****************************************************************************/

	//Global flags raised by ISR functions
	typedef struct _Isr_flags Isr_flags;

	/****************************************************************************
	**	STRUCTURE
	****************************************************************************/

	//Global flags raised by ISR functions
	struct _Isr_flags
	{
		//First byte
		U8 system_tick	: 1;	//System Tick
		U8 				: 7;	//unused bits
	};

	/****************************************************************************
	**	PROTOTYPE: INITIALISATION
	****************************************************************************/

	//port configuration and call the peripherals initialization
	extern void init( void );

	/****************************************************************************
	**	PROTOTYPE: FUNCTION
	****************************************************************************/


	/****************************************************************************
	**	PROTOTYPE: GLOBAL VARIABILE
	****************************************************************************/

		///----------------------------------------------------------------------
		///	STATUS FLAGS
		///----------------------------------------------------------------------

	//Volatile flags used by ISRs
	extern volatile	Isr_flags g_isr_flags;
		

#else
	#warning "multiple inclusion of the header file global.h"
#endif


