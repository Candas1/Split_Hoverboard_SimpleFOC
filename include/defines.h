// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#ifdef LAYOUT_2_0
	#include "defines_2-0.h"		// https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2
	// hall input pins A,B,C = B11,F1,C14 , etc.
#endif
#ifdef LAYOUT_2_1
	#include "defines_2-1.h"		// https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1
#endif
#ifdef LAYOUT_2_2
	#include "defines_2-2.h"		// https://github.com/krisstakos/Hoverboard-Firmware-Hack-Gen2.1
#endif
#ifdef LAYOUT_2_4
	#include "defines_2-4.h"		// https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/3
#endif
#ifdef LAYOUT_2_5
	#include "defines_2-5.h"		// https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x/issues/11
#endif
#ifdef LAYOUT_2_NONE	// for testing the binary upload with no harmful pin definitions
	#include "defines_2-x.h"
#endif
#ifdef LAYOUT_Gen1	// for testing the binary upload with no harmful pin definitions
	#include "defines_1-0.h"
#endif


#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

#ifdef DEBUG_UART
	#define SERIALDEBUG DEBUG_UART
#else
	#ifdef DEBUG_STLINK
		#define SERIALDEBUG DEBUG_STLINK
	#endif	
#endif	
#ifdef SERIALDEBUG
	#define DEBUG(code)	{code}
	#define OUT(s)	{SERIALDEBUG.print(s);}
	#define OUT2(s,i)	{SERIALDEBUG.print(s);SERIALDEBUG.print(": ");SERIALDEBUG.print(i);}
	#define OUT2T(s,i)	{SERIALDEBUG.print(s);SERIALDEBUG.print(": ");SERIALDEBUG.print(i);SERIALDEBUG.print("\t");}
	#define OUT2N(s,i)	{SERIALDEBUG.print(s);SERIALDEBUG.print(": ");SERIALDEBUG.print(i);SERIALDEBUG.println();}
	#define OUTN(s)	{SERIALDEBUG.println(s);}
#else
	#define DEBUG(code)
	#define OUT(s)
	#define OUT2(s)
	#define OUT2T(s)
	#define OUT2N(s)
	#define OUTN(s)
#endif

#endif //  DEFINES_H
