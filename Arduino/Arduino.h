/*
  Arduino.h - Main include file for the Arduino SDK
  Copyright (c) 2014 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Arduino_h
#define Arduino_h

#include <math.h>

#include "core_types.h"
#include "CommonMacro.h"

// some libraries and sketches depend on this AVR stuff,
// assuming Arduino.h or WProgram.h automatically includes it...
//
#include "pgmspace.h"
#include "interrupt.h"
#include "io.h"
#include "dtostrf.h"

#include "binary.h"
#include "itoa.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

#include "wiring_constants.h"
#include "yield.h"

  /* sketch */
//void setup( void ) ;
//void loop( void ) ;

#include "WVariant.h"

#ifdef __cplusplus
} // extern "C"
#endif

// The following headers are for C++ only compilation
#ifdef __cplusplus
#include "WCharacter.h"
#include "WString.h"
//#include "Tone.h"
#include "WMath.h"
#include "HardwareSerial.h"
#include "libaries/IWatchdog/IWatchdog.h"
#include "libaries/TrueRandom/TrueRandom.h"
#include "libaries/Timer0/src/Timer0.h"
// #include "RingBuf.h"
#include "Stream.h"
//#include "pulse.h"
#endif

#ifdef __cplusplus
#include "wiring_digital.h"
#include "wiring_analog.h"
#include "delay.h"
#endif


//#include "wiring_shift.h"
//#include "WInterrupts.h"

#include "heap_init.h"
// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif // abs

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define bit(b) (1UL << (b))

// dynamic F_CPU
#include "drivers/sysclock/sysclock.h"
#ifndef F_CPU
  #define F_CPU (SYSTEM_CLOCK_FREQUENCIES.hclk)
#endif

// direct port manipulation for GPIO
#include "drivers/gpio/direct_access.h"
#endif // Arduino_h
