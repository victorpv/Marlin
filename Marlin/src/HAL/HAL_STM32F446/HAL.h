/* **************************************************************************
 
 Marlin 3D Printer Firmware
 Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

/**
 * Description: HAL for HAL_NUCLEO_F746ZG
 *
 * For ST ARM cortex M7
 */


#ifndef _HAL_STM32F446_H
#define _HAL_STM32F446_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>

#include <cmath>
#include <algorithm>

using std::min;
using std::max;
using std::abs;

//arduino defined?
#define F_CPU 180000000
#undef NOOP
#define NOOP __NOP()

#define PI          3.1415926535897932384626433832795
#define HALF_PI     1.5707963267948966192313216916398
#define TWO_PI      6.283185307179586476925286766559
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define min(a,b)                ((a)<(b)?(a):(b))
#define max(a,b)                ((a)>(b)?(a):(b))
//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)                ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg)            ((deg)*DEG_TO_RAD)
#define degrees(rad)            ((rad)*RAD_TO_DEG)
//#define sq(x)                   ((x)*(x))

#define USBCON

//arduino: Print.h
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
//arduino: binary.h (weird defines)
#define B01 1
#define B10 2

#include "main.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"

#include "fastio.h"
#include "watchdog.h"

//#include "Serial.h"
#include "USBSerial.h"
#define USBDP_PIN PA12 //USB Plus (+) pin number. That pin is normally pulled up to 3.3v by a 1.5k resistor
#include "HAL_timers.h"
#include "TMC26XStepper.h"

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------
extern USBSerial USerial;
#define MYSERIAL USerial
typedef bool boolean; //todo: fix HAL hack
#define PSTR(val) (val) //todo fix HAL hack
#define PROGMEM //todo :fix HAL hack
#define PGM_P const char *
#define _BV(bit) 	(1 << (bit))

#if ENABLED(DELTA_FAST_SQRT)
  #undef ATAN2
  #undef FABS
  #undef POW
  #undef SQRT
  #undef CEIL
  #undef FLOOR
  #undef LROUND
  #undef FMOD
  #define ATAN2(y, x) atan2f(y, x)
  #define FABS(x) fabsf(x)
  #define POW(x, y) powf(x, y)
  #define SQRT(x) sqrtf(x)
  #define CEIL(x) ceilf(x)
  #define FLOOR(x) floorf(x)
  #define LROUND(x) lroundf(x)
  #define FMOD(x, y) fmodf(x, y)
#endif

#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) ((p < 12u) ? (p) + 54u : -1)
#endif

#define CRITICAL_SECTION_START	uint32_t primask=__get_PRIMASK(); __disable_irq();
#define CRITICAL_SECTION_END    if (primask==0) __enable_irq();

// On AVR this is in math.h?
#define square(x) ((x)*(x))
#define sq(x) square(x)

#ifndef strncpy_P
  #define strncpy_P(dest, src, num) strncpy((dest), (src), (num))
#endif

// Fix bug in pgm_read_ptr
#undef pgm_read_ptr
#define pgm_read_ptr(addr) (*(addr))

#define RST_POWER_ON   1
#define RST_EXTERNAL   2
#define RST_BROWN_OUT  4
#define RST_WATCHDOG   8
#define RST_JTAG       16
#define RST_SOFTWARE   32
#define RST_BACKUP     64

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------
/** result of last ADC conversion */
extern uint32_t HAL_adc_result;
extern const PinInfo pin_map[];

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------
void Error_Handler(void);
//todo: HAL implement
//todo: arduino overrides
void pinMode(int pin_number, int mode);
void digitalWrite(int pin_number, int pin_status);
void analogWrite(int pin_number, int pin_status);
uint16_t analogRead(int adc_pin);
void delay(int milis);
void _delay_ms(int delay);
long constrain(long, long, long);
void delayMicroseconds(unsigned long);
bool digitalRead(int pin);
uint8_t pgm_read_byte_near(const signed char*);
uint8_t pgm_read_byte(const char * ptr);
float pgm_read_float_near(const float*);
uint16_t pgm_read_word(const short int*);
void serialprintPGM(const char *);
void sprintf_P(char *, const char *, ...);
bool strstr_P(const char *, const char *);
long millis();
uint64_t micros();
int freeMemory(void);

/*
 * Interrupt Control
 */
// Disable interrupts
void cli(void);
// Enable interrupts
void sei(void);

/*
 * Reboot reason Determination
 */
/** clear reset reason */
void HAL_clear_reset_source (void);
/** reset reason */
uint8_t HAL_get_reset_source (void);





// SPI: Extended functions which take a channel number (hardware SPI only)
/** Write single byte to specified SPI channel */
void spiSend(uint32_t chan, uint8_t b);
/** Write buffer to specified SPI channel */
void spiSend(uint32_t chan, const uint8_t* buf, size_t n);
/** Read single byte from specified SPI channel */
uint8_t spiRec(uint32_t chan);



// EEPROM
void eeprom_setup();
void eeprom_write_byte(unsigned char *pos, char value);
char eeprom_read_byte(uint8_t * pos);
void eeprom_read_block (void *__dst, const void *__src, size_t __n);
void eeprom_update_block (const void *__src, void *__dst, size_t __n);


// ADC
#define HAL_ANALOG_SELECT(pin)
#define HAL_START_ADC(pin)  HAL_adc_start_conversion(pin)
#define HAL_READ_ADC        HAL_adc_get_result()

void HAL_adc_init(void); //unused
void HAL_adc_start_conversion (uint8_t adc_pin); //only selects correct pin to read
uint32_t HAL_adc_get_result(void); //just reads value from array filled by ADC DMA

/*
uint16_t HAL_getAdcReading(uint8_t chan);
void HAL_startAdcConversion(uint8_t chan);
uint8_t HAL_pinToAdcChannel(int pin);
uint16_t HAL_getAdcFreerun(uint8_t chan, bool wait_for_conversion = false);
//uint16_t HAL_getAdcSuperSample(uint8_t chan);
void HAL_enable_AdcFreerun(void);
//void HAL_disable_AdcFreerun(uint8_t chan);
*/

// SPI

void spiSend(unsigned char c);
unsigned char spiRec();
void spiBegin();
void spiInit(unsigned char rate);
void spiRead(unsigned char* dst, unsigned short count);
void spiSendBlock(unsigned char token, unsigned char const* src);



// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif //_HAL_STM32F446_H
