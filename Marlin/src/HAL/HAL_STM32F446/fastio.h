/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
  This code contributed by Triffid_Hunter and modified by Kliment
  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/

/**
 * Description: Fast IO functions for STM32F446
 *
 * For STM32F446_H
 */

#ifndef	_FASTIO_STM32F446_H
#define	_FASTIO_STM32F446_H

/**
  utility functions
*/

//todo: make not quick and dirty
struct PinInfo {
  GPIO_TypeDef * port;
  uint16_t pin;
  TIM_HandleTypeDef* pwmTimer ;
  uint32_t pwmTimerChannel ;
};

#ifndef MASK
  #define MASK(PIN)  (1 << PIN)
#endif

/**
  magic I/O routines
  now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);
*/
#define LOW 0
#define HIGH 1

typedef enum WiringPinMode {
    OUTPUT,
    OUTPUT_OPEN_DRAIN,
	INPUT,
    INPUT_ANALOG,
    INPUT_PULLUP,
    INPUT_PULLDOWN,
    INPUT_FLOATING,
    PWM,
    PWM_OPEN_DRAIN,
} WiringPinMode;


/// Read a pin
#define _READ(IO) HAL_GPIO_ReadPin(pin_map[IO].port, pin_map[IO].pin)
/// Write to a pin
#define _WRITE_VAR(IO, v) digitalWrite(IO,v)  //these have overheads that are probably not good in the stepper ISR //HAL_GPIO_WritePin(pin_map[IO].port, pin_map[IO].pin, (GPIO_PinState(v)))
#define _WRITE(IO, v) digitalWrite(IO,v)      //HAL_GPIO_WritePin(pin_map[IO].port, pin_map[IO].pin, (GPIO_PinState(v)))
/// toggle a pin
#define _TOGGLE(IO)  HAL_GPIO_TogglePin(pin_map[IO].port, pin_map[IO].pin)

/*
 *
 * Pin Configuration Defines
 * todo: implement
 */
/// set pin as input
//todo: HAL implement
#define _SET_INPUT(IO) pinMode(IO, INPUT)

/// set pin as output
//todo: HAL implement
#define _SET_OUTPUT(IO) pinMode(IO, OUTPUT)

/// set pin as input with pullup mode
#define _PULLUP(IO, v)  { pinMode(IO, (v!=LOW ? INPUT_PULLUP : INPUT)); }

/// check if pin is an input
//#define _GET_INPUT(IO)
/// check if pin is an output
//#define _GET_OUTPUT(IO)

/// check if pin is an timer
//#define _GET_TIMER(IO)

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define READ(IO)  _READ(IO)
/// Write to a pin wrapper
#define WRITE_VAR(IO, v)  _WRITE_VAR(IO, v)
#define WRITE(IO, v)  _WRITE(IO, v)

/// toggle a pin wrapper
#define TOGGLE(IO)  _TOGGLE(IO)


/// set pin as input with pullup wrapper
#define SET_INPUT_PULLUP(IO) do{ _SET_INPUT(IO); _PULLUP(IO, HIGH); }while(0)

/// set pin as output wrapper
#define SET_OUTPUT(IO)  do{ _SET_OUTPUT(IO); _WRITE(IO, LOW); }while(0)
/// set pin as input wrapper
#define SET_INPUT(IO)  _SET_INPUT(IO)
/// check if pin is an input wrapper
//#define GET_INPUT(IO)  _GET_INPUT(IO)
/// check if pin is an output wrapper
//#define GET_OUTPUT(IO)  _GET_OUTPUT(IO)

/// check if pin is an timer wrapper
//#define GET_TIMER(IO)  _GET_TIMER(IO)

// Shorthand
#define OUT_WRITE(IO, v) { SET_OUTPUT(IO); WRITE(IO, v); }

/**
  ports and functions
*/

// UART


// TWI (I2C)


/**
pins
*/
const PinInfo pin_map[] = {
  { GPIOA, GPIO_PIN_0, &htim5, TIM_CHANNEL_1 },
  { GPIOA, GPIO_PIN_1, &htim5, TIM_CHANNEL_2 },
  { GPIOA, GPIO_PIN_2, &htim5, TIM_CHANNEL_3 },
  { GPIOA, GPIO_PIN_3 },
  { GPIOA, GPIO_PIN_4 },
  { GPIOA, GPIO_PIN_5 },
  { GPIOA, GPIO_PIN_6 },
  { GPIOA, GPIO_PIN_7 },
  { GPIOA, GPIO_PIN_8 },
  { GPIOA, GPIO_PIN_9 },
  { GPIOA, GPIO_PIN_10 },
  { GPIOA, GPIO_PIN_11 },
  { GPIOA, GPIO_PIN_12 },
  { GPIOA, GPIO_PIN_13 },
  { GPIOA, GPIO_PIN_14 },
  { GPIOA, GPIO_PIN_15 },
  { GPIOA, GPIO_PIN_0 },
  { GPIOB, GPIO_PIN_1 },
  { GPIOB, GPIO_PIN_2 },
  { GPIOB, GPIO_PIN_3 },
  { GPIOB, GPIO_PIN_4 },
  { GPIOB, GPIO_PIN_5 },
  { GPIOB, GPIO_PIN_6 },
  { GPIOB, GPIO_PIN_7 },
  { GPIOB, GPIO_PIN_8 },
  { GPIOB, GPIO_PIN_9 },
  { GPIOB, GPIO_PIN_10 },
  { GPIOB, GPIO_PIN_11 },
  { GPIOB, GPIO_PIN_12 },
  { GPIOB, GPIO_PIN_13 },
  { GPIOB, GPIO_PIN_14 },
  { GPIOB, GPIO_PIN_15 },
  { GPIOC, GPIO_PIN_0 },
  { GPIOC, GPIO_PIN_1 },
  { GPIOC, GPIO_PIN_2 },
  { GPIOC, GPIO_PIN_3 },
  { GPIOC, GPIO_PIN_4 },
  { GPIOC, GPIO_PIN_5 },
  { GPIOC, GPIO_PIN_6 },
  { GPIOC, GPIO_PIN_7 },
  { GPIOC, GPIO_PIN_8 },
  { GPIOC, GPIO_PIN_9 },
  { GPIOC, GPIO_PIN_10 },
  { GPIOC, GPIO_PIN_11 },
  { GPIOC, GPIO_PIN_12 },
  { GPIOC, GPIO_PIN_13 },
  { GPIOC, GPIO_PIN_14 },
  { GPIOC, GPIO_PIN_15 },
  { GPIOD, GPIO_PIN_0 },
  { GPIOD, GPIO_PIN_1 },
  { GPIOD, GPIO_PIN_2 },
  { GPIOD, GPIO_PIN_3 },
  { GPIOD, GPIO_PIN_4 },
  { GPIOD, GPIO_PIN_5 },
  { GPIOD, GPIO_PIN_6 },
  { GPIOD, GPIO_PIN_7 },
  { GPIOD, GPIO_PIN_8 },
  { GPIOD, GPIO_PIN_9 },
  { GPIOD, GPIO_PIN_10 },
  { GPIOD, GPIO_PIN_11 },
  { GPIOD, GPIO_PIN_12, &htim4, TIM_CHANNEL_1 },
  { GPIOD, GPIO_PIN_13, &htim4, TIM_CHANNEL_2 },
  { GPIOD, GPIO_PIN_14, &htim4, TIM_CHANNEL_3 },
  { GPIOD, GPIO_PIN_15, &htim4, TIM_CHANNEL_4 },
  { GPIOE, GPIO_PIN_0 },
  { GPIOE, GPIO_PIN_1 },
  { GPIOE, GPIO_PIN_2 },
  { GPIOE, GPIO_PIN_3 },
  { GPIOE, GPIO_PIN_4 },
  { GPIOE, GPIO_PIN_5 },
  { GPIOE, GPIO_PIN_6 },
  { GPIOE, GPIO_PIN_7 },
  { GPIOE, GPIO_PIN_8 },
  { GPIOE, GPIO_PIN_9 },
  { GPIOE, GPIO_PIN_10 },
  { GPIOE, GPIO_PIN_11 },
  { GPIOE, GPIO_PIN_12 },
  { GPIOE, GPIO_PIN_13 },
  { GPIOE, GPIO_PIN_14 },
  { GPIOE, GPIO_PIN_15 }
};

//pins defined in HAL.cpp



#endif	/* _FASTIO_STM32F446_H */
