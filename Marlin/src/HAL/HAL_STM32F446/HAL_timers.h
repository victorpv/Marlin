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
 * Description: HAL for NUCLEO_F746ZG
 *
 */


#ifndef _HAL_TIMERS_STM32F446_H
#define _HAL_TIMERS_STM32F446_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------
extern uint32_t getTimerClock;

#define FORCE_INLINE __attribute__((always_inline)) inline

#define HAL_TIMER_TYPE uint32_t

#define STEP_TIMER_NUM 0
#define TEMP_TIMER_NUM 1

//#define DISABLE_MULTI_STEPPING

//extern uint32_t hal_timer_rate;
//#define HAL_TIMER_RATE         (hal_timer_rate)
#define HAL_TIMER_RATE         (HAL_RCC_GetSysClockFreq()/2)
#define STEPPER_TIMER_PRESCALE 1
#define HAL_STEPPER_TIMER_RATE (HAL_TIMER_RATE)
#define HAL_TICKS_PER_US       (HAL_STEPPER_TIMER_RATE/1000000)

#define TEMP_TIMER_FREQUENCY   1000

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt (STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt (STEP_TIMER_NUM)

//
extern void TC3_Handler();
extern void TC4_Handler();
#define HAL_STEP_TIMER_ISR  void TC3_Handler()
#define HAL_TEMP_TIMER_ISR  void TC4_Handler()

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------
typedef struct {
  TIM_HandleTypeDef timerdef;
  IRQn_Type   IRQ_Id;
  uint32_t callback;
} tTimerConfig;

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void timers_init();
void HAL_timer_start (uint8_t timer_num, uint32_t frequency);
void HAL_timer_set_count (uint8_t timer_num, uint32_t count);

HAL_TIMER_TYPE HAL_timer_get_count (uint8_t timer_num);
uint32_t HAL_timer_get_current_count(uint8_t timer_num);

void HAL_timer_enable_interrupt (uint8_t timer_num);
void HAL_timer_disable_interrupt (uint8_t timer_num);

void HAL_timer_isr_prologue (uint8_t timer_num);



// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif // _HAL_TIMERS_STM32F446_H

