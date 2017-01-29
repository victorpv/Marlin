/* **************************************************************************
 
 Marlin 3D Printer Firmware
 Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
   
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
 * Description: HAL for TARGET_NUCLEO_F746ZG
 *
 * For TARGET_NUCLEO_F746ZG
 */

#if defined(STM32F446xx)

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../HAL.h"

#include "HAL_timers.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 2

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------
tTimerConfig timerConfig[NUM_HARDWARE_TIMERS];

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------
uint32_t hal_timer_rate = 90000000;
void timers_init() {
  //STEPPER TIMER TIM2 //use a 32bit timer 
  hal_timer_rate = HAL_RCC_GetPCLK1Freq()*2; //Timer is on APB1 with 2x multiplier
  __HAL_RCC_TIM2_CLK_ENABLE();
  timerConfig[0].timerdef.Instance               = TIM2;
  timerConfig[0].timerdef.Init.Prescaler         = ((HAL_TIMER_RATE / HAL_STEPPER_TIMER_RATE) - 1);
  timerConfig[0].timerdef.Init.CounterMode       = TIM_COUNTERMODE_UP;
  timerConfig[0].timerdef.Init.Period            = 0;
  timerConfig[0].timerdef.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  timerConfig[0].IRQ_Id = TIM2_IRQn;
  timerConfig[0].callback = (uint32_t)TC3_Handler;
  NVIC_SetPriority(timerConfig[0].IRQ_Id, 0);

  //TEMP TIMER TIM7 // any available 16bit Timer (1 already used for PWM)
  __HAL_RCC_TIM7_CLK_ENABLE();
  timerConfig[1].timerdef.Instance               = TIM7;
  timerConfig[1].timerdef.Init.Prescaler         = ((HAL_TIMER_RATE / 1000000) - 1);
  timerConfig[1].timerdef.Init.CounterMode       = TIM_COUNTERMODE_UP;
  timerConfig[1].timerdef.Init.Period            = (1000000 / 1000) - 1; //1 interupt/millisecond 999 count
  timerConfig[1].timerdef.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  timerConfig[1].IRQ_Id = TIM7_IRQn;
  timerConfig[1].callback = (uint32_t)TC4_Handler;
  NVIC_SetPriority(timerConfig[1].IRQ_Id, 2);
}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start (uint8_t timer_num, uint32_t frequency) {
  timerConfig[timer_num].timerdef.Init.Period =  ((HAL_TIMER_RATE / timerConfig[timer_num].timerdef.Init.Prescaler) / (frequency)) - 1;
  if(HAL_TIM_Base_Init(&timerConfig[timer_num].timerdef)  == HAL_OK ){
    HAL_TIM_Base_Start_IT(&timerConfig[timer_num].timerdef);
  } //else panic();
}


//forward the interrupt
extern "C" void TIM2_IRQHandler()
{
    ((void(*)(void))timerConfig[0].callback)();
}
extern "C" void TIM7_IRQHandler()
{
    ((void(*)(void))timerConfig[1].callback)();
}

void HAL_timer_set_count (uint8_t timer_num, uint32_t count) {
  __HAL_TIM_SetAutoreload(&timerConfig[timer_num].timerdef, count);
}

void HAL_timer_enable_interrupt (uint8_t timer_num) {
  NVIC_EnableIRQ(timerConfig[timer_num].IRQ_Id);
}

void HAL_timer_disable_interrupt (uint8_t timer_num) {
  NVIC_DisableIRQ(timerConfig[timer_num].IRQ_Id);
}

HAL_TIMER_TYPE HAL_timer_get_count (uint8_t timer_num) {
  return __HAL_TIM_GetAutoreload(&timerConfig[timer_num].timerdef);
}

uint32_t HAL_timer_get_current_count(uint8_t timer_num) {
  return __HAL_TIM_GetCounter(&timerConfig[timer_num].timerdef);
}

void HAL_timer_isr_prologue (uint8_t timer_num) {
  if (__HAL_TIM_GET_FLAG(&timerConfig[timer_num].timerdef, TIM_FLAG_UPDATE) == SET) {
    __HAL_TIM_CLEAR_FLAG(&timerConfig[timer_num].timerdef, TIM_FLAG_UPDATE);
  }
}

#endif

