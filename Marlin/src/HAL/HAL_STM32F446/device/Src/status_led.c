#include "status_led.h"
#include "stm32f4xx_hal.h"
#include <math.h>

TIM_HandleTypeDef ledTimer;

void pulse_status_led_ms(int times, int ms) {
	ledTimer.Instance = TIM5;

	for(int i = 0; i < times; i++) {
		for(int pwm = 0; pwm < 255; pwm++) {
		  __HAL_TIM_SetCompare(&ledTimer, TIM_CHANNEL_3,pwm);
		  HAL_Delay(ms);
		}

		for(int pwm = 255; pwm >= 0; pwm--) {
		  __HAL_TIM_SetCompare(&ledTimer, TIM_CHANNEL_3,pwm);
		  HAL_Delay(ms);
		}
	}
}

void pulse_status_led(int times) {
	pulse_status_led_ms(times,1);
}
