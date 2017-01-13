/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define E4_DIR_Pin GPIO_PIN_2
#define E4_DIR_GPIO_Port GPIOE
#define E4_CS_Pin GPIO_PIN_3
#define E4_CS_GPIO_Port GPIOE
#define E3_DIR_Pin GPIO_PIN_4
#define E3_DIR_GPIO_Port GPIOE
#define E3_STEP_Pin GPIO_PIN_5
#define E3_STEP_GPIO_Port GPIOE
#define E3_CS_Pin GPIO_PIN_6
#define E3_CS_GPIO_Port GPIOE
#define E2_STEP_Pin GPIO_PIN_13
#define E2_STEP_GPIO_Port GPIOC
#define E2_DIR_Pin GPIO_PIN_14
#define E2_DIR_GPIO_Port GPIOC
#define E2_CS_Pin GPIO_PIN_15
#define E2_CS_GPIO_Port GPIOC
#define THERM3_Pin GPIO_PIN_0
#define THERM3_GPIO_Port GPIOC
#define THERM2_Pin GPIO_PIN_1
#define THERM2_GPIO_Port GPIOC
#define THERM1_Pin GPIO_PIN_2
#define THERM1_GPIO_Port GPIOC
#define THERM0_Pin GPIO_PIN_3
#define THERM0_GPIO_Port GPIOC
#define F1_PWM_Pin GPIO_PIN_0
#define F1_PWM_GPIO_Port GPIOA
#define F2_PWM_Pin GPIO_PIN_1
#define F2_PWM_GPIO_Port GPIOA
#define STAT_LED_Pin GPIO_PIN_2
#define STAT_LED_GPIO_Port GPIOA
#define PS_ON_Pin GPIO_PIN_3
#define PS_ON_GPIO_Port GPIOA
#define Z_PROBE_Pin GPIO_PIN_4
#define Z_PROBE_GPIO_Port GPIOA
#define E1_STEP_Pin GPIO_PIN_4
#define E1_STEP_GPIO_Port GPIOC
#define E1_DIR_Pin GPIO_PIN_5
#define E1_DIR_GPIO_Port GPIOC
#define E1_CS_Pin GPIO_PIN_0
#define E1_CS_GPIO_Port GPIOB
#define E0_STEP_Pin GPIO_PIN_1
#define E0_STEP_GPIO_Port GPIOB
#define E0_DIR_Pin GPIO_PIN_2
#define E0_DIR_GPIO_Port GPIOB
#define E_STOP_Pin GPIO_PIN_7
#define E_STOP_GPIO_Port GPIOE
#define Z_STOP_Pin GPIO_PIN_8
#define Z_STOP_GPIO_Port GPIOE
#define Y_STOP_Pin GPIO_PIN_9
#define Y_STOP_GPIO_Port GPIOE
#define X_STOP_Pin GPIO_PIN_10
#define X_STOP_GPIO_Port GPIOE
#define E0_CS_Pin GPIO_PIN_11
#define E0_CS_GPIO_Port GPIOE
#define Z_STEP_Pin GPIO_PIN_15
#define Z_STEP_GPIO_Port GPIOE
#define Z_DIR_Pin GPIO_PIN_10
#define Z_DIR_GPIO_Port GPIOB
#define Z_CS_Pin GPIO_PIN_8
#define Z_CS_GPIO_Port GPIOD
#define Y_STEP_Pin GPIO_PIN_9
#define Y_STEP_GPIO_Port GPIOD
#define Y_DIR_Pin GPIO_PIN_10
#define Y_DIR_GPIO_Port GPIOD
#define Y_CS_Pin GPIO_PIN_11
#define Y_CS_GPIO_Port GPIOD
#define HB_PWM_Pin GPIO_PIN_12
#define HB_PWM_GPIO_Port GPIOD
#define F0_PWM_Pin GPIO_PIN_13
#define F0_PWM_GPIO_Port GPIOD
#define H1_PWM_Pin GPIO_PIN_14
#define H1_PWM_GPIO_Port GPIOD
#define H0_PWM_Pin GPIO_PIN_15
#define H0_PWM_GPIO_Port GPIOD
#define X_STEP_Pin GPIO_PIN_6
#define X_STEP_GPIO_Port GPIOC
#define X_DIR_Pin GPIO_PIN_7
#define X_DIR_GPIO_Port GPIOC
#define X_CS_Pin GPIO_PIN_8
#define X_CS_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOC
#define USD_CD_Pin GPIO_PIN_8
#define USD_CD_GPIO_Port GPIOA
#define ESP_EN_Pin GPIO_PIN_15
#define ESP_EN_GPIO_Port GPIOA
#define ESP_RST_Pin GPIO_PIN_10
#define ESP_RST_GPIO_Port GPIOC
#define BTN_ENC_Pin GPIO_PIN_11
#define BTN_ENC_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_12
#define LCD_RS_GPIO_Port GPIOC
#define BTN_EN2_Pin GPIO_PIN_0
#define BTN_EN2_GPIO_Port GPIOD
#define LCD_D4_Pin GPIO_PIN_1
#define LCD_D4_GPIO_Port GPIOD
#define LCD_D5_Pin GPIO_PIN_2
#define LCD_D5_GPIO_Port GPIOD
#define LCD_D6_Pin GPIO_PIN_3
#define LCD_D6_GPIO_Port GPIOD
#define LCD_D7_Pin GPIO_PIN_4
#define LCD_D7_GPIO_Port GPIOD
#define KILL_Pin GPIO_PIN_5
#define KILL_GPIO_Port GPIOD
#define BTN_EN1_Pin GPIO_PIN_6
#define BTN_EN1_GPIO_Port GPIOD
#define LCD_EN_Pin GPIO_PIN_7
#define LCD_EN_GPIO_Port GPIOD
#define SD_CSEL_Pin GPIO_PIN_6
#define SD_CSEL_GPIO_Port GPIOB
#define SD_DET_Pin GPIO_PIN_7
#define SD_DET_GPIO_Port GPIOB
#define STEP_EN_Pin GPIO_PIN_0
#define STEP_EN_GPIO_Port GPIOE
#define E4_STEP_Pin GPIO_PIN_1
#define E4_STEP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
