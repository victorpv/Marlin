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
#include "eeprom.h"
#include "USBSerial.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

//-----------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

uint32_t HAL_adc_result;

/* DMA serial wrapper variables */
USBSerial USerial;

/* DMA ADC buffer for all pins */
const uint8_t adc_pins = 6;
uint16_t adc_readings[adc_pins] = {0};

#ifdef __cplusplus
extern "C" {
#endif

void USBSerial_Rx_Handler(uint8_t *data, uint16_t len){
	USerial.CDC_RxHandler(data, len);
}

#ifdef __cplusplus
}
#endif


// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

void HAL_init() {
  timers_init();

  //HAL_UART_Receive_DMA(&huart1, rxbuffer, rxbuffer_size);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_readings, 4);

  //Enable the used PWM channels

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);

  eeprom_setup();

}

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------



extern void setup();
extern void loop();

extern "C" int HAL_main() {
    HAL_init();

    setup();
    while (true) {
        loop();
    }
}


void pinMode(int pin_number, int mode) {
	uint8_t i;

	uint32_t _mode, _type, _pupd;
	bool setType = false;

	//PWM capable pins should use PWM hardware when set as outputs
	if(pin_map[pin_number].pwmTimer) {
		switch (mode) {
		case OUTPUT:
			mode = PWM;
			break;
		case OUTPUT_OPEN_DRAIN:
			mode = PWM_OPEN_DRAIN;
			break;
		}
	}

	//set pin as input, output, alternate or analog
	switch (mode) {
	case INPUT:
	case INPUT_PULLUP:
	case INPUT_PULLDOWN:
	case INPUT_FLOATING:
		_mode = GPIO_MODE_INPUT;
		break;
	case INPUT_ANALOG:
		_mode = GPIO_MODE_ANALOG;
		break;
	case OUTPUT:
	case OUTPUT_OPEN_DRAIN:
		_mode = GPIO_MODE_OUTPUT_PP;
		break;
	case PWM:
	case PWM_OPEN_DRAIN:
		_mode = GPIO_MODE_AF_PP;
	}

	//set pullups/pulldowns
	switch (mode) {
	case INPUT_PULLUP:
		_pupd = GPIO_PULLUP;
		break;
	case INPUT_PULLDOWN:
		_pupd = GPIO_PULLDOWN;
		break;
	default:
		_pupd = GPIO_NOPULL;
		break;
	}

	//set push-pull/open-drain
	switch (mode) {
	case OUTPUT:
	case PWM:
		_type = 0;
		setType = true;
		break;
	case OUTPUT_OPEN_DRAIN:
	case PWM_OPEN_DRAIN:
		_type = 1;
		setType = true;
		break;
	}

	/* Go through all pins */
	for (i = 0x00; i < 0x10; i++) {
	/* Pin is set */
		if (pin_map[pin_number].pin & (1 << i)) {
		/* Set 00 bits combination for input */

			//set pin MODE
			pin_map[pin_number].port->MODER = (pin_map[pin_number].port->MODER & ~(0x03 << (2 * i))) | (_mode << (2 * i));

			//set pin pullup/pulldown
			pin_map[pin_number].port->PUPDR = (pin_map[pin_number].port->PUPDR & ~(0x03 << (2 * i))) | ((uint32_t)(_pupd << (2 * i)));

			//set pin push-pull/open-drain
			if(setType) {
				pin_map[pin_number].port->OTYPER = (pin_map[pin_number].port->OTYPER & ~(0x01 << (i))) | ((uint32_t)(_type << (i)));
			}
		}
	}
}

void digitalWrite(int pin_number, int pin_status) {
  if (pin_map[pin_number].pwmTimer) {
    __HAL_TIM_SetCompare(pin_map[pin_number].pwmTimer, pin_map[pin_number].pwmTimerChannel,(pin_status? 255 : 0));
  } else {
    HAL_GPIO_WritePin(pin_map[pin_number].port, pin_map[pin_number].pin, (GPIO_PinState(pin_status)));
  }  
}

void analogWrite(int pin_number, int pin_status) {
   if (pin_map[pin_number].pwmTimer) {
        __HAL_TIM_SetCompare(pin_map[pin_number].pwmTimer, pin_map[pin_number].pwmTimerChannel,pin_status);
   } else {
    digitalWrite(pin_number, ((pin_status > 0) ? 1 : 0));
   }
}

bool digitalRead(int pin) {
	return HAL_GPIO_ReadPin(pin_map[pin].port, pin_map[pin].pin);
}

uint16_t analogRead(int adc_pin) {
    HAL_adc_start_conversion (adc_pin);
	return HAL_adc_get_result();
}

long millis() {
	return HAL_GetTick();
}

extern TIM_HandleTypeDef htim14;
uint64_t micros() {
    return (uint64_t)(HAL_GetTick() * 1000) + __HAL_TIM_GetCounter(&htim14);
}

void delay(int milis) {
	HAL_Delay(milis);
}

void delayMicroseconds(unsigned long us) {
	uint32_t start = micros();
	while((start+us) > micros());
}

long constrain(long val, long min, long max) {
	if(val < min) return min;
	if(val > max) return max;
	return val;
}

void serialprintPGM(const char * str){
	USerial.write(str);
	//delay(10);
}

void sprintf_P(char * target, const char * format, ...){
	//va_list va;
	//va_start(va,format);
	//vsprintf(target,format,va);
	//va_end(va);
}

bool strstr_P(const char * str1, const char * str2) {
	return strcmp(str1, str2);
}

// disable interrupts
void cli(void)
{
	__disable_irq();
}

// enable interrupts
void sei(void)
{
	__enable_irq();
}

void HAL_clear_reset_source (void)
{
	__HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t HAL_get_reset_source (void)
{
 if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
	return RST_WATCHDOG;

 if(__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
	 return RST_SOFTWARE;

 if(__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
	 return RST_EXTERNAL;

 if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
	 return RST_POWER_ON;

	return 0;
}

void _delay_ms (int delay_ms) {
	delay (delay_ms);
}

int availableMemory(int resolution = 256, int maximum = 0x2E200, bool disableInterrupts = false) {	//find real value for maximum
    if (resolution < 1) resolution = 1;
    if (maximum < 0) maximum = 0;

    int low = 0;
    int high = maximum + 1;

    if (disableInterrupts) __disable_irq();
    while (high - low > resolution) {
        int mid = (low + high) / 2;
        void* p = malloc(mid);
        if (p == NULL) {
            high = mid;
        } else {
            free(p);
            low = mid;
        }
    }
    if (disableInterrupts) __enable_irq();
    return low;
}

// return free memory between end of heap (or end bss) and whatever is current
int freeMemory()
{
	return availableMemory();
}

// --------------------------------------------------------------------------
// ADC
// --------------------------------------------------------------------------
void HAL_adc_init(void) {

}

uint8_t active_adc_pin = 0;

void HAL_adc_start_conversion (uint8_t adc_pin)
{
	switch (adc_pin) {
	case TEMP_0_PIN:
		active_adc_pin = 0;
		break;
	case TEMP_1_PIN:
		active_adc_pin = 1;
		break;
	case TEMP_2_PIN:
		active_adc_pin = 2;
		break;
	case TEMP_BED_PIN:
		active_adc_pin = 3;
		break;
	}
}

uint32_t HAL_adc_get_result(void)
{
    return adc_readings[active_adc_pin];
}

//---------------------------
//  eeprom emulation from flash
//---------------------------
uint32_t flash_start_addr = 0x8000000;
uint32_t flash_size = 0x100000;//1MB
uint32_t write_block_size = 0x8000;
uint32_t write_block_addr = (flash_start_addr + flash_size) - write_block_size;

void eeprom_setup() {
	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	/* EEPROM Init */
	if(EE_Initialise() != EE_OK)
	{
		while(1) {
			HAL_Delay(1);
		}
	}

	HAL_FLASH_Lock();

}

void eeprom_write_byte(unsigned char *pos, char value) {
	uint16_t eeprom_address = (unsigned) pos;
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	if(EE_WriteVariable(eeprom_address, (uint16_t) value) != EE_OK) {
		while(1) {
			HAL_Delay(1);
		}
	}
	HAL_FLASH_Lock();
}
char eeprom_read_byte(unsigned char *pos) {
	uint16_t data = 0xFF;
	uint16_t eeprom_address = (unsigned) pos;

	if(EE_ReadVariable(eeprom_address, &data) != EE_OK) {
		return (char) data;
	}
	return (char)data;
}
void eeprom_read_block (void *__dst, const void *__src, size_t __n) {
	uint16_t data = 0xFF;
	uint16_t eeprom_address = (unsigned) __src;

	for(uint8_t c = 0; c < __n; c++) {
		EE_ReadVariable(eeprom_address+c, &data);
		*((uint8_t*)__dst + c) = data;
	}
}
void eeprom_update_block (const void *__src, void *__dst, size_t __n) {

}
//todo: well.. itl do, perhaps
uint8_t pgm_read_byte_near(const signed char* ptr) {
    return (*ptr);
}

uint8_t pgm_read_byte(const char * ptr) {
    return (*ptr);
}

float pgm_read_float_near(const float* ptr) {
    return (*ptr);
}

uint16_t pgm_read_word(const short int* ptr) {
    return (*ptr);
}

//Blocking Delay Functions

extern "C" void u8g_xMicroDelay(uint16_t val) {
    delayMicroseconds(val); //todo: all this jumping around isn't good for timing
}
extern "C" void u8g_MicroDelay(void) {
    u8g_xMicroDelay(1);
}
extern "C" void u8g_10MicroDelay(void) {
    u8g_xMicroDelay(10);
}
//todo: the lcd class tries to initialise before the Timers, cant use HAL
extern "C" void u8g_Delay(uint16_t val) {
    u8g_xMicroDelay(val*1000);
}

// Blocking SPI functions
void spiBegin(){
    //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(1)));
}

void spiInit(unsigned char rate){
    //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(1)));
}

void spiSend(unsigned char c){
    //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(0)));

    unsigned char rx;
    if(HAL_SPI_TransmitReceive(&hspi3, &c, &rx, 1, 10000) != HAL_OK) {
        Error_Handler();
    }
    while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY);
    //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(1)));
}

unsigned char spiRec(){
   // HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(0)));
    unsigned char rx;
    unsigned char tx = 0xFF;
    if(HAL_SPI_TransmitReceive(&hspi3, &tx, &rx, 1, 10000) != HAL_OK) {
        Error_Handler();
    }
    while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY);
   // HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(1)));
    return rx;
}

void spiRead(unsigned char* dst, unsigned short count){
    //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(0)));
    /*if(HAL_SPI_Receive(&hspi3, dst, count, 1000000) != HAL_OK) {
        Error_Handler();
    }
    while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY);*/
    for (int i = 0; i < count; i++) {
      dst[i] = spiRec();
    }
    //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(1)));
}

void spiSendBlock(unsigned char token, unsigned char const* src){
   //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(0)));
   HAL_SPI_Transmit(&hspi3, &token, 1, 1000);
   while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY);
   HAL_SPI_Transmit(&hspi3, (unsigned char*)src, 512, 1000);
   while(HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_BUSY);
   //HAL_GPIO_WritePin(pin_map[SDCard_SEL].port, pin_map[SDCard_SEL].pin, (GPIO_PinState(1)));
}

#endif

