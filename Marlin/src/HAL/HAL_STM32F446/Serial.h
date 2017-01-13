/*
 * Serial.h
 *
 *  Created on: 24 Dec 2016
 *      Author: p3p
 */

#ifndef HAL_STM32F446_SERIAL_H_
#define HAL_STM32F446_SERIAL_H_

#include "usart.h"

class HalSerial {
public:
    uint8_t *buffer;
    uint16_t buffer_size;
    uint16_t tail;
    UART_HandleTypeDef *port;

	HalSerial(UART_HandleTypeDef *huart, uint8_t *rxbuffer, uint16_t rxbuffer_size) {
	    buffer = rxbuffer;
	    buffer_size = rxbuffer_size;
	    port = huart;
	    tail = 0;
	}

	void begin(int32_t baud) {
		//todo: reinit port with new baud
	}

	char read() {
	    uint8_t c = buffer[tail++];
	    if(tail == buffer_size) {
	        tail = 0;
	    }
		return c;
	}

	//todo: use asyn api instead of blocking
	void write(char c) {
		printf("%c", c);
	}

	//NDTR amount of dma block transfer remaining, will restart at 0
	//perhaps there is a HAL function to retrieve this
	uint16_t available() {
	    uint16_t head = (buffer_size - port->hdmarx->Instance->NDTR);
		if(head < tail) {
		    return (buffer_size + head) - tail;
		}
		return head - tail;
	}

	void flush() {
		//todo: implement
	}
//todo: finish arduino wrapper and actually format using argument
	void print(const char value[]) {
		printf("%s" , value);
	}
	void print(char value, int = 0) {
		printf("%c" , value);
	}
	void print(unsigned char value, int = 0) {
		printf("%u" , value);
	}
	void print(int value, int = 0) {
		printf("%d" , value);
	}
	void print(unsigned int value, int = 0) {
		printf("%u" , value);
	}
	void print(long value, int = 0) {
		printf("%ld" , value);
	}
	void print(unsigned long value, int = 0) {
		printf("%lu" , value);
	}
	//todo: make these less stupid, link to float capable printf??
	void print(float value, int round = 6) {
		printf("%ld.%ld" , lround(value), lround( modf(value, NULL) * pow(10, round) ) );
	}
	void print(double value, int round = 6) {
        printf("%ld.%ld" , lround(value), lround( modf(value, NULL) * pow(10, round) ) );
	}

	void println(const char value[]) {
		printf("%s\n" , value);
	}
	void println(char value, int = 0) {
		printf("%c\n" , value);
	}
	void println(unsigned char value, int = 0) {
		printf("%u\r\n" , value);
	}
	void println(int value, int = 0) {
		printf("%d\n" , value);
	}
	void println(unsigned int value, int = 0) {
		printf("%u\n" , value);
	}
	void println(long value, int = 0) {
		printf("%ld\n" , value);
	}
	void println(unsigned long value, int = 0) {
		printf("%lu\n" , value);
	}
	void println(float value, int round = 6) {
        printf("%ld.%ld\n" , lround(value), lround( modf(value, NULL) * pow(10, round) ) );
	}
	void println(double value, int round = 6) {
        printf("%ld.%ld\n" , lround(value), lround( modf(value, NULL) * pow(10, round) ) );
	}
	void println(void) {
		print('\n');
	}

};


#endif /* HAL_STM32F446_SERIAL_H_ */
