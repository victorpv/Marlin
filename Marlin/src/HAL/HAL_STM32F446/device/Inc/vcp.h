/*
 * VCP.h
 *
 *  Created on: Jun 12, 2016
 *      Author: brosnan
 */

/* Virtual communication port for STM32  USB CDC */

#ifndef VCP_H_
#define VCP_H_


#include "stm32f4xx_hal.h"
#include "usb_device.h"
//#include "usbd_cdc_if.h"

#define  FIFO_SIZE 4
#define  FIFO_WIDTH 256

/* Structure of FIFO*/
typedef struct FIFO
{
	uint32_t FIFO_POS;
    uint8_t FIFO_DATA[FIFO_SIZE][FIFO_WIDTH];
    uint32_t FIFO_DATA_LEN[FIFO_SIZE];
} FIFO;

extern FIFO RX_FIFO;

/**
  * @brief  VCP_write
  *         Send data to the PC using CDC
  *
  *
  * @param  message: Buffer of data to be send
  * @param  length: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t VCP_write (uint8_t* message,uint16_t length);

/**
  * @brief  copy_array
  *         Copy data from str2 to str1
  *
  *
  * @param  str1: Destination
  * @param  str2: Origin
  * @param  length: Strings
  * @retval Result of the operation: 0 if failed, 1 if succeed
  */
uint8_t copy_array(uint8_t* str1,uint8_t* str2,uint32_t length);


/**
  * @brief  VCP_write
  *         Receive data from the PC using CDC
  *
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: 1 if there is data to be processed, 0 if there is no data
  */
uint8_t VCP_read(uint8_t* Buf, uint32_t *Len);


#endif /* VCP_H_ */
