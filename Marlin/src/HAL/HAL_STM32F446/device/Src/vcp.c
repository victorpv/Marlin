/*
 * VCP.c
 *
 *  Created on: Jun 12, 2016
 *      Author: brosnan yuen
 */

/* Virtual communication port for STM32  USB CDC */


#include "vcp.h"



/* Create FIFO*/
FIFO RX_FIFO = {.FIFO_POS=0};


uint8_t VCP_write (uint8_t* message,uint16_t length)
{
	uint32_t startTime = HAL_GetTick();

	if ((message == NULL) || (length <= 0))
	{
		return USBD_FAIL;
	}

	uint8_t result = USBD_OK;
	do
	{
	   result = CDC_Transmit_FS(message,length);
	}
	while(result != USBD_OK && (HAL_GetTick() - startTime < 50));

	return result;
}

uint8_t copy_array(uint8_t* str1,uint8_t* str2,uint32_t length)
{
	if (length <= 0)
	{
		return 0;
	}

	uint32_t pos;
	for (pos=0;pos < length;++pos)
	{
		str1[pos] = str2[pos];
	}
	return 1;
}

extern FIFO RX_FIFO;

uint8_t VCP_read(uint8_t* Buf, uint32_t *Len)
{
	/* Check inputs */
	if ((Buf == NULL) || (Len == NULL))
	{
		return 0;
	}

    if (RX_FIFO.FIFO_POS <= 0)
    {
    	RX_FIFO.FIFO_POS = 0;
    	return 0;
    }

    /* Retrieve first data */
    *Len=RX_FIFO.FIFO_DATA_LEN[0];
    copy_array(Buf,RX_FIFO.FIFO_DATA[0],*Len);
    //Buf[*Len]='\0';



	/* Move rest of data */
	uint32_t index;

	for (index=0;index<(RX_FIFO.FIFO_POS-1);++index)
	{
		RX_FIFO.FIFO_DATA_LEN[index] = RX_FIFO.FIFO_DATA_LEN[index+1];
		copy_array(RX_FIFO.FIFO_DATA[index],RX_FIFO.FIFO_DATA[index+1],RX_FIFO.FIFO_DATA_LEN[index]);
	}

	RX_FIFO.FIFO_POS--;

    return 1;
}
