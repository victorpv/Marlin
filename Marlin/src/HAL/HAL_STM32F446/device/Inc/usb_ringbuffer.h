#ifndef __USBD_RING_H
#define __USBD_RING_H

#ifdef __cplusplus
 extern "C" {
#endif

#define APP_RX_DATA_SIZE  64
#define APP_TX_DATA_SIZE  64

 extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
 extern volatile uint32_t rxReadIndex;
 extern volatile uint32_t rxWriteIndex;
 extern volatile uint32_t rxBuffLength;


#ifdef __cplusplus
}
#endif

#endif /* __USBD_RING_H */
