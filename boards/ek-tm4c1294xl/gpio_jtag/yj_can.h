#ifndef __YJ_CAN_H__
#define __YJ_CAN_H__

#include <stdint.h>
#include <stdbool.h>


#define MSG_OBJ_TX_NUM 1
#define MSG_OBJ_RX_NUM 2





void
SimpleDelay(void);
void InitCAN(uint32_t datarate);
void InitCANTxMsg(uint32_t ID, uint8_t *buf, uint8_t len);
void InitCANRxMsg(uint32_t ID);
void SendCANMsg();
void ReceiveCANMsg();
void HandleCANMsgRx();

#endif