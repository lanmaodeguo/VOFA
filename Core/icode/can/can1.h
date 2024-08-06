/*
 * can.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef CAN_CAN1_H_
#define CAN_CAN1_H_

#include "stm32f1xx_hal.h" //HAL库文件声明
#include <string.h>//用于字符串处理的库
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

// extern CAN_HandleTypeDef hcan;//声明的HAL库结构体

// CAN_TxHeaderTypeDef     TxMeg;//CAN发送设置相关结构体
// CAN_RxHeaderTypeDef     RxMeg;//CAN接收设置相关结构体

#define CAN1_ID_H      0x0000 //32位基础ID设置（高16位）
#define CAN1_ID_L      0x0000 //32位基础ID设置（低16位）
#define CAN1_MASK_H    0x0000 //32位屏蔽MASK设置（高16位）
#define CAN1_MASK_L    0x0000 //32位屏蔽MASK设置（低16位）
#define CAN1_REC_LEN  200//定义CAN1最大接收字节数

extern uint8_t  CAN1_RX_BUF[CAN1_REC_LEN];//接收缓冲,最大CAN1_REC_LEN个字节.末字节为换行符
extern uint16_t CAN1_RX_STA;//接收状态标记

void CAN_User_Init(CAN_HandleTypeDef* hcan  );//CAN用户初始化函数
void  HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);//CAN接收回调函数
uint8_t  CAN1_SendNormalData(CAN_HandleTypeDef*  hcan,uint16_t ID,uint8_t *pData,uint16_t  Len);//CAN发送函数
void CAN1_printf (char *fmt, CAN_HandleTypeDef* hcan,...);//CAN总线通信，使用CAN1，这是CAN专用的printf函数

#endif /* CAN_CAN1_H_ */
