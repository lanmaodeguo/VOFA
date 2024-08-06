/*
 * can.h
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#ifndef CAN_CAN1_H_
#define CAN_CAN1_H_

#include "stm32f1xx_hal.h" //HAL���ļ�����
#include <string.h>//�����ַ�������Ŀ�
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"

// extern CAN_HandleTypeDef hcan;//������HAL��ṹ��

// CAN_TxHeaderTypeDef     TxMeg;//CAN����������ؽṹ��
// CAN_RxHeaderTypeDef     RxMeg;//CAN����������ؽṹ��

#define CAN1_ID_H      0x0000 //32λ����ID���ã���16λ��
#define CAN1_ID_L      0x0000 //32λ����ID���ã���16λ��
#define CAN1_MASK_H    0x0000 //32λ����MASK���ã���16λ��
#define CAN1_MASK_L    0x0000 //32λ����MASK���ã���16λ��
#define CAN1_REC_LEN  200//����CAN1�������ֽ���

extern uint8_t  CAN1_RX_BUF[CAN1_REC_LEN];//���ջ���,���CAN1_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t CAN1_RX_STA;//����״̬���

void CAN_User_Init(CAN_HandleTypeDef* hcan  );//CAN�û���ʼ������
void  HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);//CAN���ջص�����
uint8_t  CAN1_SendNormalData(CAN_HandleTypeDef*  hcan,uint16_t ID,uint8_t *pData,uint16_t  Len);//CAN���ͺ���
void CAN1_printf (char *fmt, CAN_HandleTypeDef* hcan,...);//CAN����ͨ�ţ�ʹ��CAN1������CANר�õ�printf����

#endif /* CAN_CAN1_H_ */
