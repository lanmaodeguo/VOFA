//
// Created by Panda on 24-8-6.
//

#ifndef VOFA_BSP_H
#define VOFA_BSP_H

#include "usart_bsp.h"
typedef enum
{
     VOFA_RAWDATA = 0x00U,
     VOFA_JUSTFLOAT = 0x02U,
     VOFA_FIREWATER = 0x04U,
}VOFA_MODE_TypeDef;

uint8_t VOFA_Send_Message(UART_HandleTypeDef *huart , uint16_t len,VOFA_MODE_TypeDef Mode,...);
uint8_t VOFA_Send_Message_VC (uint16_t len,VOFA_MODE_TypeDef Mode,...);
#endif //VOFA_BSP_H
