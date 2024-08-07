//
// Created by Panda on 24-8-6.
//

#include "vofa_bsp.h"
#include <stdarg.h>

#include "usbd_cdc_if.h"

uint8_t VOFA_Send_Message(UART_HandleTypeDef *huart , uint16_t len,VOFA_MODE_TypeDef Mode,...)
{

    if(Mode == VOFA_FIREWATER)//FiREWATER����֡ģʽ��β֡����
        {
            va_list valist;
            /* Ϊ num ��������ʼ�� valist */
            va_start(valist,Mode);
            /* �������и��� valist �Ĳ��� */
            float *temp;
            temp = va_arg(valist, float *);
            /* ����Ϊ valist �������ڴ� */
            va_end(valist);
            UART_Send_Data(huart,(char*)temp,len*sizeof(float));
            while (huart->gState != HAL_UART_STATE_READY){};
            //��ѯ����״̬�Ƿ���ȷ����ʱһ��ʱ��ȴ��ָ�
            uint8_t data[4] = {0x00, 0x00, 0x80, 0x7f};
            UART_Send_Data(huart,data,4);
            while (huart->gState != HAL_UART_STATE_READY){};
            // return HAL_OK;
        }
    if (Mode == VOFA_RAWDATA)
        {
            // RAWData ���账��
            va_list valist;
            /* Ϊ num ��������ʼ�� valist */
            va_start(valist,Mode);
            /* �������и��� valist �Ĳ��� */
            uint8_t *temp;
            temp = va_arg(valist, uint8_t *);
            /* ����Ϊ valist �������ڴ� */
            va_end(valist);
            UART_Send_Data(huart,temp,len);
            // while (huart->gState != HAL_UART_STATE_READY){};
            return HAL_OK;
        }

}

// uint8_t VOFA_Send_Message_VC (uint16_t len,VOFA_MODE_TypeDef Mode,...)
// {
//     if(Mode == VOFA_FIREWATER)//FiREWATER����֡ģʽ��β֡����
//     {
//         va_list valist;
//         /* Ϊ num ��������ʼ�� valist */
//         va_start(valist,Mode);
//         /* �������и��� valist �Ĳ��� */
//         float *temp;
//         temp = va_arg(valist, float *);
//         /* ����Ϊ valist �������ڴ� */
//         va_end(valist);
//         // CDC_Transmit_FS((char*)temp,len*sizeof(float));
//         // while (CDC_Transmit_FS((char*)temp,len*sizeof(float)) != USBD_OK){};
//         //��ѯ����״̬�Ƿ���ȷ����ʱһ��ʱ��ȴ��ָ�
//         uint8_t data[4] = {0x00, 0x00, 0x80, 0x7f};
//         CDC_Transmit_FS(data,4);
//         // while (CDC_Transmit_FS(data,4) != USBD_OK){};
//         return HAL_OK;
//     }
//     if (Mode == VOFA_RAWDATA)
//     {
//         // RAWData ���账��
//         va_list valist;
//         /* Ϊ num ��������ʼ�� valist */
//         va_start(valist,Mode);
//         /* �������и��� valist �Ĳ��� */
//         uint8_t *temp;
//         temp = va_arg(valist, uint8_t *);
//         /* ����Ϊ valist �������ڴ� */
//         va_end(valist);
//         // CDC_Transmit_FS(temp,len);
//         // while (CDC_Transmit_FS(temp,len) != USBD_OK){};
//         return HAL_OK;
//     }
// }