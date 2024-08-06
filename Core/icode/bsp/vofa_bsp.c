//
// Created by Panda on 24-8-6.
//

#include "vofa_bsp.h"
#include <stdarg.h>

uint8_t VOFA_Send_Message(UART_HandleTypeDef *huart , uint16_t len,VOFA_MODE_TypeDef Mode,...)
{

    if(Mode == VOFA_FIREWATER)//FiREWATER数据帧模式，尾帧规律
        {
            va_list valist;
            /* 为 num 个参数初始化 valist */
            va_start(valist,Mode);
            /* 访问所有赋给 valist 的参数 */
            float *temp;
            temp = va_arg(valist, float *);
            /* 清理为 valist 保留的内存 */
            va_end(valist);
            UART_Send_Data(huart,(char*)temp,len*sizeof(float));
            while (huart->gState != HAL_UART_STATE_READY){};
            //查询串口状态是否正确，延时一段时间等待恢复
            uint8_t data[4] = {0x00, 0x00, 0x80, 0x7f};
            UART_Send_Data(huart,data,4);
            while (huart->gState != HAL_UART_STATE_READY){};
            // return HAL_OK;
        }
    if (Mode == VOFA_RAWDATA)
        {
            // RAWData 无需处理
            va_list valist;
            /* 为 num 个参数初始化 valist */
            va_start(valist,Mode);
            /* 访问所有赋给 valist 的参数 */
            uint8_t *temp;
            temp = va_arg(valist, uint8_t *);
            /* 清理为 valist 保留的内存 */
            va_end(valist);
            UART_Send_Data(huart,temp,len);
            // while (huart->gState != HAL_UART_STATE_READY){};
            return HAL_OK;
        }

}