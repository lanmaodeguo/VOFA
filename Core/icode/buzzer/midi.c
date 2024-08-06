//
// Created by Panda on 24-8-2.
//
#include "buzzer.h"

uint16_t music[78] = {
    330,750,
    440,375,
    494,375,
    523,750,
    587,375,
    659,375,
    587,750,
    494,375,
    392,375,
    440,1500,
    330,750,
    440,375,
    494,375,
    523,750,
    587,375,
    659,375,
    587,750,
    494,375,
    392,375,
    784,1500,
    659,750,
    698,375,
    784,375,
    880,750,
    784,375,
    698,375,
    659,750,
    587,750,
    659,750,
    523,375,
    494,375,
    440,750,
    440,375,
    494,375,
    523,750,
    523,750,
    494,750,
    392,750,
    440,3000};

void MIDI_PLAY(void)
{
    uint16_t i;
    uint32_t delay;
    for(i=0;i<39;i++)
    {
        for (delay=0;delay<music[i*2]*music[i*2+1]/1000;delay++)
        {
            HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_RESET); //蜂鸣器接口输出低电平0
            delay_us(500000/music[i*2]); //延时
            HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_SET); //蜂鸣器接口输出高电平1
            delay_us(500000/music[i*2]); //延时
        }
    }
}