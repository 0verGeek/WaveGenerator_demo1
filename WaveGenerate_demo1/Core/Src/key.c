#include "key.h"

struct key bkeys[4] = {0,0,0};
uint8_t para = 0;
extern float amplitude_v;
extern float frequency_khz;
extern uint8_t mode;

void key_serv(void)
{
	for (size_t i = 0; i < 3; i++)
    {
        bkeys[i].sta = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0<<i);
        if (bkeys[i].sta == 0 && bkeys[i].last_sta == 1)
        {
            bkeys[i].short_flag = 1;
        }
        bkeys[i].last_sta = bkeys[i].sta;
    }
    bkeys[3].sta = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    if (bkeys[3].sta == 0 && bkeys[3].last_sta == 1)
    {
        bkeys[3].short_flag = 1;
    }
    bkeys[3].last_sta = bkeys[3].sta;
}
void key_proc(void)
{
    if (bkeys[0].short_flag)//按键B1
    {
        
        mode++;
        if (mode > 3)
        {
            mode = 0;
        }//使mode在0-3之间循环切换
        bkeys[0].short_flag = 0;
    }
    if (bkeys[1].short_flag)//按键B2
    {
        para = !para;//按键按下反转para，切换选中参数
        bkeys[1].short_flag = 0;
    }
    if (bkeys[2].short_flag)//按键B3
    {
        if (para == 0)//判断选中参数
        {
            if (amplitude_v < 1.6f)//参数最大值
            {
                amplitude_v += 0.1f;//参数增加
            }
        }
        if(para == 1)
        {
            if (frequency_khz <5.0f)//参数最大值
            {
                frequency_khz += 1.0f;//参数增加
            }
        }
        bkeys[2].short_flag = 0;
    }
    if (bkeys[3].short_flag)//按键B4
    {
        if (para == 0)//判断选中参数
        {
            if (amplitude_v > 0.6f)//参数最小值
            {
                amplitude_v -= 0.1f;//参数减小
            }
        }
        if(para == 1)
        {
            if (frequency_khz > 1.0f)//参数最小值
            {
                frequency_khz -= 1.0f;//参数减小
            }
        }
        bkeys[3].short_flag = 0;
    }
}