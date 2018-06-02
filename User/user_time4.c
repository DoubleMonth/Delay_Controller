#include "user_time4.h"
//2ms¶¨Ê±
void time4Init()
{
    TIM4_TimeBaseInit(TIM4_PRESCALER_128, 250);//       1/(16000000/128)*250=2ms
    TIM4_ARRPreloadConfig(ENABLE);
    TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
    TIM4_Cmd(ENABLE);
}

