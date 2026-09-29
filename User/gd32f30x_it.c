#include "main.h"

void SysTick_Handler(void)
{
    g_ms++;
}

void TIMER0_Channel_IRQHandler(void)
{
    if(TIMER_INTF(TIMER0) & (1U << 4))
    {
        TIMER_INTF(TIMER0) &= ~(1U << 4);  // 清除CH3比较事件

        hallFlag.read = 1;
    }
}
