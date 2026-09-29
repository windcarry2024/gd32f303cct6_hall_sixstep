#include "main.h"

volatile uint32_t g_ms = 0;
void systick_init(void)
{
    SysTick_Config(SystemCoreClock / 1000U);
}

void delay_us(uint32_t us)
{
    volatile uint32_t i;
    
    for (i = 0; i < us * 20; i++)
    {
        __NOP();
    }
}

void tim1_us_init(void)
{
    RCU_APB1EN |= (1U << 0);  // TIM1时钟使能

    TIMER_PSC(TIMER1) = 59;  // PSC = 59，TIM1时钟分频成：APB1 / (PSC + 1) = 1MHZ

    TIMER_CAR(TIMER1) = 0xFFFF;  // 不管CAR多少，只希望计数就行

    // 更新事件发生后，计数器继续计数 + 向上计数
    TIMER_CTL0(TIMER1) &= ~((1U << 4) | (1U << 3));

    TIMER_CTL0(TIMER1) |= (1U << 0);  // TIM1使能
}

uint16_t get_us(void)
{
    return (uint16_t)TIMER_CNT(TIMER1);
}
