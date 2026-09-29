#include "main.h"

LedFlag ledFlag = {0};
LedState ledState = LED_IDLE;

void led_process(void)
{
    switch (ledState)
    {
        case LED_IDLE:
            if(ledFlag.status == 1)
            {
                ledState = LED_NORMAL;
                GPIO_BOP(GPIOC) |= (1U << 14);
            }
            else if(ledFlag.status == 2)
            {
                ledState = LED_ERROR;
                GPIO_BOP(GPIOC) |= (1U << 13);
            }
            break;

        case LED_NORMAL:
            if (ledFlag.status != 1)
            {
                GPIO_BOP(GPIOC) |= (1U << 30);
                ledState = LED_IDLE;
            }
            break;
            
        case LED_ERROR:
            if (ledFlag.status != 2)
            {
                GPIO_BOP(GPIOC) |= (1U << 29);
                ledState = LED_IDLE;
            }
            break;    
        
        default:
            ledState = LED_IDLE;
            break;
    }

}

void led_init(void)
{
    RCU_APB2EN |= (1U << 4);

    GPIO_CTL1(GPIOC) &= ~(0b11111111 << 20);
    GPIO_CTL1(GPIOC) |= (0b00100010 << 20);

    GPIO_BOP(GPIOC) |= (0b11 << 29);
}