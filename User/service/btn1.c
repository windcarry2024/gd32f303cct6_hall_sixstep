#include "main.h"

BTN1Flag btn1Flag = {0};
uint32_t btn1_ms = 0;
BTN1State btn1State = BTN1_IDLE;

void btn1_process(void)
{
    switch (btn1State)
	{
		case BTN1_IDLE:
            if (!(GPIO_ISTAT(GPIOB) & (1U << 6)))
            {
                btn1State = BTN1_DEBOUNCE_PRESS;
                btn1_ms = g_ms;
            }
			break;

		case BTN1_DEBOUNCE_PRESS:
			if(g_ms - btn1_ms >= 20)
			{
				if(!(GPIO_ISTAT(GPIOB) & (1U << 6)))
				{
					btn1State = BTN1_PRESSED;
					btn1Flag.status = 1;
				}
				else
				{
					btn1State = BTN1_IDLE;
				}
			}
			break;

		case BTN1_PRESSED:
			if((GPIO_ISTAT(GPIOB) & (1U << 6)))
			{
				btn1State = BTN1_DEBOUNCE_RELEASE;
				btn1_ms = g_ms;
			}
			break;

		case BTN1_DEBOUNCE_RELEASE:
			if(g_ms - btn1_ms >= 20)
			{
				if((GPIO_ISTAT(GPIOB) & (1U << 6)))
				{
					btn1State = BTN1_IDLE;
				}
				else
				{
					btn1State = BTN1_PRESSED;
				}
			}
			break;
		
		default:
			btn1State = BTN1_IDLE;
			break;
	}
}

void btn1_init(void)
{
    RCU_APB2EN |= (1U << 3);

    GPIO_CTL0(GPIOB) &= ~(0b1111 << 24);
    GPIO_CTL0(GPIOB) |= (0b1000 << 24);

    GPIO_OCTL(GPIOB) |= (1U << 6);
}