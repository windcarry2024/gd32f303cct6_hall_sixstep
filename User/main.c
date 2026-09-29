#include "main.h"

int main(void)
{
    sysck_init();
    systick_init();
    tim1_us_init();

    btn1_init();
    led_init();
    bldc_init();
    hall_init();
    adc_init();

    while(1)
    {
        ctrl_process();
        hall_process();
        bldc_process();
        adc_process();
        btn1_process();
        led_process();
    }
}
