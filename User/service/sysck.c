#include "main.h"

void sysck_init(void)
{
    // AHB：120mhz，APB2：120mhz
    RCU_CFG0 &= ~((0b1111 << 4) | (0b111 << 8) | (0b111 << 11));

    // APB1:60mhz（最高支持）
    RCU_CFG0 |= (0b100 << 8);
}