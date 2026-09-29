#include "main.h"

#define HALL_DEB_US 5  // 霍尔去抖间隔
#define HALL_DEB_CNT_TAR 3  // 霍尔需要连续确认的计数目标
uint8_t hall_deb_cnt = 0;  // 霍尔连续确认的计数
uint16_t hall_us = 0;

HallFlag hallFlag = {0};
Hall hall = {0};

HallState hallState = HALL_IDLE;

void hall_process(void)
{
    switch (hallState)
    {
        case HALL_IDLE:
            if(hallFlag.read)
            {
                hallFlag.read = 0;

                hall.U = (GPIO_ISTAT(GPIOB) & (1U << 3))? 1 : 0;
                hall.V = (GPIO_ISTAT(GPIOB) & (1U << 4))? 1 : 0;
                hall.W = (GPIO_ISTAT(GPIOB) & (1U << 5))? 1 : 0;

                hallFlag.idx = (hall.U << 2) | (hall.V << 1) | (hall.W << 0);

                if(hallFlag.idx == 0 || hallFlag.idx == 7)
                {
                    return;
                }

                if(hallFlag.idx == hallFlag.pervIdx)
                {
                    return;
                }

                hall_us = get_us();
                hallState = HALL_DEB_CHG;
            }
            break;

        case HALL_DEB_CHG:
            if(hall_deb_cnt >= HALL_DEB_CNT_TAR)
            {
                hallFlag.pervIdx = hallFlag.idx;
                hallFlag.status = 1;
                hall_deb_cnt = 0;
                                
                hallState = HALL_IDLE;
            }

            if((uint16_t)(get_us() - hall_us) >= HALL_DEB_US)
            {
                hall.U = (GPIO_ISTAT(GPIOB) & (1U << 3))? 1 : 0;
                hall.V = (GPIO_ISTAT(GPIOB) & (1U << 4))? 1 : 0;
                hall.W = (GPIO_ISTAT(GPIOB) & (1U << 5))? 1 : 0;

                if(hallFlag.idx != ((hall.U << 2) | (hall.V << 1) | (hall.W << 0)))  // 这里要注意加括号，不然运算优先级会出错
                {
                    hall_deb_cnt = 0;
                    hallState = HALL_IDLE;
                    return;
                }

                hall_deb_cnt++;
                hall_us = get_us();
            }
            break;
            
        default:
            hallState = HALL_IDLE;
            break;
    }
}

void hall_init(void)
{
    RCU_APB1EN |= (1U << 3);
    GPIO_CTL0(GPIOB) &= ~(0b111111111111 << 12);
    GPIO_CTL0(GPIOB) |= (0b100010001000 << 12);
    GPIO_OCTL(GPIOB) |= (0b111 << 3);
}