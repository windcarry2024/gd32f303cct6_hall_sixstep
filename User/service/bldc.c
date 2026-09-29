#include "main.h"

#define BLDC_CV_MAX 980  // CV可调整的最大值
#define BLDC_CV_MIN 20  // CV可调整的最小值
#define BLDC_OPEN_CV 360  // 预定位+开环时用的占空比
#define BLDC_CLOSE_CV 650  // 刚切入闭环用的占空比

#define BLDC_ALIGN_US 6  // 预定位吸住转子多少时间
#define BLDC_OPEN_US 3600  // 开环时的固定换相周期
#define BLDC_OPEN_COMM_CNT_TAR 12  // 开环时要导通过多少次相位组合
#define BLDC_TIME_OUT_RATE 5  // 等待有效霍尔电平最大时间倍率（3 即 换相周期*3）

/* 极对数：3，转速：3000RPM */
#define BLDC_RPM_TAR 3000  // RPM目标稳速
#define BLDC_POLE_PAIRS 3  // 电机极对数
#define BLDC_RPM_DEAD 1  // RPM目标稳速死区值百分比（1 即 ±1%）
#define BLDC_PI_MS 10  // 每次PI调速的间隔时间
#define BLDC_PI_P 0.2f  // PI调速的P系数
#define BLDC_PI_I 0.02f  // PI调速的I系数
#define BLDC_AVER_TAR 6  // 测速需要测多少次当平均值

BLDCStep forwardTable[6] = 
{
	{UH, VL, 1},
	{UH, WL, 3},
	{VH, WL, 2},
	{VH, UL, 6},
	{WH, UL, 4},
	{WH, VL, 5}
};
BLDCStep reverseTable[6] = 
{
    {WH, VL, 1},
    {WH, UL, 5},
    {VH, UL, 4},
    {VH, WL, 6},
    {UH, WL, 2},
    {UH, VL, 3} 
};
BLDCStep usedTable[6] = {0};

uint8_t bldcStepIndex = 0;  // 测试用

BLDCFlag bldcFlag = {0};
BLDCState bldcState = BLDC_IDLE;
BLDCState nextBLDCState = BLDC_IDLE;
BLDCStep bldcStep = {0};
BLDCStep prevBLDCStep = {0};
uint16_t bldc_us = 0;
uint32_t bldc_ms = 0;

uint16_t bldc_comm_us = 0;  // 电机当前的换相周期
uint16_t bldc_rpm = 0;  // 电机当前的RPM
uint16_t bldc_cv = 0;  // 电机当前的CV值，用于调整占空比
uint8_t bldc_open_comm_cnt = 0;  // 开环换相次数的计数

uint8_t prevCommIndex = 0;  // 上一个有效霍尔值处于换相表的索引
uint8_t commIndex = 0;  // 这次有效霍尔值处于换相表的索引
uint16_t prev_comm_us = 0;  // 上次霍尔换相的时刻
uint16_t this_comm_us = 0;  // 这次霍尔换相的时刻

uint32_t bldc_comm_us_sum = 0;  // 换相周期累积
uint8_t bldc_comm_us_cnt = 0;  // 测量了多少次换相周期
uint16_t bldc_comm_us_aver = 0xFFFF;  // 算出来的换相周期平均值

int16_t error = 0;  // 转速差值
int32_t bldc_pi_i_sum = 0;  // 误差积分值
int16_t bldc_pi_out = 0;  // 预CV值
uint16_t bldc_pi_base = 0;  // 基准值

void bldc_process(void)
{
    if(!bldcFlag.status)
    {
        TIMER_DMAINTEN(TIMER0) &= ~(1U << 4);
        nextBLDCState = BLDC_IDLE;
        bldcState = BLDC_STOP;
    }
    
    switch (bldcState)
    {
        case BLDC_IDLE:
            if (bldcFlag.status == 1)
            {
                bldc_cv = BLDC_OPEN_CV;
                bldc_open_comm_cnt = 0;
                bldcStepIndex = 0;
                bldcFlag.align = 0;
                bldcFlag.close = 0;
                bldcFlag.pi = 0;
                bldcFlag.mode = 0;
                bldc_comm_us_aver = 0xFFFF;
                bldc_pi_i_sum = 0;

                if(!bldcFlag.mode)
                {
                    bldcStep = forwardTable[0];
                }
                else
                {
                    bldcStep = reverseTable[0];
                }

                TIMER_DMAINTEN(TIMER0) |= (1U << 4);
                bldcState = BLDC_COMM;
            }
            break;

        case BLDC_ALIGN:
            if((uint16_t)(get_us() - bldc_us) >= BLDC_ALIGN_US)
            {
                bldcFlag.align = 1;
                nextBLDCState = BLDC_OPEN;
                bldcState = BLDC_STOP;
            }
            break;

        case BLDC_OPEN:
            if((uint16_t)(get_us() - bldc_us) >= BLDC_OPEN_US)
            {
                if(bldc_open_comm_cnt >= BLDC_OPEN_COMM_CNT_TAR)
                {
                    bldcFlag.close = 1;
                    hallFlag.status = 0;
                    bldc_cv = BLDC_CLOSE_CV;
                    bldc_ms = g_ms;
                    nextBLDCState = BLDC_CLOSE;
                    bldcState = BLDC_STOP;
                    return;
                }
                
                if (!bldcFlag.mode)
                {
                    bldcStep = forwardTable[bldcStepIndex];
                }
                else
                {
                    bldcStep = reverseTable[bldcStepIndex];
                }

                bldcStepIndex++;
                if(bldcStepIndex > 5)
                {
                    bldcStepIndex = 0;
                }
                
                nextBLDCState = BLDC_COMM;
                bldcState = BLDC_STOP;
            }
            break;
            
        case BLDC_PI:
            if((commIndex - prevCommIndex + 6) % 6 == 5)
            {
               bldc_comm_us = (uint16_t)(this_comm_us - prev_comm_us);
               bldc_comm_us_sum += bldc_comm_us;
               bldc_comm_us_cnt++;
               if(bldc_comm_us_cnt >= BLDC_AVER_TAR)
               {
                    if (!bldcFlag.pi)  // 防止第一次bldc_rpm还没生产出来（默认值是0的时候），就被拿去PI调速，会出问题的
                    {
                        bldcFlag.pi = 1;
                        bldc_pi_base = bldc_cv;
                    }
                    
                    bldc_comm_us_aver = bldc_comm_us_sum / bldc_comm_us_cnt;
                    bldc_rpm = 10000000 / (uint32_t)(BLDC_POLE_PAIRS * bldc_comm_us_aver);  // 后台无限生产

                    bldc_comm_us_sum = 0;
                    bldc_comm_us_cnt = 0;
               }
            }

            if(bldcFlag.pi)
            {
                if(g_ms - bldc_ms >= BLDC_PI_MS)  // PI按时消费
                {
                    if(bldc_rpm > BLDC_RPM_TAR + (uint32_t)(BLDC_RPM_TAR * BLDC_RPM_DEAD) / 100 ||
                        bldc_rpm < BLDC_RPM_TAR - (uint32_t)(BLDC_RPM_TAR * BLDC_RPM_DEAD) / 100)
                    {
                        /* PI逻辑 */
                        bldcFlag.iStop = 0;  // 清除上次PI时，可能把I冻结的标志
                        error = (int16_t)(BLDC_RPM_TAR - bldc_rpm);

                        if((bldc_cv <= BLDC_CV_MIN && error < 0) || (bldc_cv >= BLDC_CV_MAX && error > 0))   // 撞CCR墙的时候error若是等于0就允许积分
                        {
                            bldcFlag.iStop = 1;
                        }

                        if(!bldcFlag.iStop)
                        {
                            bldc_pi_i_sum += (int32_t)(BLDC_RPM_TAR - bldc_rpm);
                        }
                        
                        bldc_pi_out = bldc_pi_base + (int16_t)((float)error * BLDC_PI_P + (float)bldc_pi_i_sum * BLDC_PI_I);

                        if(bldc_pi_out >= BLDC_CV_MAX)
                        {
                            bldc_pi_out = BLDC_CV_MAX;
                        }
                        else if (bldc_pi_out <= BLDC_CV_MIN)
                        {
                            bldc_pi_out = BLDC_CV_MIN;
                        }
                        
                        bldc_cv = bldc_pi_out;

                        bldc_ms = g_ms;  // 只在真正发生PI之后，才会进入PI冷却
                    }
                }
            }

            bldcState = BLDC_COMM;
            break;
            
        case BLDC_COMM:
            switch (bldcStep.high)
            {
                case UH:
                    TIMER_CHCTL2(TIMER0) |= (1U << 0);
                    TIMER_CH0CV(TIMER0) = bldc_cv;
                    break;
            
                case VH:
                    TIMER_CHCTL2(TIMER0) |= (1U << 4);
                    TIMER_CH1CV(TIMER0) = bldc_cv;
                    break;

                case WH:
                    TIMER_CHCTL2(TIMER0) |= (1U << 8);
                    TIMER_CH2CV(TIMER0) = bldc_cv;
                    break;    
            }
        
            switch (bldcStep.low)
            {
                case UL:
                    TIMER_CHCTL2(TIMER0) |= (1U << 2);
                    TIMER_CH0CV(TIMER0) = 1000;
                    break;
            
                case VL:
                    TIMER_CHCTL2(TIMER0) |= (1U << 6);
                    TIMER_CH1CV(TIMER0) = 1000;
                    break;

                case WL:
                    TIMER_CHCTL2(TIMER0) |= (1U << 10);
                    TIMER_CH2CV(TIMER0) = 1000;
                    break;    
            }

            if (bldc_cv < 500)
            {
                TIMER_CH3CV(TIMER0) = bldc_cv + (1000 - bldc_cv) / 2;
            }
            else
            {
                TIMER_CH3CV(TIMER0) = bldc_cv / 2;
            }

            /* 因为CCR可能会变，而CCR和ARR开了预装载功能，所以需要手动事件更新快速生效 */
            TIMER_SWEVG(TIMER0) |= (1U << 0);  // 产生UPG更新事件
            TIMER_INTF(TIMER0) &= ~(1U << 0);  // 清除UPIF更新标志

            TIMER_CCHP(TIMER0) |= (1U << 15);  // POEN使能
            bldc_us = get_us();

            if(!bldcFlag.align)
            {
                bldcState = BLDC_ALIGN;
                return;
            }

            if (!bldcFlag.close)
            {
                bldc_open_comm_cnt++;  
                bldcState = BLDC_OPEN;
            }
            else
            {
                prev_comm_us = this_comm_us;
                prevCommIndex = commIndex;
                bldcState = BLDC_CLOSE;
            }
            break;
            
        case BLDC_CLOSE:
            if((uint16_t)(get_us() - bldc_us) >= bldc_comm_us_aver * BLDC_TIME_OUT_RATE)
            {
                bldcState = BLDC_IDLE;  // 重新预定位+开环
                return;
            }

            if(hallFlag.status)
            {
                hallFlag.status = 0;

                this_comm_us = get_us();

                if (!bldcFlag.mode)
                {
                    memcpy(usedTable, forwardTable, 6);
                }
                else
                {
                    memcpy(usedTable, reverseTable, 6);
                }

                for (uint8_t i = 0; i < 6; i++)
                {
                    if (usedTable[i].idx == hallFlag.idx)
                    {
                        bldcStep = usedTable[i];
                        commIndex = i;
                        break;
                    }
                }

                nextBLDCState = BLDC_PI;
                bldcState = BLDC_STOP;
            }
            break;    

        case BLDC_STOP:
            TIMER_CCHP(TIMER0) &= ~(1U << 15);  // POEN使能禁止
            TIMER_CHCTL2(TIMER0) &= ~((1U << 10) | (1U << 8) | (1U << 6) | (1U << 4) | (1U << 2) | (1U << 0));  // 所有通道禁止

            bldcState = nextBLDCState;
            break;        
        
        default:
            bldcState = BLDC_IDLE;
            break;
    }
}

void bldc_test(void)
{
    TIMER_CCHP(TIMER0) &= ~(1U << 15);  // POEN使能禁止
    
    TIMER_CHCTL2(TIMER0) &= ~((1U << 10) | (1U << 8) | (1U << 6) | (1U << 4) | (1U << 2) | (1U << 0));

    bldcStep = forwardTable[bldcStepIndex];

    bldc_cv = 650;

    switch (bldcStep.high)
    {
        case UH:
            TIMER_CHCTL2(TIMER0) |= (1U << 0);
            TIMER_CH0CV(TIMER0) = bldc_cv;
            break;

        case VH:
            TIMER_CHCTL2(TIMER0) |= (1U << 4);
            TIMER_CH1CV(TIMER0) = bldc_cv;
            break;
            
        case WH:
            TIMER_CHCTL2(TIMER0) |= (1U << 8);
            TIMER_CH2CV(TIMER0) = bldc_cv;
            break;    
    }

    switch (bldcStep.low)
    {
        case UL:
            TIMER_CHCTL2(TIMER0) |= (1U << 2);
            TIMER_CH0CV(TIMER0) = 1000;
            break;

        case VL:
            TIMER_CHCTL2(TIMER0) |= (1U << 6);
            TIMER_CH1CV(TIMER0) = 1000;
            break;
            
        case WL:
            TIMER_CHCTL2(TIMER0) |= (1U << 10);
            TIMER_CH2CV(TIMER0) = 1000;
            break;    
    }

    /* 因为CCR可能会变，而CCR和ARR开了预装载功能，所以需要手动事件更新快速生效 */
    TIMER_SWEVG(TIMER0) |= (1U << 0);  // 产生UPG更新事件
    TIMER_INTF(TIMER0) &= ~(1U << 0);  // 清除UPIF更新标志

    TIMER_CCHP(TIMER0) |= (1U << 15);  // POEN使能

    bldcStepIndex++;
    if(bldcStepIndex > 5)
	{
		bldcStepIndex = 0;
	}
}

void bldc_init(void)
{
    /* GPIOA、GPIOB、AF时钟使能 */
    RCU_APB2EN |= (1U << 2) | (1U << 3) | (1U << 0);

    /* PA8-CH0-UH PA9-CH1-VH PA10-CH2-WH PB13-CH0N-UL PB14-CH1N-VL PB15-CH2N-WL */
    GPIO_OCTL(GPIOA) &= ~(0b111 << 8);
    GPIO_CTL1(GPIOA) &= ~(0xFFF << 0);
    GPIO_CTL1(GPIOA) |= (0xAAA << 0);
    GPIO_OCTL(GPIOB) &= ~(0b111 << 13);
    GPIO_CTL1(GPIOB) &= ~(0xFFF << 20);
    GPIO_CTL1(GPIOB) |= (0xAAA << 20);
    
    /* TIM0时钟使能 */
    RCU_APB2EN |= (1U << 11);
    
    /* 死区时间：Fdts = Fck_timer，边沿对齐模式，CNT向上计数，连续脉冲模式，允许写UPG更新TIM事件，允许TIM更新事件 */
    TIMER_CTL0(TIMER0) &= ~((0b11 << 8) | (0b11 << 5) | (1U << 4) | (1U << 3) | (1U << 2) | (1U << 1));
    /* CAR重载影子使能 */
    TIMER_CTL0(TIMER0) |= (1U << 7);

    /* CH0 CH0N CH1 CH1N CH2 CH2N的空闲电平设置为低电平，TI0S(硬件识别霍尔)禁用，无视TRGO信号(TIM通知其他外设工作)，DMAS无视，换相控制影子寄存器控制无视，换相控制影子使能关闭 */
    TIMER_CTL1(TIMER0) &= ~((0b111111 << 8) | (1U << 7) | (0b111 << 4) | (1U << 3) | (1U << 2) | (1U << 0));

    /* 从模式配置寄存器无视 */
    TIMER_SMCFG(TIMER0) &= ~(0xFFFF) << 0;

    /* DMA 和中断使能寄存器无视 */
    TIMER_DMAINTEN(TIMER0) &= ~(0xFFFF) << 0;

    /* 中断标志寄存器除了UPIF，其余都无视 */
    TIMER_INTF(TIMER0) &= ~(0xFFFF) << 0;

    /* CH0：ETIFP信号无视，PWM0模式，CV影子使能，快速比较使能关闭，输出模式 */
    /* CH1：ETIFP信号无视，PWM0模式，CV影子使能，快速比较使能关闭，输出模式 */
    /* CH2：ETIFP信号无视，PWM0模式，CV影子使能，快速比较使能关闭，输出模式 */
    TIMER_CHCTL0(TIMER0) &= ~((1U << 7) | (1U << 2) | (0b11 << 0) | (1U << 15) | (1U << 10) | (0b11 << 8));
    TIMER_CHCTL0(TIMER0) |= (0b110 << 4) | (1U << 3) | (0b110 << 12) | (1U << 11);
    TIMER_CHCTL1(TIMER0) &= ~((1U << 7) | (1U << 2) | (0b11 << 0));
    TIMER_CHCTL1(TIMER0) |= (0b110 << 4) | (1U << 3);

    /* 设置CH0~2和CH0~2N的极性为高电平有效，并且全部通道使能禁止 */
    TIMER_CHCTL2(TIMER0) &= ~(0xFFFF) << 0;

    /* 目标PWM频率是20khz，目标Tick是1000 */
    TIMER_PSC(TIMER0) = 5;
    TIMER_CAR(TIMER0) = 1000;

    /* 重复计数寄存器无视 */
    TIMER_CREP(TIMER0) &= ~(0xFFFF) << 0;

    /* POEN仅软件使能，中止极性无视，中止无视，ROS=0，IOS=0，禁能保护模式 */
    TIMER_CCHP(TIMER0) &= ~((1U << 14) | (1U << 13) | (1U << 12) | (1U << 11) | (1U << 10) | (0b11 << 8) | (0b11111111 << 0));
    TIMER_CCHP(TIMER0) |= (0b00011001 << 0);  // 死区时间：207.5ns

    /* DMA配置寄存器无视 */
    TIMER_DMACFG(TIMER0) &= ~(0xFFFF) << 0;

    /* DMA发送缓冲区寄存器无视 */
    TIMER_DMATB(TIMER0) &= ~(0xFFFF) << 0;

    /* 配置寄存器无视 */
    TIMER_CFG(TIMER0) &= ~(0xFFFF) << 0;

    /* 所有通道默认占空比为50% */
    TIMER_CH0CV(TIMER0) = 500;
    TIMER_CH1CV(TIMER0) = 500;
    TIMER_CH2CV(TIMER0) = 500;

    /* CH3定时触发中断配置 */
    TIMER_CH3CV(TIMER0) = 250;  // 默认25%占空比触发中断采样
    TIMER_DMAINTEN(TIMER0) &= ~(1U << 4);  // CH3中断使能默认禁止
    TIMER_INTF(TIMER0) &= ~(1U << 4);  // 清除CH3比较事件
    NVIC_SetPriority(TIMER0_Channel_IRQn, 1);
    NVIC_EnableIRQ(TIMER0_Channel_IRQn);

    TIMER_CNT(TIMER0) = 0;  // CNT计数初始化

    TIMER_CCHP(TIMER0) &= ~(1U << 15);  // POEN使能禁止

    TIMER_SWEVG(TIMER0) |= (1U << 0);  // 产生UPG更新事件
    TIMER_INTF(TIMER0) &= ~(1U << 0);  // 清除UPIF更新标志

    TIMER_CTL0(TIMER0) |= (1U << 0);  // TIM0使能
}