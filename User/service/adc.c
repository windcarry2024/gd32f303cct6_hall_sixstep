#include "main.h"

/* 多少次更新一次平均值、时隔多久查询新值，都会影响VBUS和DM的平均值更新频率 */
#define ADC_CHECK_US 500  // 每多少微秒查询一次最新的DM和VBUS
#define ADC_VBUS_CNT_TAR 6  // VBUS取多少次的平均值
#define ADC_DM_CNT_TAR 6  // DM取多少次的平均值，一般取6，因为六相换步

#define ADC_VREF_MV 3315  // ADC参考电压，需要测量ADC实际的工作电压是多少来定

uint16_t adc_us = 0;
ADCState adcState = ADC_IDLE;
ADCFlag adcFlag = {0};
uint16_t adc_buf[2] = {0};  // [0]为DM，[1]为VBUS
uint16_t vbus = 0;  // 母线电压毫伏值
uint16_t dm = 0;  // 驱动桥母线电流毫安值
uint32_t vbus_sum = 0;
uint32_t dm_sum = 0;
uint8_t vbus_cnt = 0;
uint8_t dm_cnt = 0;
uint16_t vbus_aver = 0;  // 母线电压毫伏值
uint16_t dm_aver = 0;  // 驱动桥母线电流毫安值

void adc_process(void)
{
    switch (adcState)
    {
        case ADC_IDLE:
            if((uint16_t)(get_us() - adc_us) >= ADC_CHECK_US)  // 确保拿到不同值（其实这里只需要大于ADC规则通道组总采样时间加点冗余就行）
            {
                adcState = ADC_CHECK;
            }
            break;

        case ADC_CHECK:
            vbus = (uint32_t)(adc_buf[1] * 6 * ADC_VREF_MV / 4095);
            vbus_sum += vbus;
            vbus_cnt++;

            if(vbus_cnt >= ADC_VBUS_CNT_TAR)
            {
                vbus_aver = vbus_sum / vbus_cnt;
                vbus_sum = 0;
                vbus_cnt = 0;
            }
        
            dm = (uint32_t)(adc_buf[0] * 2 * ADC_VREF_MV / 4095);            
            dm_sum += dm;
            dm_cnt++;

            if(dm_cnt >= ADC_DM_CNT_TAR)
            {
                dm_aver = dm_sum / dm_cnt;
                dm_sum = 0;
                dm_cnt = 0;
            }
            
            adc_us = get_us();
            adcState = ADC_IDLE;
            break;
        
        default:
            adcState = ADC_IDLE;
            break;
    }
}

void adc_init(void)
{
    RCU_APB2EN |= (1U << 2);  // GPIOA时钟使能
    GPIO_CTL0(GPIOA) &= ~((0b1111 << 16) | (0b1111 << 20));  // PA4 PA5模拟输入
    
    RCU_APB2EN |=(1U << 9);  // ADC0时钟使能

    /* ADC预分频，APB2 / 8 = 15mhz */
    RCU_CFG0 &= ~((0b11 << 14) | (1U << 28));
    RCU_CFG0 |= (0b11 << 14);

    ADC_CTL0(ADC0) |= (1U << 8);  // 扫描运行模式使能

    /* 内部参考电压采样使能，最低有效位对齐，DMA请求使能，连续模式使能，常规序列外部触发使能，软件触发 */
    ADC_CTL1(ADC0) &= ~(1U << 11);
    ADC_CTL1(ADC0) |= (1U << 23) | (1U << 8) | (1U << 1) | (0b1111 << 17); 

    /* DM：采样时间55.5周期，大约3696ns，VBUS：采样时间239.5周期，大约15950ns */
    ADC_SAMPT1(ADC0) &= ~((0b111 << 12) | (0b111 << 15));
    ADC_SAMPT1(ADC0) |= (0b101 << 12) | (0b111 << 15);

    /* 常规序列长度设置为1+1 */
    ADC_RSQ0(ADC0) &= ~(0b1111 << 20);
    ADC_RSQ0(ADC0) |= (0b0001 << 20);

    /* 通道0设置为PA4->DM，通道1设置为PA5->VBUS */
    ADC_RSQ2(ADC0) &= ~((0b11111 << 0) | (0b11111 << 5));
    ADC_RSQ2(ADC0) |= (0b00100 << 0) | (0b00101 << 5);

    /* ADC分辨率设置成12位 */
    ADC_OVSAMPCTL(ADC0) &= ~(0b11 << 12);

    RCU_AHBEN |= (1U << 0);  // DMA0时钟使能

    /* 存储器到存储器使能禁止，DMA0-CH0优先级高，存储器宽度16位，外设数据宽度16位，存储器增量地址，外设固定地址，循环模式使能，外设搬到内存 */
    DMA_CH0CTL(DMA0) &= ~((1U << 14) | (0b11 << 12) | (0b11 << 10) | (0b11 << 8) | (1U << 6) | (1U << 4));
    DMA_CH0CTL(DMA0) |= (0b10 << 12) | (0b01 << 10) | (0b01 << 8) | (1U << 7) | (1U << 5);

    DMA_CH0CNT(DMA0) = 2;
    DMA_CH0PADDR(DMA0) = &ADC_RDATA(ADC0);
    DMA_CH0MADDR(DMA0) = adc_buf;

    DMA_CH0CTL(DMA0) |= (1U << 0);  // DMA0-CH0使能
    delay_us(1024);

    ADC_CTL1(ADC0) |= (1U << 0);  // ADC0使能
    delay_us(512);

    ADC_CTL1(ADC0) |= (1U << 22);  // 软件常规序列开始
    delay_us(256);  // 等待先采样几波，用于初始化变量值
}