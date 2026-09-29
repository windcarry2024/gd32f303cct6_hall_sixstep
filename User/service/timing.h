#include "gd32f30x.h"

#ifndef TIMING_H
#define TIMING_H

// 内核1ms计时
extern volatile uint32_t g_ms;
void systick_init(void);

// 微秒延时函数
void delay_us(uint32_t us);

// TIM1微秒计时
void tim1_us_init(void);
uint16_t get_us(void);

#endif