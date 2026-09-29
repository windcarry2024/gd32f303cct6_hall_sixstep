#include "gd32f30x.h"

#ifndef HALL_H
#define HALL_H

typedef struct
{
    uint8_t status : 1;  // 霍尔信号是否改变，0代表无，1代表有
    volatile uint8_t read : 1;  // 霍尔读取信号，0代表还没到时候，1代表到时候了
    uint8_t idx : 3;  // 霍尔信号的索引值，比如0代表霍尔信号UVW=000，1代表霍尔信号UVW=001
    uint8_t pervIdx : 3;  // 上一次有效霍尔电平信号索引值
} HallFlag;
extern HallFlag hallFlag;

typedef struct
{
    uint8_t U : 1;  // 0代表低电平，1代表高电平
    uint8_t V : 1;
    uint8_t W : 1;
} Hall;
extern Hall hall;

typedef uint8_t HallState;
enum
{
    HALL_IDLE,
    HALL_DEB_CHG,
};

void hall_init(void);
void hall_process(void);

#endif
