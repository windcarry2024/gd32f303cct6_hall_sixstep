#include "gd32f30x.h"

#ifndef BLDC_H
#define BLDC_H

typedef struct
{
    uint8_t high : 2;  // 0代表UH、1代表VH、2代表WH
    uint8_t low : 2;  // 0代表UL、1代表VL、2代表WL
    uint8_t idx : 3;  // 霍尔电平的索引值
} BLDCStep;
typedef uint8_t high;
enum
{
    UH,
    VH,
    WH
};
typedef uint8_t low;
enum
{
    UL,
    VL,
    WL
};

typedef struct
{
    uint8_t status : 2;  // 0代表空闲，1代表运行中，2代表霍尔异常
    uint8_t mode : 1;  // 0代表正转，1代表反转
    uint8_t align : 1;  // 0代表未预定位，1代表已预定位
    uint8_t close : 1;  // 0代表开环未完成，1代表开环已完成
    uint8_t pi : 1;  // 开环切闭环第一次不测速，0代表不允许PID，1代表允许PID
    uint8_t iStop : 1;  // PI的时候，I是否冻结，0代表无，1代表有
} BLDCFlag;
extern BLDCFlag bldcFlag;

typedef uint8_t BLDCState;
enum
{
    BLDC_IDLE,
    BLDC_ALIGN,
    BLDC_OPEN,
    BLDC_CLOSE,
    BLDC_COMM,
    BLDC_PI,
    BLDC_STOP,
};

void bldc_process(void);
void bldc_init(void);
void bldc_test(void);

#endif