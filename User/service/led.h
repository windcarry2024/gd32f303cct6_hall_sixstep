#include "gd32f30x.h"

#ifndef LED_H
#define LED_H

typedef uint8_t LedState;
enum
{
    LED_IDLE,
    LED_NORMAL,
    LED_ERROR
};

typedef struct
{
    uint8_t status : 2;  // 0代表空闲，1代表绿灯常亮，2代表红灯常亮
} LedFlag;
extern LedFlag ledFlag;

void led_init(void);
void led_process(void);

#endif