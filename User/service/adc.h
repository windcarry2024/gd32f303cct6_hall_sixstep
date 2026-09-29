#include "gd32f30x.h"

#ifndef ADC_H
#define ADC_H

extern uint16_t vbus_aver;
extern uint16_t dm_aver;

typedef struct
{
    uint8_t vbus : 1;  // vbus取平均值完毕
    uint8_t dm : 1;  // dm取平均值完毕
} ADCFlag;
extern ADCFlag adcFlag;

typedef uint8_t ADCState;
enum
{
    ADC_IDLE,
    ADC_CHECK
};

void adc_init(void);
void adc_process(void);

#endif