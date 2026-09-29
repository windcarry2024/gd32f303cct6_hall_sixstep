#include "gd32f30x.h"

#ifndef BTN1_H
#define BTN1_H

typedef struct
{
    uint8_t status : 1;
} BTN1Flag;
extern BTN1Flag btn1Flag;

typedef uint8_t BTN1State;
enum
{
    BTN1_IDLE,             
    BTN1_DEBOUNCE_PRESS,   
    BTN1_PRESSED,          
    BTN1_DEBOUNCE_RELEASE, 
    BTN1_RELEASE          
};

void btn1_init(void);
void btn1_process(void);

#endif