#include "gd32f30x.h"

#ifndef CTRL_H
#define CTRL_H

typedef uint8_t CTRLState;
enum
{
    CTRL_IDLE,
    CTRL_WORKING,
    CTRL_STOP
};

void ctrl_process(void);

#endif