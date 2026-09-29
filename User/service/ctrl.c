#include "main.h"

#define CTRL_OCP_MA 3900  // 过流保护电流阈值
#define CTRL_BATP_MV 10000  // 电池保护电压阈值

uint32_t ctrl_ms = 0;
CTRLState ctrlState = CTRL_IDLE;

void ctrl_process(void)
{
    switch (ctrlState)
    {
        case CTRL_IDLE:
            if (btn1Flag.status)
            {
                btn1Flag.status = 0;
                
                if(vbus_aver < CTRL_BATP_MV)
                {
                    /* 电池低于阈值，不给开机 */
                    return;
                }
                
                ledFlag.status = 1;
                bldcFlag.status = 1;
                
                ctrlState = CTRL_WORKING;
            }

            break;

        case CTRL_WORKING:
            if(btn1Flag.status)
            {
                btn1Flag.status = 0;
                ctrlState = CTRL_STOP;
            }

            if (vbus_aver < CTRL_BATP_MV)
            {
                /* 电池低于阈值，强制关机 */
                ctrlState = CTRL_STOP;
            }

            if (dm_aver > CTRL_OCP_MA)
            {
                /* 过流保护 */
                ctrlState = CTRL_STOP;
            }
            
            break;

        case CTRL_STOP:
            btn1Flag.status = 0;
            ledFlag.status = 0;
            bldcFlag.status = 0;

            ctrlState = CTRL_IDLE;
            break;
        
        default:
            ctrlState = CTRL_IDLE;
            break;
    }
}