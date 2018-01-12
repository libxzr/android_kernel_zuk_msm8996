#ifndef _LINUX_PROXIMITY_STATUS_H
#define _LINUX_PROXIMITY_STATUS_H

typedef enum
{
    DOZE_DISABLED = 0,
    DOZE_ENABLED = 1,
    DOZE_WAKEUP = 2,
}DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;

#endif
