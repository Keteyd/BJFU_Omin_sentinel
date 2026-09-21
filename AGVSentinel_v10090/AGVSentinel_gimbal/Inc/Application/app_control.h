#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include <stdint.h>

typedef enum {
    CONTROL_MODE_SAFE = 0,
    CONTROL_MODE_MANUAL = 1,
    CONTROL_MODE_AUTO = 2
} Control_ModeEnum;

void Control_Init(void);
void Control_Step(void);
void Control_Task(void const *argument);

#ifdef __cplusplus
}
#endif

#endif
