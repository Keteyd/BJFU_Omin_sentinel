#ifndef PC_TEST_REMOTE_H
#define PC_TEST_REMOTE_H
#include <stdint.h>
#define Remote_STATE_CONNECTED 1U
#define Remote_SWITCH_UP 1U
#define Const_Remote_REMOTE_OFFLINE_TIME 50U
typedef struct {
    uint8_t state;
    uint32_t last_update_time;
    struct { uint8_t s[2]; } remote;
} Remote_RemoteDataTypeDef;
Remote_RemoteDataTypeDef *Remote_GetRemoteDataPtr(void);
#endif
