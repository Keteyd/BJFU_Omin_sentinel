#ifndef APP_YAW_IDENTIFICATION_H
#define APP_YAW_IDENTIFICATION_H
#include "module_yaw_burst.h"
#include "module_yaw_ident_wire.h"

typedef union {
    YawBurst_Buffer legacy;
    YawIdent_Record records[YAW_IDENT_WIRE_SAMPLES];
    YawIdent_TraceRecord trace[CAN_TRACE_CAPACITY];
} YawIdent_Storage;
extern YawIdent_Storage YawIdent_StorageData;

uint8_t YawIdentApp_OwnsControl(void);
uint8_t YawIdentApp_IsOnline(void);
uint8_t YawIdentApp_AllowYaw(void);
float YawIdentApp_BigReference(void);
uint8_t YawIdentApp_UsesSpeedReference(void);
float YawIdentApp_BigSpeedReferenceRpm(void);
float YawIdentApp_SmallSpeedReferenceRpm(void);
void YawIdentApp_Receive(const uint8_t payload[12]);
void YawIdentApp_Pre(void);
void YawIdentApp_Post(void);
void YawIdentApp_ValidateOutput(void);
void YawIdentApp_Stop(uint8_t reason);
#endif
