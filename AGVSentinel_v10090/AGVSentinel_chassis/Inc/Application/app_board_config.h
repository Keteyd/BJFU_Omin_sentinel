#ifndef APP_BOARD_CONFIG_H
#define APP_BOARD_CONFIG_H

#define BOARD_ROLE_GIMBAL   1
#define BOARD_ROLE_CHASSIS  2

/*
 * Default to the gimbal-board role to preserve the original project behavior.
 * For the chassis-board firmware, define BOARD_ROLE=BOARD_ROLE_CHASSIS or
 * define BOARD_CHASSIS in the Keil target.
 */
#ifndef BOARD_ROLE
#define BOARD_ROLE BOARD_ROLE_GIMBAL
#endif

#if defined(BOARD_GIMBAL) && defined(BOARD_CHASSIS)
#error "Only one of BOARD_GIMBAL or BOARD_CHASSIS can be defined."
#endif

#if !defined(BOARD_GIMBAL) && !defined(BOARD_CHASSIS)
#if BOARD_ROLE == BOARD_ROLE_GIMBAL
#define BOARD_GIMBAL
#elif BOARD_ROLE == BOARD_ROLE_CHASSIS
#define BOARD_CHASSIS
#else
#error "Unsupported BOARD_ROLE value."
#endif
#endif

#if defined(BOARD_GIMBAL)
#define BOARD_HAS_GIMBAL        1
#define BOARD_HAS_SHOOTER       1
#define BOARD_HAS_CHASSIS       0
#define BOARD_HAS_REMOTE_UART   1
#define BOARD_HAS_INS           1
#define BOARD_HAS_PC_UART       1
#define BOARD_HAS_VISION_UART   1
#define BOARD_HAS_REFEREE_UART  0
#else
#define BOARD_HAS_GIMBAL        0
#define BOARD_HAS_SHOOTER       0
#define BOARD_HAS_CHASSIS       1
#define BOARD_HAS_REMOTE_UART   0
#define BOARD_HAS_INS           0
#define BOARD_HAS_PC_UART       0
#define BOARD_HAS_VISION_UART   0
#define BOARD_HAS_REFEREE_UART  1
#endif

/* The old wheel-leg CAN protocol is unrelated to the two-board sentinel link. */
#define BOARD_ENABLE_LEGACY_PROTOCOL 0

#endif
