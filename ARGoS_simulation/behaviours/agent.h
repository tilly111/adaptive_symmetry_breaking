#ifndef AGENTCDCILOCAL_H
#define AGENTCDCILOCAL_H

//#include "loop_functions/kilogrid_stub.h"

#ifdef ARGOS_simulator_BUILD

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
extern "C" {
#endif

typedef enum {
    /**** Kilobot to Kilobot range ****/
    MSG_T_NORMAL_IR = 0,
    MSG_T_GPS,
    MSG_T_TEST = 0x0A, // reserved for tests
    MSG_T_VIRTUAL_ROBOT_MSG,
    /**** Module to Kilobot range ****/
    MSG_T_TRACKING = 0x10,

    MSG_T_GO_STRAIGHT = 0x7D,
    MSG_T_TURN_LEFT,
    MSG_T_TURN_RIGHT,

    /**** Kilobot configuration range ****/
    MSG_T_BOOT = 0x80,
    MSG_T_BOOTPGM_PAGE,
    MSG_T_BOOTPGM_SIZE,
    MSG_T_RESET,
    MSG_T_SLEEP,
    MSG_T_WAKEUP,
    MSG_T_CHARGE,
    MSG_T_VOLTAGE,
    MSG_T_RUN,
    MSG_T_READUID,
    MSG_T_CALIB
} IR_message_type_t;

/*-----------------------------------------------------------------------------------------------*/
/* DEBUGGING INFORMATION                                                                         */
/*                                                                                               */
/* You must define a struct called 'debug_info_t'                                                */
/*                                                                                               */
/* The name 'debug_info_t' is mandatory                                                          */
/* The content of the struct is whatever you want                                                */
/*-----------------------------------------------------------------------------------------------*/
typedef struct {
    // current commitement of the robot
    int commitement;
    // this is only needed for global communication, in order to check if robot is allowed to
    // broadcast
    double quality;
    // used for adaptive approach to inform the kilogrid how wide the communication should be
    int com_range;

    // imitate communication
    // used for addaptive approach in order to notify if robot wants to broadcast
    int broadcast_flag;

    IR_message_type_t type;
    uint8_t data0;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t data4;
    uint8_t data5;
    uint8_t data6;
    uint8_t data7;

} debug_info_t;

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
}
#endif

#endif  // ARGOS_simulator_BUILD

#endif  // AGENTCDCILOCAL
