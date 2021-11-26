#ifndef AGENTCDCILOCAL_H
#define AGENTCDCILOCAL_H
#ifdef ARGOS_simulator_BUILD

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
extern "C" {
#endif

/*-----------------------------------------------------------------------------------------------*/
/* DEBUGGING INFORMATION                                                                         */
/*                                                                                               */
/* You must define a struct called 'debug_info_t'                                                */
/*                                                                                               */
/* The name 'debug_info_t' is mandatory                                                          */
/* The content of the struct is whatever you want                                                */
/*-----------------------------------------------------------------------------------------------*/
typedef struct {
    int commitement;  // current commitement of the robot
    double quality;  // this is only needed for global communication, in order to check if robot is allowed to broadcast
    int broadcast_flag;  // used for addaptive approach in order to notify if robot wants to broadcast
    int com_range;  // used for adaptive approach to inform the kilogrid how wide the communication should be
} debug_info_t;

#ifdef __cplusplus  // If this is a C++ compiler, use C linkage
}
#endif

#endif  // ARGOS_simulator_BUILD

#endif  // AGENTCDCILOCAL
