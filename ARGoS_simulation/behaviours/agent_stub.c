/*-----------------------------------------------------------------------------------------------*/
/* This file gives a template of what functions need to be implemented for Kilobot in order to   */
/* work on the Kilogrid.                                                                         */
/*-----------------------------------------------------------------------------------------------*/

// macro if we are in sim or reality -> command out if on real robot
#define SIMULATION


/*-----------------------------------------------------------------------------------------------*/
/* Imports - depending on the platform one has different imports                                 */
/*-----------------------------------------------------------------------------------------------*/

#include "kilolib.h"

#ifdef SIMULATION

#include <stdio.h>
#include <float.h>
#include "agent.h"
#include <debug.h>

#else

#include "utils.h"  // TODO check if this is needed ?!?
#include "kilob_tracking.h"
#include "kilo_rand_lib.h"
#include "../communication.h"
#include "kilob_messaging.h"

#endif


/*-----------------------------------------------------------------------------------------------*/
/* Define section here you can define values, e.g., messages types                               */
/*-----------------------------------------------------------------------------------------------*/
// options
#define UNCOMMITTED 0
// message types
#define INIT_MSG 10  // initial message from the kilogrid
#define GRID_MSG 11  // info msg from the kilogrid with option and position
#define VIRTUAL_AGENT_MSG 12  // msg forwarded from the kilogrid
#define TO_KILOGRID_MSG 62


/*-----------------------------------------------------------------------------------------------*/
/* Enum section - here we can define useful enums                                                */
/*-----------------------------------------------------------------------------------------------*/
typedef enum{
    false = 0,
    true = 1,
} bool;


/*-----------------------------------------------------------------------------------------------*/
/* Robot state variables.                                                                        */
/*-----------------------------------------------------------------------------------------------*/
uint8_t robot_gps_x;  // current x position
uint8_t robot_gps_y;  // current y position

uint8_t my_commitment = 0;  // This is the initial commitment
float my_commitment_quality = 0.0;

uint8_t communication_range = 0;  // communication range in cells



/*-----------------------------------------------------------------------------------------------*/
/* Communication variables - used for communication and stuff                                    */
/*-----------------------------------------------------------------------------------------------*/
// how often we try to send the msg - in simulation once is sufficient
#ifdef SIMULATION
#define MSG_SEND_TRIES 1
#else
#define MSG_SEND_TRIES 10
#endif
// Kilobot -> Kilogrid
uint32_t msg_counter_sent = MSG_SEND_TRIES + 1;  // counts the messages sent
uint32_t msg_number_send = 0;  // change if you want to send a msg
uint32_t msg_number_current_send = 0;  // var for checking against the last
// Kilogrid -> Kilobot
bool init_flag = false;
bool received_grid_msg_flag = false;
bool received_virtual_agent_msg_flag = false;
// message content
#ifdef SIMULATION
bool broadcast_msg = false;
uint8_t communication_range_msg = 0;
uint8_t x_pos_msg = 0;
uint8_t y_pos_msg = 0;
uint32_t msg_counter = 0;

#else
IR_message_t* message;
#endif


/*-----------------------------------------------------------------------------------------------*/
/* Arena variables                                                                               */
/*-----------------------------------------------------------------------------------------------*/
uint8_t NUMBER_OF_OPTIONS = 0;
uint8_t current_ground = 0;



/*-----------------------------------------------------------------------------------------------*/
/* Function to process the data received from the kilogrid regarding the environment             */
/*-----------------------------------------------------------------------------------------------*/
void update_grid_msg() {
    // TODO add logic here
    return;
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to process the data received from the kilogrid regarding other robots                */
/*-----------------------------------------------------------------------------------------------*/
void update_virtual_agent_msg() {
    // TODO add logic here
    return;
}


/*-----------------------------------------------------------------------------------------------*/
/* This function implements the callback, for when the robot receives an infrared message (here  */
/* only from the kilogrid)                                                                       */
/*-----------------------------------------------------------------------------------------------*/
// because there has been an "updated" version of the kilo_lib we have a slightly different
// implementation
#ifdef SIMULATION
void message_rx( message_t *msg, distance_measurement_t *d ) {
#else
void message_rx( IR_message_t *msg, distance_measurement_t *d ) {
#endif
    // check the messages
    // all data should be stored in temporary variables and then be written in the loop
    // in order to dont fuck up your calculations, because this function works like an interrupt!!
    if(msg->type == INIT_MSG && !init_flag){
        // TODO add logic ...
        // example usage
//        my_commitment = msg->data[0];
//        my_commitment_quality = msg->data[1];
//        NUMBER_OF_OPTIONS = msg->data[2];
//        option_to_sample = rand() % NUMBER_OF_OPTIONS;
//        current_ground = msg->data[3];
//        communication_range = msg->data[4];

        init_flag = true;
    }else if(msg->type == GRID_MSG && init_flag){
    // TODO add logic ...
        received_grid_msg_flag = true;
    }else if(msg->type == VIRTUAL_AGENT_MSG  && init_flag){
    // TODO add logic ...
        received_virtual_agent_msg_flag = true;
    }
    return;
}


/*-----------------------------------------------------------------------------------------------*/
/* This function implements the sending to the kilogrid. you should call this function every     */
/* loop cycle because in reality you dont have a indicator if the message was received so we     */
/* have to send it multiple times. The when and how often to send a message should be            */
/* implemented here!                                                                             */
/*-----------------------------------------------------------------------------------------------*/
void message_tx(){
    // implementation differs because in simulation we use the debugstruct - faster and easier to
    // understand
    // in reality we send infrared msg - we send more than one to make sure that the messages arrive!
    if (msg_number_current_send != msg_number_send){
        msg_number_current_send = msg_number_send;
        msg_counter_sent = 0;
    }
#ifdef SIMULATION
    // reset broadcast flag - needed in simulation to stop sending messages
    if(msg_counter_sent == MSG_SEND_TRIES){
        debug_info_set(broadcast_flag, 0);
        msg_counter_sent += 1;
    }
#endif
    // send msg if not sent enough yet
    if (msg_counter_sent < MSG_SEND_TRIES){
#ifdef SIMULATION
        // count messages
        msg_counter_sent += 1;
#else
        if((message = kilob_message_send()) != NULL) {
            msg_counter_sent += 1;

        }
#endif
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Setting values of the message                                                                 */
/*-----------------------------------------------------------------------------------------------*/
void set_message(){
    // TODO this needs to be adjusted on what kind of messages you want to send: fill in data !
#ifdef SIMULATION
    msg_number_send += 1;
    debug_info_set(broadcast_flag, 1);
    debug_info_set(type, MSG_T_VIRTUAL_ROBOT_MSG);
    debug_info_set(data0, 1);
    debug_info_set(data1, 2);
    debug_info_set(data2, 3);
    debug_info_set(data3, 4);
    debug_info_set(data4, msg_number_send);
    debug_info_set(data5, 6);
    debug_info_set(data6, 7);
    debug_info_set(data7, 8);
#else
    // sample usage
    /*
    message->type = TO_KILOGRID_MSG;
    message->data[0] = my_commitment;
    message->data[1] = communication_range;
    message->data[2] = robot_gps_x;
    message->data[3] = robot_gps_y;
    message->data[4] = msg_number_current_send;
    */
#endif
}


/*-----------------------------------------------------------------------------------------------*/
/* Init function                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
void setup(){
#ifndef SIMULATION
    // for tracking the robot in real life
    kilob_tracking_init();
    kilob_messaging_init();
    tracking_data.byte[0] = kilo_uid;
    tracking_data.byte[5] = 0;
#endif
    // Initialise motors
    set_motors(0,0);

    // TODO add logic here ...

    // Intialize time to 0
    kilo_ticks = 0;
}


/*-----------------------------------------------------------------------------------------------*/
/* Main loop                                                                                     */
/*-----------------------------------------------------------------------------------------------*/
void loop() {
    if(init_flag){  // initalization happend
        // run logic
        // process received msgs
        if (received_grid_msg_flag) {
            update_grid_msg();
            received_grid_msg_flag = false;
        }

        if (received_virtual_agent_msg_flag) {
            update_virtual_agent_msg();
            received_virtual_agent_msg_flag = false;
        }

        // TODO add logic here ...
        // example robot sends empty message to the kilogrid
        msg_counter += 1;
        if(msg_counter > 30){
            msg_counter = 0;
            set_message();
        }

        // for sending messages
        message_tx();
    }else{
        // not initialized yet ... can be omitted just for better understanding
        // also you can do some debugging here
    }

#ifdef SIMULATION
    // debug info - is now also important for the inter-robot communication, so do not delete
    debug_info_set(commitement, my_commitment);
#else
    tracking_data.byte[1] = received_x;
    tracking_data.byte[2] = received_y;
    tracking_data.byte[3] = com_range;
    tracking_data.byte[4] = msg_number_current;
    kilob_tracking(&tracking_data);
#endif
}


/*-----------------------------------------------------------------------------------------------*/
/* Main function - obviously needs to be implemented by both platforms.                          */
/*-----------------------------------------------------------------------------------------------*/
int main(){
    // initialize the hardware of the robot
    kilo_init();
    // now initialize specific things only needed for one platform
#ifdef SIMULATION
    // create debug struct - mimics the communication with the kilogrid
    debug_info_create();
#else
    utils_init();
#endif
    // callback for received messages
    kilo_message_rx = message_rx;
    // start control loop
    kilo_start(setup, loop);
    return 0;
}