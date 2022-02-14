/*-----------------------------------------------------------------------------------------------*/
/* This file gives a template of what functions need to be implemented for Kilobot in order to   */
/* work on the Kilogrid.                                                                         */
/*-----------------------------------------------------------------------------------------------*/

// macro if we are in sim or reality -> command out if on real robot
#define SIMULATION


/*-----------------------------------------------------------------------------------------------*/
/* Imports - depending on the platform one has different imports                                 */
/*-----------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include<time.h>
#include "kilolib.h"
#include <math.h>

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
#define PI 3.14159265358979323846
// options
#define UNCOMMITTED 0
#define UNINITIALISED 20
// message types


// counters
#define AVOIDANCE_TURNING_COUNTER_MAX 30
#define AVOIDANCE_STRAIGHT_COUNTER_MAX 300
#define STRAIGHT_COUNTER_MAX 20
#define TURNING_COUNTER_MAX 37
#define MAX_WAYPOINT_TIME 3600
#define ROTATION_SPEED 38

// parameters
#define SAMPLE_COUNTER_MAX 30
#define UPDATE_TICKS 60
#define BROADCAST_TICKS 15
//#define MAX_COMMUNICATION_RANGE 45  // should be 2 <= com range <= 45
#define MIN_COMMUNICATION_RANGE 1  // should be smaller max com range
#define COMMUNICATION_THRESHOLD_TIMER 11250 // in ticks
#define PARAM 0.0

// flags
#define CALIBRATED true


/*-----------------------------------------------------------------------------------------------*/
/* Enum section - here we can define useful enums                                                */
/*-----------------------------------------------------------------------------------------------*/
typedef enum{
    false = 0,
    true = 1,
} bool;

typedef enum{
    STOP,
    MOVE_STRAIGHT,
    MOVE_LEFT,
    MOVE_RIGHT,
    AVOIDANCE_TURN_LEFT,
    AVOIDANCE_TURN_RIGHT,
    AVOIDANCE_STRAIGHT
} state;


/*-----------------------------------------------------------------------------------------------*/
/* Robot state variables.                                                                        */
/*-----------------------------------------------------------------------------------------------*/
uint8_t robot_gps_x;  // current x position
uint8_t robot_gps_y;  // current y position

uint8_t robot_gps_x_last;
uint8_t robot_gps_y_last;

uint8_t goal_gps_x = 0;
uint8_t goal_gps_y = 0;

int32_t robot_orientation;
bool calculated_orientation = false;

uint8_t robot_commitment = UNINITIALISED;
float robot_commitment_quality = 0.0;

uint8_t last_robot_commitment = UNINITIALISED;
float last_robot_commitment_quality = 0.0;

uint8_t communication_range = 0;  // communication range in cells
uint8_t max_communication_range = 0;  // for dynamic setting

uint8_t received_option = 0;

bool robot_hit_wall = false;

uint32_t last_waypoint_time = 0;
uint32_t state_counter = 0;
state current_state = MOVE_STRAIGHT;
state last_state = STOP;

uint32_t last_commitment_switch = 0;
bool init_commitment_switch = false;
uint8_t  step_size = 1;

// sample variables
const uint32_t SAMPLE_TICKS = 32;  // TODO change back to 32 or 160
uint32_t last_sample_ticks = 0;
uint32_t sample_counter_max_noise = 0;

uint32_t sample_counter = 0;  // counts how often we sampled
uint32_t sample_op_counter = 0;  // counter on how often we encounter our op tp sample
uint8_t op_to_sample = 1;  // option we want to sample -> start with the crappy one ?

bool discovered = false;
uint8_t discovered_option = 0;
double discovered_quality = 0.0;

// commitment update variables
uint32_t last_update_ticks = 0;
uint32_t update_ticks_noise = 0;

bool new_robot_msg = false;

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
#else
IR_message_t* message;
#endif
uint8_t communication_range_msg = 0;
uint8_t x_pos_msg = 0;
uint8_t y_pos_msg = 0;
uint32_t msg_counter = 0;
// tmp variables for saving in callback function
uint8_t received_option_msg = 0;
uint8_t received_kilo_uid = 0;

uint8_t received_x = 0;
uint8_t received_y = 0;
uint8_t received_ground = 0;
uint8_t received_role = 0;

// broadcast variables
uint32_t last_broadcast_ticks = 0;


/*-----------------------------------------------------------------------------------------------*/
/* Arena variables                                                                               */
/*-----------------------------------------------------------------------------------------------*/
uint8_t NUMBER_OF_OPTIONS = 0;
uint8_t current_ground = 0;
const uint8_t CELLS_X = 20;
const uint8_t CELLS_Y = 40;

time_t t;


uint32_t normalize_angle(double angle){
    while(angle>180){
        angle=angle-360;
    }
    while(angle<-180){
        angle=angle+360;
    }

    return (int32_t) angle;
}


/*-----------------------------------------------------------------------------------------------*/
/* Retruns a random number in range_rnd.                                                         */
/*-----------------------------------------------------------------------------------------------*/
unsigned int GetRandomNumber(unsigned int range_rnd){
    int randomInt = RAND_MAX;
    while (randomInt > 30000){
        randomInt = rand();
    }
    unsigned int random = randomInt % range_rnd + 1;
    return random;

}


/*-----------------------------------------------------------------------------------------------*/
/* Sample function for the ground - sampling should take 10 sec (so the robot is able to         */
/* explore 10 grid cells)                                                                        */
/*-----------------------------------------------------------------------------------------------*/
void sample(){
    // sample every second
    if(kilo_ticks > last_sample_ticks + SAMPLE_TICKS) {
        last_sample_ticks = kilo_ticks;
        // if we are currently hitting a wall we do not want to measure the options!
        if(robot_hit_wall) return;

        // check if we reached our sampling time
        if(sample_counter < sample_counter_max_noise){
            sample_counter++;
            if (current_ground == op_to_sample){
                sample_op_counter++;
            }
        }else{ // sampling finished
            // update discovered option
            discovered_option = op_to_sample;
            discovered_quality = (float)sample_op_counter/(float)sample_counter;

            // set my quality to the measured quality if it's the robot commitment
            // also delete the last commitment, bc the robot can only store one!
            if (op_to_sample == robot_commitment){
                robot_commitment_quality = discovered_quality;
                last_robot_commitment = UNINITIALISED;
                last_robot_commitment_quality = 0.0;
            }else{
                // set discovery flag if we discovered something new!
                discovered = true;
            }

            // reset sampling
            op_to_sample = current_ground;
            sample_counter = 0;
            sample_op_counter = 0;
            // for shuffling up we set the max sample counter
            sample_counter_max_noise = SAMPLE_COUNTER_MAX + GetRandomNumber(10000) % 5;
        }
    }
}

/*-----------------------------------------------------------------------------------------------*/
/* Function for updating the commitment state (wrt to the received message)                      */
/*-----------------------------------------------------------------------------------------------*/
void update_commitment() {
    if(kilo_ticks > last_update_ticks + update_ticks_noise){
        // set next update
        last_update_ticks = kilo_ticks;
        update_ticks_noise = UPDATE_TICKS + GetRandomNumber(10000) % 5;

        // drawing a random number
        unsigned int range_rnd = 10000;
        unsigned int random = GetRandomNumber(range_rnd);

        double quality;
        bool social = false;
        bool recruitment = false;
        bool individual = false;

        // if robot made a discovery
        if(discovered){
            quality = discovered_quality;
        }else{  // robot did not sample enough yet
            quality = 0.0;
        }
        unsigned int P_qualityInt = (unsigned int) (quality * range_rnd) + 1;

        /// robot is uncommitted - it can do discovery or recruitment.
        if(robot_commitment == UNCOMMITTED){
            // DISCOVERY: with prob quality switch new option
            if(quality > 0 && random <= P_qualityInt){
                individual = true;
            }
            // RECRUITMENT: recruited by other robot
            if(new_robot_msg && received_option != UNCOMMITTED){
                social = true;
                recruitment = true;
            }
        }else{  /// robot is committed
            // COMPARE: found a better option TODO maybe choose PARAM higher than 0.0 in order to improve stability
            if(quality > robot_commitment_quality + PARAM && random <= P_qualityInt){
                individual = true;
            }
            // DIRECT-SWITCH: message with different option
            if(new_robot_msg && robot_commitment != received_option && received_option != UNCOMMITTED){
                // only switch when you are also allowed to speak otherwise you would be just listen
                social = true;
            }
        }
        // if both true do a flip
        if(individual && social) {
            if (GetRandomNumber(10000) % 2) {
                individual = true;
                social = false;
            } else {
                individual = false;
                social = true;
            }
        }

        if(individual){
            /// set new commitment
            robot_commitment = discovered_option;
            robot_commitment_quality = discovered_quality;
            // reset last robot commitment
            last_robot_commitment = UNINITIALISED;
            last_robot_commitment_quality = 0.0;
            /// set last commitment switch - trigger for updating the communication range dynamically
//            last_commitment_switch = kilo_ticks;
//            init_commitment_switch = true;
            // in case we want to try communication range based on how large the change is
//            step_size = (int)(20.0*(robot_commitment_quality/(last_robot_commitment_quality + robot_commitment_quality))); // this should be between 1 and 0.5
        }else if(social){
            /// case the robot got recruited back - never true if the robot is committed?
            if(last_robot_commitment == received_option){
                /// setting current commitment - this is executed, when robot gets recruited
                robot_commitment = received_option;
                robot_commitment_quality = last_robot_commitment_quality;
                op_to_sample = received_option;
                // reset last robot commitment
                last_robot_commitment = UNINITIALISED;
                last_robot_commitment_quality = 0.0;
            }else{  /// robot got new commitment
                if (recruitment){
                    /// RECRUITMENT - applies when the robot is uncommitted and gets a different option
                    robot_commitment = received_option;
                    robot_commitment_quality = 0.0; // thus, we first sample and then broadcast
                    op_to_sample = received_option;
                }else {
                    // remember last robot commitment
                    if (robot_commitment_quality != 0.0){
                        last_robot_commitment = robot_commitment;
                        last_robot_commitment_quality = robot_commitment_quality;
                    }
                    /// DIRECT-SWITCH: cross inhibition - becomes uncommitted
                    robot_commitment = UNCOMMITTED;
                    robot_commitment_quality = 0.0;
                    op_to_sample = current_ground;
                    /// DIRECT-SWITCH: recruited directly
//                    robot_commitment = received_option;
//                    robot_commitment_quality = 0.0;
//                    op_to_sample = received_option;
                }
            }
            /// reset sampling to make a new estimate on current commitment
            sample_op_counter = 0;
            sample_counter = 0;
            /// set last commitment switch - if we switch based on social information it is only second hand, thus we do not want to tell everybody
            init_commitment_switch= false;
        }
        new_robot_msg = false;
        discovered = false;
    }
}


void update_communication_range(){
    // TODO clean up when we exactly know what we want to do!!!!
    // here we implement different adaptive communication ranges
    uint32_t tmp_communication_range;
    uint32_t threshold_1 = COMMUNICATION_THRESHOLD_TIMER;
    uint32_t threshold_2 = 2 * threshold_1;  // TODO do we need to make this dynamic as well

    tmp_communication_range = communication_range;

    /// different update rules
    /// exponential increase
//    tmp_communication_range = (uint32_t) exp( (double)(kilo_ticks - last_commitment_switch) / 4925 );  // 2462
    /// exponential decrease
//    tmp_communication_range = (uint32_t) 45 * exp( -((double)((kilo_ticks+1) - last_commitment_switch) / 2462.0));
    /// linear increase
//    tmp_communication_range = (uint32_t) 45 * ( (double)(kilo_ticks - last_commitment_switch) / 18750);  //9375
    /// linear decrease
//    tmp_communication_range = (uint32_t) 45 * (1 - (kilo_ticks - last_commitment_switch) / 9375);
    /// on of basically step
//    if (kilo_ticks - last_commitment_switch < threshold_1) {
//        tmp_communication_range = 1;
//    }else if(kilo_ticks - last_commitment_switch < threshold_2 ){
//        tmp_communication_range = (int)(44.0/((double)(threshold_2-threshold_1)) * (double)kilo_ticks) - 43;
//    }else {
//        tmp_communication_range = 45;
//    }
    /// adaptive by changing its opinion - step
//    if (kilo_ticks - last_commitment_switch < threshold_1 && init_commitment_switch) {
//        tmp_communication_range = 45;
//    }else {
//        tmp_communication_range = 1;
//    }

    /// adaptive by changing its opinion - linear decrease
//    if (kilo_ticks - last_commitment_switch < threshold_1 && init_commitment_switch) {
//        tmp_communication_range = max_communication_range;
//    }else if (kilo_ticks - last_commitment_switch < threshold_2 && init_commitment_switch){
//        tmp_communication_range = max_communication_range - (int)((double)(max_communication_range - MIN_COMMUNICATION_RANGE)/(double)(threshold_2 - threshold_1)*((kilo_ticks - last_commitment_switch)-threshold_1));
//    }else {
//        tmp_communication_range = MIN_COMMUNICATION_RANGE;
//    }

    // check for bounds
    if (tmp_communication_range > 45) {
        tmp_communication_range = 45;
    } else if (tmp_communication_range < 1) {
        tmp_communication_range = 1;
    }

    communication_range = tmp_communication_range;
}


/*-----------------------------------------------------------------------------------------------*/
/* Setting values of the message                                                                 */
/* TODO this needs to be adjusted on what kind of messages you want to send                      */
/*-----------------------------------------------------------------------------------------------*/
void set_message(){
#ifdef SIMULATION
    msg_number_send += 1;
    debug_info_set(broadcast_flag, 1);
    debug_info_set(type, MSG_T_VIRTUAL_ROBOT_MSG);
    debug_info_set(data0, robot_gps_x);
    debug_info_set(data1, robot_gps_y);
    debug_info_set(data2, robot_commitment);
    debug_info_set(data3, communication_range);
    debug_info_set(data4, msg_number_send);
    debug_info_set(data5, kilo_uid);
    debug_info_set(data6, 7);
    debug_info_set(data7, 8);
#else
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
/* Function to broadcast the commitment message                                                  */
/*-----------------------------------------------------------------------------------------------*/
void broadcast() {
    // try to broadcast every 0.5 s
    if(kilo_ticks > last_broadcast_ticks + BROADCAST_TICKS && init_flag){
        last_broadcast_ticks = kilo_ticks;

        // share commitment with certain probability if the robot is committed
        unsigned int range_rnd = 10000;
        unsigned int random = GetRandomNumber(range_rnd);

        unsigned int P_ShareCommitementInt = (unsigned int)(robot_commitment_quality * range_rnd) + 1;

        // broadcast message if the robot is committed - with probability equal to commitment quality
        if (robot_commitment != UNCOMMITTED && robot_commitment_quality > 0 && random <= P_ShareCommitementInt){
            set_message();
        }
    }
}

/*-----------------------------------------------------------------------------------------------*/
/* Function for setting the motor speed.                                                         */
/*-----------------------------------------------------------------------------------------------*/
void set_motion(){
    // only change if motion type changed
    if(last_state != current_state){
        last_state = current_state;

        spinup_motors();
        switch(current_state) {
            case MOVE_STRAIGHT:
                if (CALIBRATED) set_motors(kilo_straight_left, kilo_straight_right);
                else set_motors(67, 67);
                break;
            case AVOIDANCE_STRAIGHT:
                if (CALIBRATED) set_motors(kilo_straight_left, kilo_straight_right);
                else set_motors(67, 67);
                break;
            case MOVE_LEFT:
                if (CALIBRATED) set_motors(kilo_turn_left, 0);
                else set_motors(70, 0);
                break;
            case MOVE_RIGHT:
                if (CALIBRATED) set_motors(0, kilo_turn_right);
                else set_motors(0, 70);
                break;
            case AVOIDANCE_TURN_LEFT:
                if (CALIBRATED) set_motors(kilo_turn_left + GetRandomNumber(10000) % 30, 0);
                else set_motors(70 + GetRandomNumber(10000) % 30, 0);
                break;
            case AVOIDANCE_TURN_RIGHT:
                if (CALIBRATED) set_motors(0, kilo_turn_right + GetRandomNumber(10000) % 30);
                else set_motors(0, 70 + GetRandomNumber(10000) % 30);
                break;

            case STOP:
            default:
                set_motors(0, 0);
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk with the random waypoint model.            */
/*-----------------------------------------------------------------------------------------------*/
void random_walk_waypoint_model(){
    do {
        // getting a random number in the range [1,GPS_maxcell-1] to avoid the border cells (upper bound is -2 because index is from 0)
        goal_gps_x = GetRandomNumber(10000)%( CELLS_X - 4 ) + 2;
        goal_gps_y = GetRandomNumber(10000)%( CELLS_Y - 4 ) + 2;
        if(abs(robot_gps_x - goal_gps_x) >= 4 || abs(robot_gps_y - goal_gps_y) >= 4){
            // if the selected cell is enough distant from the current location, it's good
            break;
        }
    } while(true);
}


// todo implement the walking method of the robot
void move(){
    // reached goal - select new waypoint
    if((goal_gps_x == robot_gps_x && goal_gps_y == robot_gps_y) || last_waypoint_time >= kilo_ticks + MAX_WAYPOINT_TIME){
        last_waypoint_time = kilo_ticks;
        random_walk_waypoint_model();
    }

    // select current state
    if(robot_hit_wall && !(current_state == AVOIDANCE_STRAIGHT || current_state == AVOIDANCE_TURN_LEFT || current_state == AVOIDANCE_TURN_RIGHT)){
        // escape from wall triggered
        int32_t angletogoal = normalize_angle(
                atan2((CELLS_Y/2) - robot_gps_y, (CELLS_X/2) - robot_gps_x) / PI * 180 -
                robot_orientation);
        // right from target
        if (angletogoal < 0) {
            current_state = AVOIDANCE_TURN_RIGHT;
        } else if (angletogoal > 0) {
            current_state = AVOIDANCE_TURN_LEFT;
        } else {
            printf("[%d] ERROR: turning calculation is fraud \n", kilo_uid);
        }
        state_counter =(uint32_t) ( fabs(angletogoal)/ROTATION_SPEED*32.0 );  // it is important to keep this as fabs bc abs introduces error that the robtos only turn on the spot

    }else if(current_state == AVOIDANCE_TURN_LEFT || current_state == AVOIDANCE_TURN_RIGHT){
        // avoidance turn -> avoidance straight
        state_counter -= 1;
        if(state_counter == 0){
            current_state = AVOIDANCE_STRAIGHT;
            state_counter = GetRandomNumber(10000) % AVOIDANCE_STRAIGHT_COUNTER_MAX + AVOIDANCE_STRAIGHT_COUNTER_MAX/2;
        }
    }else if(current_state == AVOIDANCE_STRAIGHT){
        // avoidance straight -> move straight
        state_counter -= 1;
        if(state_counter == 0){
            current_state = MOVE_STRAIGHT;
        }
    }else if(current_state == MOVE_LEFT || current_state == MOVE_RIGHT){
        // turning -> move straight
        state_counter -= 1;
        if(state_counter == 0){
            current_state = MOVE_STRAIGHT;
            state_counter = STRAIGHT_COUNTER_MAX + GetRandomNumber(10000) % (STRAIGHT_COUNTER_MAX/2);
        }
    }else if(current_state == MOVE_STRAIGHT){
        if(!(state_counter == 0)){
            state_counter -= 1;
        }else{
            if (calculated_orientation) {
                calculated_orientation = false;
                // check if robot needs to turn
                // see if we are on track
                bool right_direction = false; // flag set if we move towards the right celestial direction
                if (robot_gps_y == goal_gps_y && robot_gps_x < goal_gps_x && robot_orientation == 0) {
                    right_direction = true;
                } else if (robot_gps_y > goal_gps_y && robot_gps_x < goal_gps_x &&
                           robot_orientation == -45) {
                    right_direction = true;
                } else if (robot_gps_y > goal_gps_y && robot_gps_x == goal_gps_x &&
                           robot_orientation == -90) {
                    right_direction = true;
                } else if (robot_gps_y > goal_gps_y && robot_gps_x > goal_gps_x &&
                           robot_orientation == -135) {
                    right_direction = true;
                } else if (robot_gps_y == goal_gps_y && robot_gps_x > goal_gps_x &&
                           (robot_orientation == -180 || robot_orientation == 180)) {
                    right_direction = true;
                } else if (robot_gps_y < goal_gps_y && robot_gps_x > goal_gps_x &&
                           robot_orientation == 135) {
                    right_direction = true;
                } else if (robot_gps_y < goal_gps_y && robot_gps_x == goal_gps_x &&
                           robot_orientation == 90) {
                    right_direction = true;
                } else if (robot_gps_y < goal_gps_y && robot_gps_x < goal_gps_x &&
                           robot_orientation == 45) {
                    right_direction = true;
                } else {
                    // in this case the robot needs to turn
    //                printf("[%d] ERROR: robot orientation is of \n", kilo_uid);
                }

                // if we are not in the right direction -> turn
                if (!right_direction) {
                    int32_t angletogoal = normalize_angle(
                            atan2(goal_gps_y - robot_gps_y, goal_gps_x - robot_gps_x) / PI * 180 -
                            robot_orientation);
                    // right from target
                    if (angletogoal < 0) {
                        current_state = MOVE_RIGHT;
                    } else if (angletogoal > 0) {
                        current_state = MOVE_LEFT;
                    } else {
                        printf("[%d] ERROR: turning calculation is fraud \n", kilo_uid);
                    }
                    state_counter = (uint32_t) ( (fabs(angletogoal)/ROTATION_SPEED)*32.0 ); // it is important to keep this as fabs bc abs introduces error that the robtos only turn on the spot
                }
            }else{
                current_state = MOVE_STRAIGHT;  // TODO this is probably not needed !?!?!?
            }
        }
    }else{
        printf("[%d] ERROR: bad movement state \n", kilo_uid);
    }

    set_motion();
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to process the data received from the kilogrid regarding the environment             */
/*-----------------------------------------------------------------------------------------------*/
void update_grid_msg() {
    // check for obstacles
    if(received_role == 42){
        robot_hit_wall = true;
    }else{
        robot_hit_wall = false;
        current_ground = received_ground;
    }

    // check for position update
    if(robot_gps_x != received_x || robot_gps_y != received_y){
        robot_gps_x_last = robot_gps_x;
        robot_gps_y_last = robot_gps_y;
        robot_gps_x = received_x;
        robot_gps_y = received_y;

        // update orientation
        double angleOrientation = atan2(robot_gps_y-robot_gps_y_last, robot_gps_x-robot_gps_x_last)/PI*180;
        robot_orientation = normalize_angle(angleOrientation);
        calculated_orientation = true;
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to process the data received from the kilogrid regarding other robots                */
/*-----------------------------------------------------------------------------------------------*/
void update_virtual_agent_msg() {
    if(received_kilo_uid != kilo_uid){
        received_option = received_option_msg;
        received_kilo_uid = kilo_uid;
        // needs to be set here because we only get a new msg when we update!
        new_robot_msg = true;
    }
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
        // Initial setups can be done here because you are not in the middle of some calculation
        robot_gps_x = msg->data[0];
        robot_gps_y = msg->data[1];
        random_walk_waypoint_model();  // select first goal
        robot_commitment = msg->data[2];
        robot_commitment_quality = (msg->data[3])/255.0;
        NUMBER_OF_OPTIONS = msg->data[4];
        // how to init the robot
        // 1 -> start at option one
        // else start 50/50/50/50
        if (robot_commitment != 1){
            robot_commitment = (kilo_uid % (NUMBER_OF_OPTIONS-1)) + 2;
        }
        current_ground = msg->data[5];
        op_to_sample = current_ground;
        communication_range = msg->data[6];
        max_communication_range = msg->data[7];
        init_flag = true;
    }else if(msg->type == GRID_MSG && init_flag){
        received_x = msg->data[0];
        received_y = msg->data[1];
        received_ground = msg->data[2];
        received_role = msg->data[3];
        received_grid_msg_flag = true;
    }else if(msg->type == VIRTUAL_AGENT_MSG  && init_flag){
        // in the prcessing method we check that it is not our own msg ..
        received_option_msg = msg->data[2];
        received_kilo_uid = msg->data[3];
        received_virtual_agent_msg_flag = true;
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* This function implements the sending to the kilogrid. you should call this function every     */
/* loop cycle because in reality you dont have a indicator if the message was received so we     */
/* have to send it multiple times. The when and how often to send a message should be            */
/* implemented here!                                                                             */
/*-----------------------------------------------------------------------------------------------*/
// TODO check if this name is free, but should be - want to keep it close to the og kilobot
void message_tx(){
    // implementation differs because in simulation we use the debugstruct - faster and easier to
    // understand
    // in reality we send infrared msg - we send more than one to make sure that the messages arrive!
    if (msg_number_current_send != msg_number_send){
        msg_number_current_send = msg_number_send;
        msg_counter_sent = 0;
    }
#ifdef SIMULATION
    // TODO find a better solution currently needed to reset the broadcast flag
    // check if counter reached ... reset send flag so kilogrid knows that message stoped
    if(msg_counter_sent >= MSG_SEND_TRIES){
        debug_info_set(broadcast_flag, 0);
    }
#endif
    // send msg if not sended enough yet
    if (msg_counter_sent <= MSG_SEND_TRIES){
#ifdef SIMULATION
        // count messages
        msg_counter_sent += 1;
#else
        if((message = kilob_message_send()) != NULL) {
            /*
            message->type = TO_KILOGRID_MSG;
            message->data[0] = my_commitment;
            message->data[1] = communication_range;
            message->data[2] = robot_gps_x;
            message->data[3] = robot_gps_y;
            message->data[4] = msg_number_current_send;
            */
            msg_counter_sent += 1;

        }
#endif
    }
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
    // Initialise random seed
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    // Initialise motors
    set_motors(0,0);

    received_kilo_uid = kilo_uid;

    // init some counters
    sample_counter_max_noise = SAMPLE_COUNTER_MAX + (GetRandomNumber(10000) % SAMPLE_COUNTER_MAX);
    update_ticks_noise = UPDATE_TICKS + (GetRandomNumber(10000) % UPDATE_TICKS);

    last_broadcast_ticks = GetRandomNumber(10000) % BROADCAST_TICKS + 1;

    // Intialize time to 0
    kilo_ticks = 0;
}


/*-----------------------------------------------------------------------------------------------*/
/* Main loop                                                                                     */
/*-----------------------------------------------------------------------------------------------*/
void loop() {
    if(init_flag){  // initalization happend
        // process received msgs
        if (received_grid_msg_flag) {
            update_grid_msg();
            received_grid_msg_flag = false;
        }

        if (received_virtual_agent_msg_flag) {
            update_virtual_agent_msg();
            received_virtual_agent_msg_flag = false;
        }

        // move robot
        move();

        // sample - current ground for getting your own opinion
        sample();

        // update commitment - of the robot
        update_commitment();

        // update communication range
        update_communication_range();

        // set broadcast if needed
        broadcast();

        switch(robot_commitment) {
            case 0:
                set_color(RGB(0,0,0));
                break;
            case 1:
                set_color(RGB(3,0,0));
                break;
            case 2:
                set_color(RGB(0,3,0));
                break;
            case 3:
                set_color(RGB(0,0,3));
                break;
            case 4:
                set_color(RGB(3,3,0));
                break;
            case 5:
                set_color(RGB(0,0,0));
                break;
            default:
                //printf("[%d] ERROR - wrong state %d \n", kilo_uid, robot_commitment);
                set_color(RGB(3,3,3));
                break;
        }

        // for sending messages
        message_tx();
    }else{
        // not initialized yet ... can be omitted just for better understanding
        // also you can do some debugging here
    }

    // DEBUG SECTION - FOR SIMULATION NEEDED SO DO NOT DELETE
#ifdef SIMULATION
    // debug info - is now also important for the inter-robot communication, so do not delete
    debug_info_set(commitement, robot_commitment);
    debug_info_set(x_pos, robot_gps_x);
    debug_info_set(y_pos, robot_gps_y);
    if (robot_commitment_quality == 0.0){
        debug_info_set(inactive, 1);
    }else{
        debug_info_set(inactive, 0);
    }
    debug_info_set(com_range, communication_range);

    // debug prints
//    if(kilo_uid == 0){
//        printf("[%d] current ground: %d  op_to_sample %d  progress %d/%d  robot_commitment_quality(%d) %f \n", kilo_uid, current_ground, op_to_sample, sample_op_counter, sample_counter, robot_commitment, robot_commitment_quality);
//    }
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
    // initalize utils - TODO check if needed ?
    utils_init();
#endif
    // callback for received messages
    kilo_message_rx = message_rx;
    // start control loop
    kilo_start(setup, loop);
    return 0;
}