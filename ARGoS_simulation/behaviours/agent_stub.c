/*-----------------------------------------------------------------------------------------------*/
/* This file gives a template of what functions need to be implemented for Kilobot in order to   */
/* work on the Kilogrid.                                                                         */
/*-----------------------------------------------------------------------------------------------*/

// macro if we are in sim or reality -> command out if on real robot
#define SIMULATION
#define CROSS_INHIBITION
//#define RECRUITBACK
//#define COMPARE_PROB


/*-----------------------------------------------------------------------------------------------*/
/* Imports - depending on the platform one has different imports                                 */
/*-----------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include "kilolib.h"
#include <math.h>

#ifdef SIMULATION

#include <stdio.h>
#include <float.h>
#include "agent.h"
#include <debug.h>

#else

#include "utils.h"
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
#define AVOIDANCE_STRAIGHT_COUNTER_MAX 300
#define STRAIGHT_COUNTER_MAX 20
#define MAX_WAYPOINT_TIME 3600
#define ROTATION_SPEED 38

// parameters
// #define SAMPLE_COUNTER_MAX 30
// TODO made dynaic for ants paper
uint8_t SAMPLE_COUNTER_MAX = 0;
#define SAMPLE_TICKS 150
#define UPDATE_TICKS 60
#define BROADCAST_TICKS 15
#define MIN_COMMUNICATION_RANGE 1  // is used in dynamic com update
#define COMMUNICATION_THRESHOLD_TIMER 1875 // in ticks - should be 1 min????
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
bool robot_hit_semi_wall = false;

uint32_t last_waypoint_time = 0;
uint32_t state_counter = 0;
state current_state = MOVE_STRAIGHT;
state last_state = STOP;

uint32_t last_commitment_switch = 0;
bool commitment_switch_flag = false;
uint8_t  step_size = 1;

// sample variables
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
//bool sample_time_estimate_flag = false;
#else
IR_message_t* message;
uint32_t msg_counter = 0;
#endif

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
/* Sample function for the ground - sampling should take 30 sec                                  */
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
//            sample_time_estimate_flag = true;
//             printf("[%d] estimate %f for option %d \n", kilo_uid, discovered_quality, discovered_option);

            // set my quality to the measured quality if it's the robot commitment
            // also delete the last commitment, bc the robot can only store one!
            if (op_to_sample == robot_commitment){
                last_commitment_switch = kilo_ticks;
                commitment_switch_flag = true;
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
            // TODO no noise on the sample time for ants - add some noise in order to make it work
            //  and that it is not a random switch at some point
            //  +1 needed at modulo bc otherwise it is undefined for 5 samples
            sample_counter_max_noise = SAMPLE_COUNTER_MAX + ((GetRandomNumber(10000) % ((uint8_t)(SAMPLE_COUNTER_MAX/10) + 1)) - (uint8_t)(SAMPLE_COUNTER_MAX/20));
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
//        unsigned int range_rnd = 10000;
//        unsigned int random = GetRandomNumber(range_rnd);

        double quality;
        bool social = false;
        bool individual = false;

        // if robot made a discovery we have a discovery quality
        if(discovered){
            quality = discovered_quality;
        }else{  // robot did not sample enough yet
            quality = 0.0;
        }

        // Discovery and COMPARE: found a better option (in case of discovery robot is uncommitted
        // thus robot_commitment_quality should be 0
        // TODO does it makes sense to introduce this random variable?!??
#ifdef COMPARE_PROB
        unsigned int rand_number = GetRandomNumber(10000);
        unsigned int quality_int = (unsigned int)(10000 * quality)+1;
        if(quality > robot_commitment_quality + PARAM && rand_number <= quality_int){
#else
        if(quality > robot_commitment_quality + PARAM){
#endif
            individual = true;
        }
        // RECRUITMENT and DIRECT-SWITCH: message with different option
        if(new_robot_msg && robot_commitment != received_option && received_option != UNCOMMITTED){
            social = true;
        }

        // if both are true do a flip
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
#ifdef RECRUITBACK
            // reset last robot commitment
            last_robot_commitment = UNINITIALISED;
            last_robot_commitment_quality = 0.0;
#endif
            /// set last commitment switch - trigger for updating the communication range dynamically
            last_commitment_switch = kilo_ticks;
            commitment_switch_flag = true;
            // in case we want to try communication range based on how large the change is
//            step_size = (int)(20.0*(robot_commitment_quality/(last_robot_commitment_quality + robot_commitment_quality))); // this should be between 1 and 0.5

#ifdef CROSS_INHIBITION
        /// CROSS-INHIBITION MODEL
        }else if(social){
            if (robot_commitment == UNCOMMITTED){
                // set new commitment
                robot_commitment = received_option;
                // set new option the robot should sample
                op_to_sample = received_option;
#ifdef RECRUITBACK
                /// Depending on the storage set commitment quality
                if (last_robot_commitment == received_option){
                    robot_commitment_quality = last_robot_commitment_quality;
                    // reset last robot commitment
                    last_robot_commitment = UNINITIALISED;
                    last_robot_commitment_quality = 0.0;
                } else {  // no information -> start form 0
#endif
                    robot_commitment_quality = 0.0;
#ifdef RECRUITBACK
                }
#endif
            } else {  /// Robot is committed
#ifdef RECRUITBACK
                // remember last robot commitment if there is some information
                if (robot_commitment_quality != 0.0){
                    last_robot_commitment = robot_commitment;
                    last_robot_commitment_quality = robot_commitment_quality;
                }
#endif
                /// CROSS-INHIBITION - becomes uncommitted
                robot_commitment = UNCOMMITTED;
                robot_commitment_quality = 0.0;
                op_to_sample = current_ground;
            }
            /// reset sampling to make a new estimate on current commitment
            sample_op_counter = 0;
            sample_counter = 0;
            /// set last commitment switch - if we switch based on social information it is only second hand, thus we do not want to tell everybody
            commitment_switch_flag = false;
        }
#else
        /// DIRECT-SWITCHING MODEL
        }else if(social){
#ifdef RECRUITBACK
            if (last_robot_commitment == received_option){
                // set commitment
                robot_commitment = received_option;
                robot_commitment_quality = last_robot_commitment_quality;
                // reset last robot commitment
                last_robot_commitment = UNINITIALISED;
                last_robot_commitment_quality = 0.0;
            }else {
                if (robot_commitment_quality != 0.0) {
                    last_robot_commitment = robot_commitment;
                    last_robot_commitment_quality = robot_commitment_quality;
                }
#endif
                /// DIRECT-SWITCHING
                robot_commitment = received_option;
                robot_commitment_quality = 0.0;
                op_to_sample = received_option;
#ifdef RECRUITBACK
            }
#endif
            /// reset sampling to make a new estimate on current commitment
            sample_op_counter = 0;
            sample_counter = 0;
            /// set last commitment switch - if we switch based on social information it is only second hand, thus we do not want to tell everybody
            commitment_switch_flag = false;
        }
#endif
        // reset discovery and new message, so that they are not used again
        new_robot_msg = false;
        discovered = false;
    }
}


void update_communication_range(){
    // TODO clean up when we exactly know what we want to do
    // @giovanni this is very messy at the moment and is still under construction so does not need
    // to be checked
//    uint32_t tmp_communication_range;
//    uint32_t threshold_1 = COMMUNICATION_THRESHOLD_TIMER;
////    uint32_t threshold_2 = 2 * threshold_1;  // TODO maybe we need to do this dynamic as well
//
//    tmp_communication_range = communication_range;
//
//    /// different update rules
//    /// exponential increase
////    tmp_communication_range = (uint32_t) exp( (double)(kilo_ticks - last_commitment_switch) / 4925 );  // 2462
//    /// exponential decrease
////    tmp_communication_range = (uint32_t) 45 * exp( -((double)((kilo_ticks+1) - last_commitment_switch) / 2462.0));
//    /// linear increase
////    tmp_communication_range = (uint32_t) 45 * ( (double)(kilo_ticks - last_commitment_switch) / 18750);  //9375
//    /// linear decrease
////    tmp_communication_range = (uint32_t) 45 * (1 - (kilo_ticks - last_commitment_switch) / 9375);
//    /// on of basically step
////    if (kilo_ticks - last_commitment_switch < threshold_1) {
////        tmp_communication_range = 1;
////    }else if(kilo_ticks - last_commitment_switch < threshold_2 ){
////        tmp_communication_range = (int)(44.0/((double)(threshold_2-threshold_1)) * (double)kilo_ticks) - 43;
////    }else {
////        tmp_communication_range = 45;
////    }
//    /// adaptive by changing its opinion - step
////    if (kilo_ticks - last_commitment_switch < threshold_1 && commitment_switch_flag) {
////        tmp_communication_range = max_communication_range;
////    }else {
////        tmp_communication_range = MIN_COMMUNICATION_RANGE;
////    }
//
//    /// linear decrease
//    //tmp_communication_range = max_communication_range - ((kilo_ticks-last_commitment_switch)*max_communication_range/COMMUNICATION_THRESHOLD_TIMER);
////    if (kilo_ticks > 20000) {
////        tmp_communication_range = 2;
////    }else if (kilo_ticks > 10000) {
////        tmp_communication_range = 45;
////    }else{
////        tmp_communication_range = 2;
////    }
//
//    /// adaptive by changing its opinion - linear decrease
////    if (kilo_ticks - last_commitment_switch < threshold_1 && commitment_switch_flag) {
////        tmp_communication_range = max_communication_range;
////    }else if (kilo_ticks - last_commitment_switch < threshold_2 && commitment_switch_flag){
////        tmp_communication_range = max_communication_range - (int)((double)(max_communication_range - MIN_COMMUNICATION_RANGE)/(double)(threshold_2 - threshold_1)*((kilo_ticks - last_commitment_switch)-threshold_1));
////    }else {
////        tmp_communication_range = MIN_COMMUNICATION_RANGE;
////    }
//
//    // check for bounds
//    if (tmp_communication_range > 45) {
//        tmp_communication_range = 45;
//    } else if (tmp_communication_range < 1) {
//        tmp_communication_range = 1;
//    }
//
//    communication_range = tmp_communication_range;
}


/*-----------------------------------------------------------------------------------------------*/
/* Setting values of the message                                                                 */
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

        unsigned int p_share_commitment_int = (unsigned int)((2*robot_commitment_quality) * range_rnd) + 1;

        // broadcast message if the robot is committed - with probability equal to commitment quality
        if (robot_commitment != UNCOMMITTED && robot_commitment_quality > 0 && random <= p_share_commitment_int){
            set_message();
//            printf("[%d] sending option %d  \n", kilo_uid, robot_commitment);
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


/*-----------------------------------------------------------------------------------------------*/
/* Implements the movement of the robot.                                                         */
/*-----------------------------------------------------------------------------------------------*/
void move(){
    // reached goal - select new waypoint
    if((goal_gps_x == robot_gps_x && goal_gps_y == robot_gps_y) || last_waypoint_time >= kilo_ticks + MAX_WAYPOINT_TIME){
        last_waypoint_time = kilo_ticks;
        random_walk_waypoint_model();
    }

    // select current state
    if((robot_hit_wall || robot_hit_semi_wall) && !(current_state == AVOIDANCE_STRAIGHT || current_state == AVOIDANCE_TURN_LEFT || current_state == AVOIDANCE_TURN_RIGHT)){
        // escape from wall triggered - even if the robot hit the semi wall but it is still allowed
        //  to sample
        int32_t angletogoal = normalize_angle(
                atan2((CELLS_Y/2) - robot_gps_y, (CELLS_X/2) - robot_gps_x) / PI * 180 -
                robot_orientation);
        // right from target
        if (angletogoal <= 0) {
            current_state = AVOIDANCE_TURN_RIGHT;
        } else {  // if (angletogoal > 0) {
            current_state = AVOIDANCE_TURN_LEFT;
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
        if(state_counter != 0){
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
                current_state = MOVE_STRAIGHT;
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
    // check for obstacles - lock that only one obstacle is viable
    if (received_role == 21){
        robot_hit_semi_wall = true;
        robot_hit_wall = false;
    }else if(received_role == 42){
        robot_hit_wall = true;
        robot_hit_semi_wall = false;
    }else{
        robot_hit_wall = false;
        robot_hit_semi_wall = false;
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
        // else start uniform distributed over all options
        if (robot_commitment != 1){
            robot_commitment = (kilo_uid % (NUMBER_OF_OPTIONS)) + 1;
        }
        current_ground = msg->data[5];
        op_to_sample = current_ground;
        communication_range = msg->data[6];
        // TODO: changed for experiment for ants paper -> init
        // max_communication_range = msg->data[7];
         SAMPLE_COUNTER_MAX = msg->data[7];
         sample_counter_max_noise = SAMPLE_COUNTER_MAX + (GetRandomNumber(10000) % SAMPLE_COUNTER_MAX);  // to ensure that no robot makes random estimate with only one
        init_flag = true;
    }else if(msg->type == GRID_MSG && init_flag){
        received_x = msg->data[0];
        received_y = msg->data[1];
        received_ground = msg->data[2];
        received_role = msg->data[3];
        received_grid_msg_flag = true;
    }else if(msg->type == VIRTUAL_AGENT_MSG  && init_flag){
        // in the processing method we check that it is not our own msg
        received_option_msg = msg->data[2];
        received_kilo_uid = msg->data[3];
        received_virtual_agent_msg_flag = true;
//        printf("[%d] received msg from %d with option %d  \n", kilo_uid, received_kilo_uid, received_option_msg);
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Callback function for successful transmission                                                 */
/*-----------------------------------------------------------------------------------------------*/
void tx_message_success() {
    msg_counter_sent += 1;
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
    /// this is needed because in simulation we use the debug struct, thus we do not really send
    /// a message
    if(msg_counter_sent >= MSG_SEND_TRIES){
        debug_info_set(broadcast_flag, 0);
    } else {
        tx_message_success();
    }
#else
    /// sending a real message - thus tx_message_success gets called anyway
    // TODO check if this is true
    if (msg_counter_sent <= MSG_SEND_TRIES){
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
    }
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
    // Initialise random seed
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    // Initialise motors
    set_motors(0,0);

    received_kilo_uid = kilo_uid;

    // init some counters
    // TODO: is commented out due to experiments for ants paper with different sampling numbers
    //sample_counter_max_noise = SAMPLE_COUNTER_MAX + (GetRandomNumber(10000) % SAMPLE_COUNTER_MAX);
    update_ticks_noise = UPDATE_TICKS + (GetRandomNumber(10000) % UPDATE_TICKS);

    last_broadcast_ticks = GetRandomNumber(10000) % BROADCAST_TICKS + 1;

    // Initialise time to 0
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
//        if (robot_hit_semi_wall){
//            set_color(RGB(3,0,0));
//        } else if (robot_hit_wall){
//            set_color(RGB(0,3,0));
//        }else {
//            set_color(RGB(0,0,3));
//        }

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
    debug_info_set(quality, robot_commitment_quality);
//    if (sample_time_estimate_flag){
//        debug_info_set(sample, discovered_quality);
//        debug_info_set(sample_flag, true);
//        sample_time_estimate_flag = false;
//    }else{
//        debug_info_set(sample_flag, false);
//    }
//    debug_info_set(com_range, communication_range);

    // debug prints
//    if(kilo_uid == 0){
//        printf("[%d] %d \n", kilo_uid, SAMPLE_COUNTER_MAX);
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
    // initalize utils
    utils_init();
#endif
    // callback for received messages
    kilo_message_rx = message_rx;
    kilo_message_tx_success = tx_message_success;
    // start control loop
    kilo_start(setup, loop);
    return 0;
}