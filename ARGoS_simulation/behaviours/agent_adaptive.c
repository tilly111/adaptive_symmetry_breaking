// some notes: rotation speed = 38
// OHC - overhead controller
// a tick here is every ~31ms

// broad cast ticks

#include "kilolib.h"
//#define DEBUG
//#include "debug.h" // for real robots only
#include <stdio.h> // for ARGOS only
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "agentCDCIlocal.h"
#include <debug.h>

#define PI 3.14159265358979323846
#define UNCOMMITTED 0
#define AGENT_MSG 21
#define GRID_MSG 22
#define GLOBAL_MSG 23  // msg mimicing the global communication
#define VIRTUAL_LOCAL_MSG 24
#define INIT_MSG 10

#define size(x)  (sizeof(x) / sizeof((x)[0]))  // length of array bc c is fun

/*-----------------------------------------------------------------------------------------------*/
/* Timer values to select                                                                        */
/*-----------------------------------------------------------------------------------------------*/
#define NUMBER_OF_SAMPLES 25 // we sample each second
#define SAMPLE_NOISE 10  // discrete sample noise equal distribution NUMBER_OF_SAMPLES + 0~10

#define BROADCAST_SEC 0.1  // try to broadcast every x seconds


#define UPDATE_COMMITMENT_SEC 2.5  // updates commitment every 10 sec - was 1
#define UPDATE_NOISE 5*31 // first number is seconds of update rate


/*-----------------------------------------------------------------------------------------------*/
/* Enums section                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
// Enum for different motion types
typedef enum {
    STOP = 0,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
} motion_t;

// Enum for boolean flags
typedef enum {
    false = 0,
    true = 1,
} bool;

// Emum for new way point calculation
typedef enum {
    SELECT_NEW_WAY_POINT = 1,
    UPDATE_WAY_POINT = 0,
} waypoint_option;


/*-----------------------------------------------------------------------------------------------*/
/* Robot state                                                                                   */
/*-----------------------------------------------------------------------------------------------*/
// robot state variables
uint8_t Robot_GPS_X;  // current x position
uint8_t Robot_GPS_Y;  // current y position
uint32_t Robot_orientation;  // current orientation
uint8_t Robot_GPS_X_last;  // last x position  -> needed for calculating the orientation
uint8_t Robot_GPS_Y_last;  // last y position  -> needed for calculating the orientation
uint8_t current_ground = 100;  // current sensor reading
motion_t current_motion_type = STOP;  // current type of motion

// robot goal variables
uint8_t Goal_GPS_X;  // x pos of goal
uint8_t Goal_GPS_Y;  // y pos of goal

const uint8_t MIN_DIST = 4;  // min distance the new way point has to differ from the last one
const uint32_t MAX_WAYPOINT_TIME = 3600; // about 2 minutes -> after this time choose new way point
uint32_t lastWaypointTime;  // time when the last way point was chosen

// stuff for motion
const bool CALIBRATED = true;  // flag if robot is calibrated??
const uint8_t MAX_TURNING_TICKS = 37; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint32_t MAX_STRAIGHT_TICKS = 300;

uint32_t turning_ticks = 0;  // TODO chnage back to unsigned int?????
uint32_t last_motion_ticks = 0;
uint32_t straight_ticks = 0;

// Wall Avoidance manouvers
uint32_t wallAvoidanceCounter = 0; // to decide when the robot is stuck...
bool stuck = false;  // needed if the robot gets stuck


/*-----------------------------------------------------------------------------------------------*/
/* Variables for commitment of the robot                                                         */
/*-----------------------------------------------------------------------------------------------*/
// commitment of the robot
uint8_t my_commitment = 1;  // This is the initial commitment
double my_commitment_quality = 0.13;  // needed for the case of global communication that the robots evetually communicate before sampling a better option

const float PARAM = 0.01;  // parameter on how much the new option must be better than the old one
const uint32_t UPDATE_TICKS = (UPDATE_COMMITMENT_SEC*1000)/31;  // time between opinion updates
uint32_t update_ticks_noise;
uint32_t last_update_ticks = 0;


/*-----------------------------------------------------------------------------------------------*/
/* Communication variables for robot - robot                                                     */
/*-----------------------------------------------------------------------------------------------*/
// incomming msgs
bool received_msg_robot = false;
bool received_msg_robot_write = false;  // flag for updating the robot state
uint8_t received_commitment;  // received commitment - buffer for callback
uint8_t neigh_commitment;  // commitment of the neighbourning robot the robot received a msg from
uint8_t received_quality;  // quality the other robot measured and send
// outgoing msgs
bool broadcast_msg = false;  // Flag for decision to broadcast a message
const uint32_t BROADCAST_TICKS = (BROADCAST_SEC*1000)/31;  // time between broadcasts
uint32_t last_broadcast_ticks = 0;

message_t message;  // variable for outgoing msgs

uint8_t communication_range = 0;  // communication range in cells

/*-----------------------------------------------------------------------------------------------*/
/* Communication variables for robot - kilogrid                                                  */
/*-----------------------------------------------------------------------------------------------*/
bool received_msg_kilogrid = false;  // set to true if msg was received
bool hit_wall = false;  // set to true if wall detected
bool update_orientation = false;  // set if the orientation has been updated
uint8_t received_x = 0;  // saves received x pos of robot
uint8_t received_y = 0;  // saves received y pos of robot
uint8_t received_option = 0;  // saves received opinion of the tile the robot is on

bool init_write = false;
bool init_flag = true;
uint8_t received_quality = 0;
uint8_t received_number_of_options = 0;
uint8_t NUMBER_OF_OPTIONS = 0;


/*-----------------------------------------------------------------------------------------------*/
/* Sample variables                                                                              */
/*-----------------------------------------------------------------------------------------------*/
const uint32_t SAMPLE_TICKS = 32;  // sample every 1 sec once ?
uint32_t last_sample_ticks = 0;  // used to count

const uint32_t SAMPLE_COUNTER_MAX = NUMBER_OF_SAMPLES;  // how many samples we need to obtain
uint32_t sample_counter_max_noise;
const double sample_counter_std_dev = 1;
uint32_t sample_counter = 0;  // counts how often we sampled
uint32_t sample_op_counter = 0;  // counter on how often we encounter our op tp sample
uint8_t op_to_sample = 1;  // option we want to sample -> start with the crappy one ?

bool sampling_done = false;  // flag if sampling is finished

bool discovered = false;
uint8_t discovered_option = 0;
double discovered_quality = 0.0;


/*-----------------------------------------------------------------------------------------------*/
/* Arena variables                                                                               */
/*-----------------------------------------------------------------------------------------------*/
const uint8_t GPS_MAX_CELL_X = 20;
const uint8_t GPS_MAX_CELL_Y = 40;



/*-----------------------------------------------------------------------------------------------*/
/* Function to sample a random number form a Gaussian distribution.                              */
/*-----------------------------------------------------------------------------------------------*/
double generateGaussianNoise(double mu, double std_dev){
    const double epsilon = DBL_MIN;
    const double two_pi = 2.0*PI;
    double u1, u2;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );

    double z0;
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    return z0 * std_dev + mu;
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
/* Normalizes angle between -180 < angle < 180.                                                  */
/*-----------------------------------------------------------------------------------------------*/
void NormalizeAngle(double* angle){
    while(*angle>180){
        *angle=*angle-360;
    }
    while(*angle<-180){
        *angle=*angle+360;
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function for setting the motor speed.                                                         */
/*-----------------------------------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type) {
    // only update current motion if change
    if(current_motion_type != new_motion_type){
        current_motion_type = new_motion_type;

        switch( new_motion_type ){
            case FORWARD:
                spinup_motors();
                if (CALIBRATED) set_motors(kilo_straight_left,kilo_straight_right);
                else set_motors(67,67);
                break;
            case TURN_LEFT:
                spinup_motors();
                if (CALIBRATED){
                    uint8_t leftStrenght = kilo_turn_left;
                    uint8_t i;
                    for (i=3; i <= 18; i += 3){
                        if (wallAvoidanceCounter >= i){
                            leftStrenght+=2;
                        }
                    }
                    set_motors(leftStrenght,0);
                }else{
                    set_motors(70,0);
                }
                break;
            case TURN_RIGHT:
                spinup_motors();
                if (CALIBRATED){
                    uint8_t rightStrenght = kilo_turn_right;
                    uint8_t i;
                    for (i=3; i <= 18; i += 3){
                        if (wallAvoidanceCounter >= i){
                            rightStrenght+=2;
                        }
                    }
                    set_motors(0,rightStrenght);
                }
                else{
                    set_motors(0,70);
                }
                break;
            case STOP:
            default:
                set_motors(0,0);
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk with the random waypoint model.            */
/*-----------------------------------------------------------------------------------------------*/
void random_walk_waypoint_model(waypoint_option selectNewWaypoint){
    // if new waypoint flag, goal reached or timeout
    if(selectNewWaypoint || ((Robot_GPS_X==Goal_GPS_X) && (Robot_GPS_Y==Goal_GPS_Y)) || kilo_ticks >= lastWaypointTime + MAX_WAYPOINT_TIME){
        lastWaypointTime = kilo_ticks;
        do {
            // getting a random number in the range [1,GPS_maxcell-1] to avoid the border cells (upper bound is -2 because index is from 0)
            Goal_GPS_X = rand()%( GPS_MAX_CELL_X - 4 ) + 2;
            Goal_GPS_Y = rand()%( GPS_MAX_CELL_Y - 4 ) + 2;
            if(abs(Robot_GPS_X-Goal_GPS_X) >= MIN_DIST || abs(Robot_GPS_Y-Goal_GPS_Y) >= MIN_DIST){
                // if the selected cell is enough distant from the current location, it's good
                break;
            }
        } while(true);
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to check if the option has disappeared (to call at each GSP reading)                 */
/*-----------------------------------------------------------------------------------------------*/
void check_if_my_option_has_disappeared(){
    // TODO IMPLEMENT!!??
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
        if(hit_wall || sampling_done) return;
    
        // check if we reached our sampling time
        if(sample_counter < sample_counter_max_noise && !sampling_done){
            sample_counter++;
            if (current_ground == op_to_sample){
                sample_op_counter++;
            }
        }else{
            // sampling finished -> new sampling is set in update commitment either it is set
            // to sample the option it received from another robot or samples based on what the
            // robot measured last
            sampling_done = true;
            
            discovered_option = op_to_sample;
            discovered_quality = (float)sample_op_counter/(float)sample_counter;
            
            // set my quality to the measured quality iff its our commitment
            if(discovered_option == my_commitment){
                my_commitment_quality = (float)sample_op_counter/(float)sample_counter;
            } else{ //else flag that we discovered something
                discovered = true;
            }
            
            // reset sampling ?
            op_to_sample = current_ground;
            sample_counter = 0;
            sample_op_counter = 0;
            sampling_done = false;
            // for shuffling up we set the max sample counter
            sample_counter_max_noise = SAMPLE_COUNTER_MAX + (rand() % SAMPLE_NOISE);
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function for updating the communication range.                                                */
/*-----------------------------------------------------------------------------------------------*/
void update_communication_range() {
    // calculate the minutes the robot is running one tick is 31 ms -> 10 min
//    int sim_sec = (int)(kilo_ticks/32);
//    if (sim_sec > 600){
//        communication_range = 45;
//    }else{
//        communication_range = 2;
//    }
}



/*-----------------------------------------------------------------------------------------------*/
/* Function for updating the commitment state (wrt to the received message)                      */
/*-----------------------------------------------------------------------------------------------*/
void update_commitment() {
    if(kilo_ticks > last_update_ticks + update_ticks_noise){
        last_update_ticks = kilo_ticks;
        update_ticks_noise = UPDATE_TICKS + (rand() % UPDATE_NOISE);
        
        // drawing a random number
        unsigned int range_rnd = 10000;
        unsigned int random = GetRandomNumber(range_rnd);
        
        double quality;
        bool social = false;
        bool individual = false;
        
        // we can decide on the quality if the robot sampled enough
        if(discovered){
            quality = (float)sample_op_counter/(float)sample_counter;
        }else{  // robot did not sample enough yet
            quality = 0.0;  // if not sampled enough sampled quality is zero - basically a flag
        }
        unsigned int P_qualityInt = (unsigned int) (quality * range_rnd) + 1;
        
        // robot is uncommitted - it can do discovery or recruitment.
        if(my_commitment == UNCOMMITTED){
            // DISCOVERY: with prob quality switch new option
            if(quality > 0 && random <= P_qualityInt){
                individual = true;
            }
            
            // RECRUITMENT: in message we trust -> always true bc robot is uncommitted
            if(received_msg_robot && neigh_commitment != UNCOMMITTED){
                social = true;
            }
        }else{  // robot is committed
            // if current sampled option is better than current committed one switch
            // COMPARE
            if(quality > my_commitment_quality + PARAM && random <= P_qualityInt && my_commitment != discovered_option){
                individual = true;
            }
            
            // DIRECT-SWITCH
            // TODO global communication mimiced by allowing to switch to current commitment
            if(received_msg_robot && my_commitment != neigh_commitment){
                social = true;
            }
        }
        
        // if both true do a flip
        if(individual && social){
            if(rand() % 2) {
                individual = true;
                social = false;
            }else{
                individual = false;
                social = true;
            }
        }
        
        // do the switch
        if(individual){
            my_commitment = discovered_option;
            my_commitment_quality = discovered_quality;
            random_walk_waypoint_model(SELECT_NEW_WAY_POINT);
        }else if(social){
            my_commitment = neigh_commitment;
            my_commitment_quality = 0; // thus we first sample and then broadcast
            // reset sampling -> sample what you got told to
            op_to_sample = neigh_commitment;
            sample_op_counter = 0;
            sample_counter = 0;
            sampling_done = false;
        }
        received_msg_robot = false;
        discovered = false;
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to broadcast the commitment message                                                  */
/*-----------------------------------------------------------------------------------------------*/
void broadcast() {
    if(kilo_ticks > last_broadcast_ticks + BROADCAST_TICKS && !init_flag){
        last_broadcast_ticks = kilo_ticks;
        
        // share commitment with certain probability if the robot is committed
        unsigned int range_rnd = 10000;
        unsigned int random = GetRandomNumber(range_rnd);
        
        unsigned int P_ShareCommitementInt = (unsigned int)(my_commitment_quality * range_rnd) + 1;
        
        if (my_commitment != 0 && my_commitment_quality > 0 && random <= P_ShareCommitementInt){
            broadcast_msg = true;
            return;
        }
    }
    // if we do not broadcast reset the broadcast flag every cycle -> assuming we do one cycle per calculation
    debug_info_set(broadcast_flag, 0);
    broadcast_msg = false;
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to check if the robot is aganst the wall                                             */
/*-----------------------------------------------------------------------------------------------*/
void check_if_against_a_wall() {
    // handles target when we are at a wall?
    if(hit_wall){
        // drive towards the map center if coliding with a wall??
        if (wallAvoidanceCounter<18) wallAvoidanceCounter += 1;
        else wallAvoidanceCounter = 1;
    } else {
        if (wallAvoidanceCounter > 0){ // flag when the robot hit a wall -> after select new waypoint -> else ignore
            random_walk_waypoint_model(SELECT_NEW_WAY_POINT);
        }
        wallAvoidanceCounter = 0;
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Updates the robot state, needs to be called when new msg from kilogrid arrived                */
/*-----------------------------------------------------------------------------------------------*/
// todo basically we can move everything insdie the if condition bc shouldnt change
void update_robot_state(){
    // update reading of sensor - also check for hit_wall flag - setting it inside callback
    // function could lead to inconsistancy!!
    if (received_option == 42){  // robot sensed wall -> dont update the received option
        hit_wall = true;
    }else{
        current_ground = received_option;
        hit_wall = false;
    }
    
    // update current and last position
    if (received_x != Robot_GPS_X || received_y != Robot_GPS_Y){
        Robot_GPS_X_last = Robot_GPS_X;
        Robot_GPS_X = received_x;
        Robot_GPS_Y_last = Robot_GPS_Y;
        Robot_GPS_Y = received_y;
        update_orientation = true;
    }
    
    // calculate orientation of the robot based on the last and current visited cell -> rough estimate
    double angleOrientation = atan2(Robot_GPS_Y-Robot_GPS_Y_last, Robot_GPS_X-Robot_GPS_X_last)/PI*180;
    NormalizeAngle(&angleOrientation);
    Robot_orientation = (uint32_t) angleOrientation;
    
    received_msg_kilogrid = false;
}


/*-----------------------------------------------------------------------------------------------*/
/* Updates the robot state, needs to be called when new msg from other robot arrived             */
/*-----------------------------------------------------------------------------------------------*/
void update_received_msg(){
    // set neighbour commitement
    neigh_commitment = received_commitment;
    // set msg received flag
    received_msg_robot = true;
    // reset computation flag
    received_msg_robot_write = false;
}


/*-----------------------------------------------------------------------------------------------*/
/* Function to go to the Goal location, called every cycle,                                      */
/*-----------------------------------------------------------------------------------------------*/
void GoToGoalLocation() {
    // only recalculate movement if the robot has an update on its orientation aka. moved
    // if hit wall do the hit wall case... also stuck ensures that they drive straight forward for
    // a certain amount of time
    if (update_orientation && !hit_wall && !stuck){
        update_orientation = false;
        
        // calculates the difference thus we can see if we have to turn left or right
        double angletogoal = atan2(Goal_GPS_Y-Robot_GPS_Y,Goal_GPS_X-Robot_GPS_X)/PI*180-Robot_orientation;
        NormalizeAngle(&angletogoal);
        
        // see if we are on track
        bool right_direction = false; // flag set if we move towards the right celestial direction
        if(Robot_GPS_Y == Goal_GPS_Y && Robot_GPS_X < Goal_GPS_X){ // right case
            if(Robot_orientation == 0){ right_direction = true;}
        }else if (Robot_GPS_Y > Goal_GPS_Y && Robot_GPS_X < Goal_GPS_X){  // bottom right case
            if(Robot_orientation == -45){ right_direction = true;}
        }else if (Robot_GPS_Y > Goal_GPS_Y && Robot_GPS_X == Goal_GPS_X){  // bottom case
            if(Robot_orientation == -90){ right_direction = true;}
        }else if (Robot_GPS_Y > Goal_GPS_Y && Robot_GPS_X > Goal_GPS_X){  // bottom left case
            if(Robot_orientation == -135){ right_direction = true;}
        }else if (Robot_GPS_Y == Goal_GPS_Y && Robot_GPS_X > Goal_GPS_X){  // left case
            if(Robot_orientation == -180 || Robot_orientation == 180){ right_direction = true;}
        }else if (Robot_GPS_Y < Goal_GPS_Y && Robot_GPS_X > Goal_GPS_X){  // left upper case
            if(Robot_orientation == 135){ right_direction = true;}
        }else if (Robot_GPS_Y < Goal_GPS_Y && Robot_GPS_X == Goal_GPS_X){  // upper case
            if(Robot_orientation == 90){ right_direction = true;}
        }else if (Robot_GPS_Y < Goal_GPS_Y && Robot_GPS_X < Goal_GPS_X){  // right upper case
            if(Robot_orientation == 45){ right_direction = true;}
        }else{
            printf("[%d] ERROR - something wrong in drive cases \n", kilo_uid);
        }
        
        // if we are not in the right direction -> turn
        if (!right_direction){
            // right from target
            if ((int)angletogoal < 0 ) set_motion(TURN_RIGHT);
            // left from target
            else if ((int)angletogoal > 0) set_motion(TURN_LEFT);
            turning_ticks= MAX_TURNING_TICKS;
            last_motion_ticks = kilo_ticks;

        }else{
            set_motion(FORWARD);
        }
    }else if (hit_wall){
        // case that we hit a wall: first turn away, than drive straight for a while
        if (stuck){
            set_motion(FORWARD);
        }else if(!(current_motion_type == TURN_LEFT || current_motion_type == TURN_RIGHT)){
            double aTC =atan2(GPS_MAX_CELL_Y/2-Robot_GPS_Y,GPS_MAX_CELL_X/2-Robot_GPS_X)/PI*180-Robot_orientation;
            NormalizeAngle(&aTC);
            if (aTC > 0){
                set_motion(TURN_LEFT);
                last_motion_ticks = kilo_ticks;
                //turning_ticks = (unsigned int) (fabs(aTC)/38.0*32.0);  // /38.0*32.0
            }else{
                set_motion(TURN_RIGHT);
                last_motion_ticks = kilo_ticks;
                //turning_ticks = (unsigned int) (fabs(aTC)/38.0*32.0);  // /38.0*32.0
            }
            turning_ticks = rand() % 75 + 70; // should be max 180 deg turn aka 4.5 sec 2 bis 4 sec drehen?
            stuck = false;
        }
    }

    // needed at least for the beginning so that the robot starts moving into the right direction
    // but also to update after turning for a while -> move then straight to update orientation
    switch( current_motion_type ) {
        case TURN_LEFT:
        case TURN_RIGHT:
            if( kilo_ticks > last_motion_ticks + turning_ticks) {
                /* start moving forward */
                last_motion_ticks = kilo_ticks;  // fixed time FORWARD
                if (hit_wall){  // only enforce straight movement when trying to escape a wall
                    straight_ticks = rand() % MAX_STRAIGHT_TICKS + 150;
                    stuck = true;
                }
                set_motion(FORWARD);
            }
            break;
        case FORWARD:
            if(kilo_ticks > last_motion_ticks + straight_ticks){
                last_motion_ticks = kilo_ticks;
                stuck = false;
            }
            break;

        case STOP:
        default:
            set_motion(STOP);
    }
}

/*-----------------------------------------------------------------------------------------------*/
/* Init function                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
void setup(){
    // Initialise motors
    set_motors(0,0);
    
    // Initialise random seed
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    // Initialise motion variables
    last_motion_ticks = rand() % MAX_STRAIGHT_TICKS + 1;

    // Initialise broadcast variables
    last_broadcast_ticks = rand() % BROADCAST_TICKS + 1;

    // Initialise update variables
    last_update_ticks = rand() % UPDATE_TICKS;
    last_sample_ticks = rand() % SAMPLE_TICKS;

    // init robot state
    Robot_GPS_X_last = GPS_MAX_CELL_X/2;
    Robot_GPS_Y_last = GPS_MAX_CELL_Y/2;
    Robot_orientation = 0;
    // initialise the GSP to the middle of the environment, to avoid to trigger wall avoidance immediately
    Robot_GPS_X = GPS_MAX_CELL_X/2;
    Robot_GPS_Y = GPS_MAX_CELL_Y/2;
        
    // TODO random sample counter init that not all robots at once switch their op
    sample_counter = rand() % SAMPLE_COUNTER_MAX;
    
    // shuffle update and sample length
    sample_counter_max_noise = SAMPLE_COUNTER_MAX + (rand() % SAMPLE_NOISE);
    update_ticks_noise = UPDATE_TICKS + (rand() % UPDATE_NOISE);
    
    // Intialize time to 0
    kilo_ticks = 0;
}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
    // decode msg
    uint8_t data0 = msg->data[0];
    uint8_t data1 = msg->data[1];
    uint8_t data2 = msg->data[2];
    
    if(msg->type == INIT_MSG){
        init_write = true;
        received_commitment = data0;
        received_quality = data1;
        received_number_of_options = data2;
        received_option = msg->data[3];
        communication_range = msg->data[4];
    }else if(msg->type == GRID_MSG && !init_flag){
        received_msg_kilogrid = true;  // flag that we received msg from kilogrid
        received_option = data0;  // get falg and option
        received_x = data1;  // get x position of the robot
        received_y = data2;  // get y position of the robot
    }else if(msg->type == VIRTUAL_LOCAL_MSG && !init_flag){  // msg from other robot via kilogrid
        received_msg_robot_write = true;  // flag for update the robot state
        received_commitment = data0;  // data 1 should store the option the other robot broadcasts
    }else{
        printf("[%d] ERROR - unknown message type: %d \n", kilo_uid, msg->type);
    }
}

/*-----------------------------------------------------------------------------------------------*/
/* Callback function for message transmission - this methods do not need to be implemented for   */
/* the simulation bc inter robot communication is handled by the kilogrid                        */
/*-----------------------------------------------------------------------------------------------*/
message_t *message_tx() {
    if (broadcast_msg){
        // here we have to insert a hack -> we communicate via the kilogrid which basically means we
        // set our debug state
            // ATTENTION: we do not need to set our commitment here, because it is always set at the
            // end of the control loop (for data collecting reasons - but now also for logic)
            // debug_info_set(commitement, my_commitment);
            
            // we now just need to set our communication range and the flag that we indeed want to
            // broadcast
            //debug_info_set(broadcast_flag, 1);
            //debug_info_set(com_range, communication_range);
        debug_info_set(broadcast_flag, 1);
        debug_info_set(com_range, communication_range);
        broadcast_msg = false;
    }
    return 0;
}

/*-----------------------------------------------------------------------------------------------*/
/* Callback function for successful transmission  - this methods do not need to be implemented   */
/* for the simulation bc inter robot communication is handled by the kilogrid                    */
/*-----------------------------------------------------------------------------------------------*/
void tx_message_success() {
    //broadcast_msg = false;
    //printf("[%d] resetting ... \n");
    return;
}

/*-----------------------------------------------------------------------------------------------*/
/* Main loop                                                                                     */
/*-----------------------------------------------------------------------------------------------*/
void loop() {
    if (init_flag){  // initialize robot state + other stuff
        if (init_write){  // received the msg
            NUMBER_OF_OPTIONS = received_number_of_options;
            op_to_sample = rand() % NUMBER_OF_OPTIONS + 1;
            my_commitment = received_commitment;
            my_commitment_quality = (float)(received_quality/255.0); //+ generateGaussianNoise(0, sample_counter_std_dev);
            current_ground = received_option;
            // reset init flags
            init_write = false;
            init_flag = false;
            
            random_walk_waypoint_model(SELECT_NEW_WAY_POINT);
            set_motion( FORWARD );
        }
    }else{  // normal control loop case
        // update robot state and select waypoint if update msg from kilogrid
        if (received_msg_kilogrid) {
            update_robot_state();  // updates internal state variables and sensor readings
            check_if_against_a_wall();  // checks if the robot is on wall
            random_walk_waypoint_model(UPDATE_WAY_POINT); // update the waypoints
        }
    
        // received msg from other robot
        if(received_msg_robot_write){
            update_received_msg();
        }
    
        // sample - every cycle
        sample();
        
        // move towards random location
        GoToGoalLocation();
        
        // update commitment every 15 sec ?
        update_commitment();
        
        // update communication range
        update_communication_range();
        
        // try to broadcast
        broadcast();
    
        // visulizes the commitment of the robot
        if (hit_wall || stuck){
            if (stuck){
                set_color(RGB(3,0,0));
            }else{
                set_color(RGB(0,0,0));
            }
        }else{
            switch(my_commitment) {
            case 5:
                set_color(RGB(0,3,3));
                break;
            case 4:
                set_color(RGB(3,1,0));
                break;
            case 3:
                set_color(RGB(1,0,3));
                break;
            case 2:
                set_color(RGB(0,3,0));
                break;
            case 1:
                set_color(RGB(3,0,1));
                break;
            case 0:
                set_color(RGB(0,0,0));
                break;
            case 42:
                set_color(RGB(3,3,3));
                break;
            default:
                printf("[%d] ERROR - wrong state %d \n", kilo_uid, my_commitment);
                set_color(RGB(3,3,3));
                break;
            }
        }
    }
    // debug info - is now also important for the inter robot communication, so do not delete
    debug_info_set(commitement, my_commitment);
    
    // debug prints
//    if(kilo_uid == 1){
//        printf("[%d] Robot at %d %d sampling Option %d with progress %d / %d on ground %d - current option %d with qualtiy %f \n", kilo_uid, Robot_GPS_X, Robot_GPS_Y, op_to_sample, sample_op_counter, sample_counter, current_ground, my_commitment, my_commitment_quality);
//    }
    
}

/*-----------------------------------------------------------------------------------------------*/
/* Main function                                                                                 */
/*-----------------------------------------------------------------------------------------------*/
int main(){
    kilo_init();  // init hardware of the kilobot
    kilo_message_tx = message_tx;  // register callback - pointer to message which should be send - roughly every 0.5 s
    kilo_message_tx_success = tx_message_success;  // triggered when transmission is successfull
    kilo_message_rx = message_rx;  // callback for received messages

    debug_info_create();  // does the inital setup do drag information out of the simulation -> enables to use the debug struct.

    kilo_start(setup, loop);
    return 0;
}
