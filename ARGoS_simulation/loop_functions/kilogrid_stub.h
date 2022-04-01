//
// Created by Till Konrad Aust on 17.12.21.
//
#ifndef KILOGRID_H
#define KILOGRID_H

namespace argos {
    class CSpace;
    class CRay3;
    class CFloorEntity;
    class CSimulator;
}

#include <math.h>
#include <random>

#include <behaviours/agent.h>

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/logging/argos_log.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/plugins/simulator/entities/box_entity.h>

#include <argos3/plugins/robots/kilobot/simulator/kilobot_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_entity.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_medium.h>
#include <argos3/plugins/robots/kilobot/simulator/kilobot_communication_default_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>

//kilobot messaging
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/kilobot/control_interface/message_crc.h>
#include <argos3/plugins/robots/kilobot/control_interface/message.h>

#include <array>



using namespace argos;

// for mimicing addressing cells
typedef enum {
    CELL_00 = 0,
    CELL_01 = 1,
    CELL_02 = 2,
    CELL_03 = 3
}cell_num_t;
cell_num_t cell_id[4] = {CELL_00, CELL_01, CELL_02, CELL_03};

typedef enum {
    WHITE   = 0,
    RED     = 1,
    GREEN   = 2,
    BLUE    = 3,
    YELLOW  = 4,
    CYAN    = 5,
    MAGENTA = 6,
    LED_OFF = 7
} color_t;

typedef enum {
    HIGH     = 1
} brightness_t;


typedef struct {
    uint8_t data[9]; ///< message payload
    uint8_t type;    ///< message type
    uint16_t crc;    ///< message crc
} IR_message_t;

typedef struct{
    uint16_t id;

    struct {
        int8_t rtr : 1;
        uint8_t length : 4;
    } header;

    uint8_t data[8];

} CAN_message_t;

typedef enum{
    CAN_MODULE_USER = 0x00,
    CAN_FORWARD_IR_MESSAGE,
    CAN_FORWARD_IR_MESSAGE_STOP,
    CAN_KILO_BOOTPAGE,
    CAN_KILO_BOOTPAGE_SIZE,
    CAN_KILO_BOOTPAGE_NUMBER,
    CAN_MODULE_BOOT,
    CAN_MODULE_BOOTPAGE,
    CAN_MODULE_BOOTPAGE_SIZE,
    CAN_MODULE_BOOTPAGE_NUMBER,
    CAN_MODULE_KILOBOT_SETUP,
    CAN_MODULE_IDLE,
    CAN_MODULE_SETUP,
    CAN_MODULE_RUN,
    CAN_MODULE_PAUSE,                 ///< IGNORED!
    CAN_MODULE_CONFIG_SIZE,           ///< Number of 6 byte configuration data segments
    CAN_MODULE_CONFIG_TRANSFER,       ///< Configuration data transmission
    CAN_TRACKING_KILOBOT,
    CAN_KILOBOT_DATA,
    CAN_MODULE_DATA,
    CAN_TRACKING_KILOBOT_START,
    CAN_TRACKING_REQ,
    CAN_USER_MESSAGE  // used for broadcasting inter module communication
} CAN_message_type_t;

typedef enum __attribute__ ((__packed__)) {
    ADDR_BROADCAST = 0x00, ///< broadcast to all modules
    ADDR_ROW, ///< broadcast to all modules in a row
    ADDR_COLUMN, ///< broadcast to all modules in a column
    ADDR_INDIVIDUAL, ///< send to an individual module
    ADDR_BROADCAST_TO_MODULE,
    ADDR_DISPATCHER = 0xFF ///< send to dispatcher
} kilogrid_addressing_type_t;

typedef struct __attribute__((__packed__)){

    uint8_t x; ///< columns
    uint8_t y; ///< rows

    kilogrid_addressing_type_t type; ///< type of address in Kilogrid

} kilogrid_address_t;

class CKilogrid : public CLoopFunctions
{

public:
    // Basic argos Loopfunctions
    CKilogrid();
    virtual ~CKilogrid();
    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual void PreStep();
    virtual void PostStep();
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

    // utility functions for the simulation
    // Get a Vector of all the Kilobots in the space
    void get_kilobots_entities();

    CVector2 GetKilobotPosition(CKilobotEntity& c_kilobot_entity);
    UInt16 GetKilobotId(CKilobotEntity& c_kilobot_entity);

    // collect data and set data
    void virtual_message_reception();
    // Get debug infromation of all kilobots
    void GetDebugInfo();
    // setup environemnt
//    void setup_virtual_environments(TConfigurationNode& t_node);


    // this parser mimics the real kilogrid
    void read_configuration(TConfigurationNode& t_tree);

    // for changing the environment
    void change_environment();

    // this is the setup method - which is simular to the setup method on the real kilogrid
    // atleast from the logic point - it is called before each experiment
    void setup(int x, int y);

    // runs every cycle
    void loop(int x, int y);

    // receive message
    void IR_rx(int x, int y, IR_message_t *m, cell_num_t c, distance_measurement_t *d, uint8_t CRC_error);


    // this method draws stuff on the kilogrid
    void set_LED_with_brightness(int x, int y, cell_num_t cn, color_t color, brightness_t brightness);

    void set_IR_message(int x, int y, IR_message_t &m, cell_num_t cn);

    // this methods are used for sending messages between cells of the kilogrid
    void init_CAN_message(CAN_message_t* cell_msg);
    uint8_t CAN_message_tx(CAN_message_t *, kilogrid_address_t);
    void CAN_rx(int x, int y, CAN_message_t *m);

        // returns the corresponding option to the grid
    UInt16 position2option(CVector2 t_position);
    UInt16 grid_cell2option(CVector2 t_position);
    CVector2 position2cell(CVector2 t_position);
    CVector2 module2cell(CVector2 module_pos, cell_num_t cn);
    CVector2 position2module(CVector2 t_position);

private:
    CRandom::CRNG* m_pcRNG;
    // for reading the config file
    std::ifstream input;
    std::string config_file_name;
    std::string second_config_file_name;
    // this struct imitates the message structure of the messaging in the real kilogrid
    // TODO this may need adjustment
    distance_measurement_t *d;

    // debug stuff
    int sample[101][2];

    // init stuff
    uint8_t initial_commitment = 1;
    uint8_t initial_commitment_quality = 70;
    uint8_t number_of_options = 3;
    uint8_t initial_communication_range;
    uint8_t max_communication_range;
    // this data structure is used to save the init data
    std::vector<uint8_t> configuration[10][20];

    // struct of memory of one module
    struct module_mem{
        // tmp variables
        uint8_t received_option;

        // counters
        uint32_t msg_number_current;
        uint32_t msg_number;
        uint32_t status_msg_counter;

        // cell variables
        uint8_t cell_x[4] = {0, 0, 0, 0};
        uint8_t cell_y[4] = {0, 0, 0, 0};
        uint8_t cell_role[4] = {0, 0, 0, 0};
        color_t cell_colour[4] = {WHITE, WHITE, WHITE, WHITE};

        uint8_t cell_op[4] = {0, 0, 0, 0};

        // ir message received from robots
        IR_message_t *robot_message = nullptr;
        std::vector<IR_message_t*> robot_messages;
        // ir message to send
        IR_message_t *ir_message_tx = new IR_message_t;

        // can message used for inter cell communication
        CAN_message_t cell_message;
        kilogrid_address_t cell_address;  // TODO init as null?

        std::vector<CAN_message_t> received_cell_messages;

        // tmp mem for saving the data from can message callback
        uint8_t received_cell_op[4] = {0, 0, 0, 0};
        uint32_t reset_timer[4] = {0, 0, 0, 0};

        uint8_t received_IR_msg_cell = 10;

        // communication flags
        bool init_flag = false;
        bool received_can = false;
        bool received_ir = false;
        bool can_msg_to_send = false;
        bool ir_msg_to_send[4] = {false, false, false, false};


        // variables for processing the broadcast of a robot
        uint8_t com_range;
        uint8_t option;
        uint8_t my_x;
        uint8_t my_y;
        uint8_t kilo_uid;
        uint8_t can_kilo_uid;

        // tmp data to store messages from the callbacks
        uint8_t tmp_can_data[8];
        uint8_t tmp_ir_data[4][8];

        // to process received data
        uint8_t received_commitment[4];  // received commitment from ir message
        uint8_t received_x[4];
        uint8_t received_y[4];
        uint8_t received_communication_range[4];
        uint8_t received_kilo_uid[4];

        // for sending data
        uint8_t opt_to_send_ir[4];
        uint8_t received_can_kilo_uid[4];


        uint8_t com_range_module;
        uint8_t send_success_sum;
        uint8_t my_x_module;
        uint8_t my_y_module;

        uint8_t send_flag;


    };

    // this array mimics the storage of each module
    module_mem module_memory[10][20];

    // for communication - kilobots
    typedef std::vector<CKilobotEntity*> kilobot_entities_vector;
    kilobot_entities_vector kilobot_entities;
    // the position is in cells
    std::vector<CVector2> robot_positions;

    // DEBUGGING INFORMATION
    //
    // This is an efficient way to store both controllers and debug information.
    std::vector<debug_info_t*> debug_info_kilobots;
    // for logging
    std::ofstream output_logg;
    std::string data_file_name;
    std::vector<unsigned int> logg_commitment_state;
    uint32_t data_saving_counter = 0;
    const uint32_t DATA_SAVING_FREQUENCY = 1;
};

#endif