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

    // this parser mimics the real kilogrid
    void read_configuration(TConfigurationNode& t_tree);

    // this is the setup method - which is simular to the setup method on the real kilogrid
    // atleast from the logic point - it is called before each experiment
    void setup(int x, int y);

    void loop(int x, int y);

    // this method draws stuff on the kilogrid
    void set_LED_with_brightness(int x, int y, cell_num_t cn, color_t color, brightness_t brightness);

    // returns the corresponding option to the grid
    UInt16 position2option(CVector2 t_position);
    UInt16 grid_cell2option(CVector2 t_position);
    uint8_t position2cell(CVector2 t_position);

private:
    // for reading the config file
    std::ifstream input;
    std::string config_file_name;
    // this struct imitates the message structure of the messaging in the real kilogrid
    // TODO this may need adjustment
    struct {
        uint8_t type;
        uint8_t data[8];
    } IR_message_t;

    // this data structure is used to save the init data
    std::vector<uint8_t> configuration[10][20];



    // struct of memory of one module
    struct module_mem{
        // tmp variables
        uint8_t received_option;
        uint8_t received_com_range;
        uint8_t received_x;
        uint8_t received_y;

        // counters
        uint32_t msg_number_current;
        uint32_t msg_number;
        uint32_t test_counter;

        // cell variables
        uint8_t cell_x[4] = {0, 0, 0, 0};
        uint8_t cell_y[4] = {0, 0, 0, 0};
        uint8_t cell_role[4] = {0, 0, 0, 0};
        uint8_t cell_colour[4] = {0, 0, 0, 0};
    };

    // this array mimics the storage of each module
    module_mem module_memory[10][20];
};

#endif