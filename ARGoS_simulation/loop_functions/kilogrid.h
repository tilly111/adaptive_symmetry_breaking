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

#include <behaviours/agentCDCIlocal.h>

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
    // Used to plot the Virtual environment on the floor
    virtual CColor GetFloorColor(const CVector2& vec_position_on_plane);

    // init related messages
    // Get a Vector of all the Kilobots in the space
    void GetKilobotsEntities();

    // Get debug infromation of all kilobots
    void GetDebugInfo();

    // Setup the initial state of the Kilobots in the space
    void SetupInitialKilobotsStates();

    // Setup the initial state of the kilobot c_kilobot_entity
    void SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity);

    // Experiment configuration methods (From .argos files)
    
    // Setup virtual environment
    void SetupVirtualEnvironments(TConfigurationNode& t_tree);

    // Get experiment variables
    void GetExperimentVariables(TConfigurationNode& t_tree);

    // Virtual environment visualization updating
    // Plot the environment
    void PlotEnvironment();


    // Kilobots' states updating methods
    // Get the messages to send to the Kilobots according their positions
    void UpdateKilobotsState();

    // Get the message to send to a Kilobot according to its position
    void UpdateKilobotState(CKilobotEntity& c_kilobot_entity);

    // Virtual Sensors updating methods
    // Get the messages to send to the Kilobots according their positions
    void UpdateVirtualSensors();

    // Get the message to send to a Kilobot according to its position
    void UpdateVirtualSensor(CKilobotEntity& c_kilobot_entity);

    // Virtual Envirenment Updating methods
    // Update the virtual environment if dynamics
    void UpdateVirtualEnvironments();

    // Update the virtual environment if dynamics
    void UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity& c_kilobot_entity);

    // Kilobot Tracking methods
    // Get the position of a Kilobot
    CVector2 GetKilobotPosition(CKilobotEntity& c_kilobot_entity);

    // Get the orientation of a Kilobot
    CRadians GetKilobotOrientation(CKilobotEntity& c_kilobot_entity);

    // Get the kilobot ID
    UInt16 GetKilobotId(CKilobotEntity& c_kilobot_entity);

    // Get the kilobot LED color
    CColor GetKilobotLedColor(CKilobotEntity& c_kilobot_entity);

    // Add one option
    void AddOption(TConfigurationNode& t_node);

    // Convert a position in the arena to a GPS coordinates (grid)
    CVector2 PositionToGPS( CVector2 t_position );
    
    // returns the corresponding option to the grid
    UInt16 PositionToOption(CVector2 t_position);
    UInt16 GridCellToOption(CVector2 t_position);
    
    // some functions for the adaptive case - to mimic the real kilogrid
    // this method mimics the reception of the messages from the robots (to the kilogrid)
    void receiveKilobotsMessages();
    // its implementation
    void receiveKilobotMessage(CKilobotEntity& c_kilobot_entity);
    void receiveKilobotMessage(int x, int y);
    
    // this method implements the messages send to the robots from the kilogrid
    void sendKilobotsMessages();
    // its implementation
    void sendKilobotMessage(int x, int y);
    
    
private:
    // basic attribures of the loop function
    // random number stuff
    CRandom::CRNG* m_pcRNG;
    // List of the Kilobots in the space
    typedef std::vector<CKilobotEntity*> TKilobotEntitiesVector;
    TKilobotEntitiesVector m_tKilobotsEntities;

    // List of the messages sent by communication entities
    typedef std::vector<message_t> TKilobotsMessagesVector;
    TKilobotsMessagesVector m_tMessages;
   
    // virtual environment variables
    // virtual environment struct
    struct m_sOption
    {
        UInt16 id;
        UInt16 initRobotPopulation;
        Real quality;
        CVector2 position;
        CVector2 GPS_position;
        Real radius;
        CColor color;
        Real AppearanceTime=0;
        Real DisappearanceTime=0;
        Real QualityChangeTime=0;
        Real qualityAfterChange;
    };

    // options vector
    typedef std::vector<m_sOption> TOptionsVector;
    TOptionsVector m_tOptions;
    
    // grid for storing the options map
    int grid[20][40];
    // flag for shuffle the grid cells
    bool shuffle;
    // init flag for sending initial commitment and quality
    bool init_flag;
    
    // init communication range of one robot
    int INIITALCOMRANGE;
    
    // message time gps msg
    std::vector<Real> tLastTimeMessaged;
    // message time for global broadcasting 
    std::vector<Real> tLastTimeMessagedGLOBAL;
    // message time for adaptive case for grid cells
    Real tLastTimeMessagedGridCell[20][40];
    
    // Time for one kilobot message to be sent
    Real MinTimeBetweenTwoMsg;

    // Experiment variables
    // Experiment time in seconds
    Real m_fTimeInSeconds;

    // Counters for messages and data acquizition
    UInt16 m_unMessagingCounter,m_unDataSavingCounter;

    // Is the virtual environment dynamics? If yes update its plotting
    bool m_bDynamicVirtualEnvironments;

    // output file for data acquizition
    std::ofstream m_cOutput;

    // output file name
    std::string m_strOutputFileName;

    // data acquisition frequency in ticks
    UInt16 m_unDataFrequency;

    // quorum
    Real m_fQuorum;

    // quorum in robots number
    Real m_fQuorumRobots;

    // A flag for a kilbot needs a message to be sent or not
    bool m_bQuorumReached;

    // Number of GPS cells
    UInt16 m_unGpsCells;
    UInt16 m_unGpsCellsX;
    UInt16 m_unGpsCellsY;

    // GPS cell length in meters
    Real m_fCellLength;

    // Vector containing the commitement state of a robot
    std::vector<unsigned int> m_tCommitmentState;
    // Vector containing the commitement state of a robot with respect to its ability to broadcast
    std::vector<unsigned int> broadcastCommitmentState;
    
    // Vector which keeps track of the position of each robot
    std::vector<CVector2> robot_positions;
    
    // grid of lists which contain the messages which need to be send to every grid cell respectively
    std::vector<unsigned int> message_grid[20][40];
    

    // DEBUGGING INFORMATION
    //
    // This is an efficient way to store both controllers and debug information.
    std::vector<debug_info_t*> m_tKBs;


    // Vector to store the times where the simulator need to update the virtual environment (to not do checks every time-step as it is expensive)
    std::vector<Real> interestingTimes;

};
#endif
