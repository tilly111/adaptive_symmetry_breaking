#include "ark.h"

// some notes for the grid
// border is 2 tiles thic thus -> 20*40 -> 16*36 = 576

/*-----------------------------------------------------------------------------------------------*/
/* Initialization of the Loopfunctions.                                                          */
/*-----------------------------------------------------------------------------------------------*/
CArk::CArk():
CLoopFunctions(),  // ??
shuffle(true),  // if the tiles should be shuffled -> should always be true
global(false),  // TODO for mimicing global communication
m_fTimeInSeconds(0),  // current simulation time in sec
m_unDataSavingCounter(1),
m_bDynamicVirtualEnvironments(false),  // if the system is dynamic
m_unDataFrequency(100),
m_fQuorum(1.0),  // quorum of consensus
m_bQuorumReached(false),  // flag for quorum reached
initial_opinion(20){}  // initial opinion

CArk::~CArk(){}


/*-----------------------------------------------------------------------------------------------*/
/* Init method runs before every experiment starts.                                              */
/*-----------------------------------------------------------------------------------------------*/
void CArk::Init(TConfigurationNode& t_node) {
    
    // Create random number generator
    m_pcRNG = CRandom::CreateRNG("argos");
    
    // Get experiment variables from the .argos file
    GetExperimentVariables(t_node);
    
    // Get the virtual environment from the .argos file & set up the kilogrid
    SetupVirtualEnvironments(t_node);
    
    // Get the initial kilobots' states
    SetupInitialKilobotsStates();
    
    // Get kilobots' debug info
    GetDebugInfo();
    
    // Open a log file
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
    
    // write the log file header (it is not mendatory)
    m_cOutput << "time;";
    for(unsigned int i=0;i<m_tOptions.size();i++){
        m_cOutput << i << ";";
    }
    
    if(m_tOptions.size()>0){
        m_cOutput << m_tOptions.size()<< std::endl;;
    }
    
    // Intializing variables
    m_fQuorumRobots=m_fQuorum*m_tKilobotsEntities.size();
    m_tCommitmentState.resize(m_tOptions.size()+1);
    // TODO make cleaner dirty init
    for(int till = 0; till < 4; till++){
        if(till == 1){
            m_tCommitmentState[till] = 50;
        }else{
            m_tCommitmentState[till] = 0;
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment is resetted.                                                  */
/*-----------------------------------------------------------------------------------------------*/
void CArk::Reset() {
    // Close data file
    m_cOutput.close();
    
    // Reopen the file, erasing its contents
    m_cOutput.open(m_strOutputFileName, std::ios_base::trunc | std::ios_base::out);
    // write the log file header (it is not mendatory)
    m_cOutput << "time;";
    for(unsigned int i=0;i<m_tOptions.size();i++){
        m_cOutput << i << ";";
    }
    if(m_tOptions.size()>0){
        m_cOutput << m_tOptions.size()<< std::endl;;
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment ended.                                                        */
/*-----------------------------------------------------------------------------------------------*/
void CArk::Destroy() {
    // Close data file
    m_cOutput.close();
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called before every simulation step.                                                     */
/*-----------------------------------------------------------------------------------------------*/
void CArk::PreStep(){
    // Update the time variable required for the experiment (in sec)
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();
    
    // Update the virtual sensor of the kilobots
    UpdateVirtualSensors();
    
    // Update the virtual environment - not needed at the moment
    //UpdateVirtualEnvironments();
    
    // Update the virtual environment plot
    PlotEnvironment();
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called after every simulation step.                                                      */
/*-----------------------------------------------------------------------------------------------*/
void CArk::PostStep(){
    // Save experiment data to the specified log file
    
    // check if quorum is reached
    std::fill(m_tCommitmentState.begin(), m_tCommitmentState.end(), 0);
    for(unsigned int i=0;i< m_tKilobotsEntities.size();i++){
        m_tCommitmentState[((unsigned int) m_tKBs[i]->commitement)]++;
        // not initial state (crappy option) and uncommitted and more than 0 robots and more than
        // robots than needed for quorum
        if((m_tKBs[i]->commitement!=initial_opinion) && (m_tKBs[i]->commitement!=0)
           && (m_fQuorumRobots>0)
           && (m_tCommitmentState[((unsigned int) m_tKBs[i]->commitement)]>=m_fQuorumRobots) ){
            m_bQuorumReached=true;
        }
    }
    
    // if quroum reached, time to write something down, max time passed
    if(  m_bQuorumReached
       || ((m_unDataSavingCounter%m_unDataFrequency == 0) && (m_unDataSavingCounter!=0))
       || ((GetSpace().GetSimulationClock() >= GetSimulator().GetMaxSimulationClock())
           && (GetSimulator().GetMaxSimulationClock()>0))){
        m_cOutput << GetSpace().GetSimulationClock();
        for(unsigned int i=0;i< m_tCommitmentState.size();i++){
            m_cOutput << ";" << m_tCommitmentState[i];
        }
        m_cOutput<< std::endl;
    }
    
    // for viz -> that i can see the progression
    if(GetSpace().GetSimulationClock() % 1000 == 0){
        printf("Clock at %d ... \n", GetSpace().GetSimulationClock());
    }
    
    // quit simulation if quorum reached
    if(m_bQuorumReached==true){
        printf("reached quorum");
        GetSimulator().Terminate();
    }
    
    m_unDataSavingCounter++;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the position of the kilobot.                                                          */
/*-----------------------------------------------------------------------------------------------*/
CVector2 CArk::GetKilobotPosition(CKilobotEntity& c_kilobot_entity){
    CVector2 vecKP(c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                   c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    return vecKP;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the orientation of the kilobot.                                                       */
/*-----------------------------------------------------------------------------------------------*/
CRadians CArk::GetKilobotOrientation(CKilobotEntity& c_kilobot_entity){
    CRadians cZAngle;
    CRadians cYAngle;
    CRadians cXAngle;
    
    //Calculate the orientations of the kilobot
    CQuaternion cRobotOrientations = c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Orientation;
    
    cRobotOrientations.ToEulerAngles(cZAngle,cYAngle, cXAngle);
    
    return cZAngle;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the id of the kilobot.                                                                */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CArk::GetKilobotId(CKilobotEntity& c_kilobot_entity){
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the LED Color of the kilobot.                                                         */
/*-----------------------------------------------------------------------------------------------*/
CColor CArk::GetKilobotLedColor(CKilobotEntity &c_kilobot_entity){
    return c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();
}


/*-----------------------------------------------------------------------------------------------*/
/* Creates a list of kilobot robot entities.                                                     */
/*-----------------------------------------------------------------------------------------------*/
void CArk::GetKilobotsEntities(){
    // Go through all the robots in the environment and create a
    // vector of pointers on their entities
    
    // Get the map of all kilobots from the space
    CSpace::TMapPerType& mapKilobots=GetSpace().GetEntitiesByType("kilobot");
    // Go through them
    for(CSpace::TMapPerType::iterator it = mapKilobots.begin();it != mapKilobots.end();++it) {
        m_tKilobotsEntities.push_back(any_cast<CKilobotEntity*>(it->second));
    }
    
}


/*-----------------------------------------------------------------------------------------------*/
/* Set up the getting of debug information from the robot (e.g. the robots state).               */
/*-----------------------------------------------------------------------------------------------*/
void CArk::GetDebugInfo(){
    m_tKBs.clear();
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        // Check if there is a message to send to the kilobot "i" and get the message
        
        /*
         * When the 'reset' method is called on the kilobot controller, the
         * kilobot state is destroyed and recreated. Thus, we need to
         * recreate the list of controllers and debugging info from scratch
         * as well.
         */
        // Create a pointer to the current kilobot
        CCI_KilobotController* pcKBC = &dynamic_cast<CCI_KilobotController&>(m_tKilobotsEntities[it]->GetControllableEntity().GetController());
        // Create debug info for controller
        debug_info_t* ptDebugInfo = pcKBC->DebugInfoCreate<debug_info_t>();
        // Append to list
        m_tKBs.push_back(ptDebugInfo);
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Setting up the kilobot states and needed varibales for handling the communication.            */
/*-----------------------------------------------------------------------------------------------*/
void CArk::SetupInitialKilobotsStates(){
    // Get the Kilobots entities from the space.
    GetKilobotsEntities();
    
    // Create Kilobots individual messages
    m_tMessages=TKilobotsMessagesVector(m_tKilobotsEntities.size());
    
    // timer for the cell messaging
    tLastTimeMessagedGPS.resize(m_tKilobotsEntities.size());
    
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        // init msg timer
        tLastTimeMessagedGPS[GetKilobotId(*m_tKilobotsEntities[it])]=-1000;  // ?
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Sets up the kilogrid environment.                                                             */
/*-----------------------------------------------------------------------------------------------*/
void CArk::SetupVirtualEnvironments(TConfigurationNode& t_tree){
    TConfigurationNode& tVirtualEnvironmentsNode=GetNode(t_tree,"environments");
    std::string type;
    GetNodeAttribute(tVirtualEnvironmentsNode,"type",type);
    if(type=="manual"){
        TConfigurationNodeIterator itNodes;
        
        for(itNodes=itNodes.begin(&tVirtualEnvironmentsNode); itNodes!=itNodes.end(); ++itNodes){
            if(itNodes->Value()=="option"){
                AddOption(*itNodes); // setting option + quality and stuff
            }
        }
        // some check that that qualities match the requirements: Dirty hack -> make it more dynamic
        int checksum = 0;
        for(int c=0; c< m_tOptions.size();c++){
            checksum += m_tOptions[c].quality;
        }
        if (checksum != 576){
            printf("Error in number of qualities \n");
            GetSimulator().Terminate();
        }
        int counter = 0;
        int tmp_counter = 0;
        
        // needed for shuffeling
        std::vector<int> v;
        for (int k=0; k<m_tOptions.size();k++){
            tmp_counter = m_tOptions[k].quality;
            for(int j=0; j<tmp_counter;j++){
                v.push_back(k);
            }
        }
        // shuffle
        if(shuffle){
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(v.begin(), v.end(), g);
        }
        
        // first set borders
        for(int i=0; i < 20; i++){
            for(int j=0; j < 40; j++){
                grid[i][j] = 41;
            }
        }
        // rearranging tiles to a grid
        for (int k=0; k<m_tOptions.size();k++){
            tmp_counter = m_tOptions[k].quality;
            for(int j=0; j<tmp_counter;j++){
                grid[int(counter/36)+2][counter%36+2] = v[counter];  // should distribute the tiles
                counter++;
            }
        }
    }else{
        printf("SOMETHING WRONG IN SETUP TILES \n");
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Sets the experiment variables.                                                                */
/*-----------------------------------------------------------------------------------------------*/
void CArk::GetExperimentVariables(TConfigurationNode& t_tree){
    
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_tree,"variables");
    
    // Get the output datafile name and open it
    GetNodeAttribute(tExperimentVariablesNode, "datafilename", m_strOutputFileName);
    
    // Get if the virtual environments are dynamics or not
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dynamic_virtual_environment",
                              m_bDynamicVirtualEnvironments, m_bDynamicVirtualEnvironments);
    
    // Get the frequency of data saving
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "frequency", m_unDataFrequency,
                              m_unDataFrequency);
    
    // Get the quorum value
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "quorum", m_fQuorum, m_fQuorum);
    
    
    // Get the initial opinion of the robots
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "initial_opinion", initial_opinion,
                              initial_opinion);
    
    // Get the time for one kilobot message
    MinTimeBetweenTwoMsg = 1.0;  // TODO change back - ark has every 5 sec
    
    // Get the gps cell size - oriented on kilogrid so basically its hard coded
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "gps_cellsX", m_unGpsCellsX, m_unGpsCellsX);
    m_unGpsCellsY = m_unGpsCellsX * 2; // arena like table 1 x 2 m
    m_fCellLength=GetSpace().GetArenaSize().GetX()/m_unGpsCellsX;
}


/*-----------------------------------------------------------------------------------------------*/
/* This function updates the virtual sensors of each robot after certain time (like the kilogird */
/* only sends position - of the cell - and the cells opinion).                                   */
/*-----------------------------------------------------------------------------------------------*/
void CArk::UpdateVirtualSensors(){
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        // Update the virtual sensor of a kilobot based on its current state
        UpdateVirtualSensor(*m_tKilobotsEntities[it]);
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Implementation of UpdateVirtualSensors.                                                       */
/*-----------------------------------------------------------------------------------------------*/
void CArk::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity){
    // send msg like it comes from a tile every MinTimeBetweenTwoMsg
    if(m_fTimeInSeconds - tLastTimeMessagedGPS[GetKilobotId(c_kilobot_entity)] > MinTimeBetweenTwoMsg){
        // reset counter
        tLastTimeMessagedGPS[GetKilobotId(c_kilobot_entity)] = m_fTimeInSeconds;
        // Set type of the message 22 means msg from the kilogrid
        m_tMessages[GetKilobotId(c_kilobot_entity)].type = 22;
        // send option of the robot
        // +1 bc arrays start at 0 and options start at 1
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[0] =PositionToOption(GetKilobotPosition(c_kilobot_entity))+1;
        // send x position
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[1] = (UInt8) PositionToGPS(GetKilobotPosition(c_kilobot_entity)).GetX();
        // send y position
        m_tMessages[GetKilobotId(c_kilobot_entity)].data[2] = (UInt8) PositionToGPS(GetKilobotPosition(c_kilobot_entity)).GetY();
        
        // sends msg to the robot
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[GetKilobotId(c_kilobot_entity)]);
        
        
    }else{
        // imitate the global communication
        // TODO: is limited to one crappy option and else good option bc of limited space in msg
        if (global){
            m_tMessages[GetKilobotId(c_kilobot_entity)].type = 23;
            for (int i = 1; i < m_tCommitmentState.size(); i++){
                m_tMessages[GetKilobotId(c_kilobot_entity)].data[i - 1] = m_tCommitmentState[i];
                if(i==1){ // crappy option
                    m_tMessages[GetKilobotId(c_kilobot_entity)].data[i - 1 + m_tCommitmentState.size() - 1] = (UInt8)((m_tOptions[i-1].quality / 576) * 255);
                }else if(i==2){  // equal good option
                    m_tMessages[GetKilobotId(c_kilobot_entity)].data[i - 1 + m_tCommitmentState.size() - 1] = (UInt8)((m_tOptions[i-1].quality / 576) * 255);
                }
            }
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,&m_tMessages[GetKilobotId(c_kilobot_entity)]);
        }else{
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity,NULL);
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* NOT NEEDED AT THE MOMENT                                                                      */
/*-----------------------------------------------------------------------------------------------*/
void CArk::UpdateVirtualEnvironments(){
    /* Updates the virtual environments  based on the kilobots' states */
    //    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
    //        /* Let a kilobot modify the virtual environment  */
    //        UpdatesVirtualEnvironmentsBasedOnKilobotState(*m_tKilobotsEntities[it]);
    //    }
    //
    //    /* Apply change to the options' qualities */
    //    for(unsigned int i=0;i<m_tOptions.size();i++)
    //    {
    //        if(m_tOptions[i].QualityChangeTime!= 0 && m_fTimeInSeconds==m_tOptions[i].QualityChangeTime)
    //        {
    //            LOG << "Option " << i+1 << "had quality "<< m_tOptions[i].quality;
    //
    //            m_tOptions[i].quality=m_tOptions[i].qualityAfterChange;
    //
    //            LOG << "now its quality is" << m_tOptions[i].quality <<std::endl;
    //        }
    //    }
    
}


/*-----------------------------------------------------------------------------------------------*/
/* Implementation of UpdateVirtualEnvironments.                                                  */
/*-----------------------------------------------------------------------------------------------*/
void CArk::UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Here the virtual environment are updated based on the kilobot "kilobot_entity" state */
}


/*-----------------------------------------------------------------------------------------------*/
/* Visualization of the floor color.                                                             */
/*-----------------------------------------------------------------------------------------------*/
CColor CArk::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    int id = PositionToOption(vec_position_on_plane);
    
    // checks if the option is in time
    if(m_fTimeInSeconds>=m_tOptions[id].AppearanceTime
       && (m_fTimeInSeconds<m_tOptions[id].DisappearanceTime
           || m_tOptions[id].DisappearanceTime<=0)){
        if(id == 42){
            // this is wall
            cColor = CColor::WHITE;
        }else{
            cColor=m_tOptions[id].color;
        }
    }
    
    return cColor;
}


/*-----------------------------------------------------------------------------------------------*/
/* Updates the environment. Only needed if we have dynamic environment.                          */
/*-----------------------------------------------------------------------------------------------*/
void CArk::PlotEnvironment(){
    if(m_bDynamicVirtualEnvironments){
        // Update the Floor visualization of the virtual environment every
        // m_unEnvironmentPlotUpdateFrequency ticks
        if (std::find(interestingTimes.begin(), interestingTimes.end(),
                      m_fTimeInSeconds) != interestingTimes.end()){
            GetSpace().GetFloorEntity().SetChanged();
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Needed for creation - called for each option.                                                 */
/*-----------------------------------------------------------------------------------------------*/
void CArk::AddOption(TConfigurationNode& t_node){
    m_sOption sOption;
    
    GetNodeAttribute(t_node, "id", sOption.id);
    
    GetNodeAttribute(t_node, "quality", sOption.quality);
    
    GetNodeAttribute(t_node, "color", sOption.color);
    
    GetNodeAttribute(t_node, "AppearanceTime", sOption.AppearanceTime);
    
    GetNodeAttribute(t_node, "DisappearanceTime", sOption.DisappearanceTime);
    
    GetNodeAttribute(t_node, "QualityChangeTime", sOption.QualityChangeTime);
    
    GetNodeAttribute(t_node, "qualityAfterChange", sOption.qualityAfterChange);
    
    
    if(sOption.AppearanceTime!=0)
        interestingTimes.push_back(sOption.AppearanceTime);
    
    if(sOption.DisappearanceTime!=0)
        interestingTimes.push_back(sOption.DisappearanceTime);
    
    m_tOptions.push_back(sOption);
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns a gps position (deiscrete) to a given continious position.                            */
/*-----------------------------------------------------------------------------------------------*/
CVector2 CArk::PositionToGPS( CVector2 t_position ) {
    return CVector2(Ceil(t_position.GetX()/m_fCellLength)-1,
                    Ceil(t_position.GetY()/m_fCellLength)-1);
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the Option of the cell.                                                               */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CArk::PositionToOption(CVector2 t_position){
    float x = t_position.GetX()*20;
    float y = t_position.GetY()*20;
    
    if (x >= 20) x = 19;
    if (y >= 40) y = 39;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    
    return grid[int(x)][int(y)];
}


// ??
REGISTER_LOOP_FUNCTIONS(CArk, "grid_loop_functions")
