#include "kilogrid.h"

// some notes for the grid
// border is 2 tiles thic thus -> 20*40 -> 16*36 = 576

/*-----------------------------------------------------------------------------------------------*/
/* Initialization of the Loopfunctions.                                                          */
/*-----------------------------------------------------------------------------------------------*/
CKilogrid::CKilogrid():
CLoopFunctions(),
shuffle(true),  // if the tiles should be shuffled -> should always be true
init_flag(true),
INIITALCOMRANGE(0),
m_fTimeInSeconds(0),  // current simulation time in sec
m_unDataSavingCounter(1),
m_bDynamicVirtualEnvironments(false),  // if the system is dynamic
m_unDataFrequency(100),
m_fQuorum(1.0),  // quorum of consensus
m_bQuorumReached(false) {}  // flag for quorum reached

CKilogrid::~CKilogrid(){}


/*-----------------------------------------------------------------------------------------------*/
/* Init method runs before every experiment starts.                                              */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Init(TConfigurationNode& t_node) {
    // create random number generator
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
    // for the adaptive case
    robot_positions.resize(m_tKilobotsEntities.size());
    for(int x=0;x<20;x++){
        for(int y=0;y<40;y++){
            tLastTimeMessagedGridCell[x][y] = 5; // let some initial time
        }
    }
    
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment is resetted.                                                  */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Reset() {
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
void CKilogrid::Destroy() {
    // Close data file
    m_cOutput.close();
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called before every simulation step.                                                     */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PreStep(){
    // Update the time variable required for the experiment (in sec)
    m_fTimeInSeconds=GetSpace().GetSimulationClock()/CPhysicsEngine::GetInverseSimulationClockTick();
    
    // update robot states - handles communication of the robots
    receiveKilobotsMessages();
    // Update the virtual sensor of the kilobots
    sendKilobotsMessages();
    // Update the virtual environment - not needed at the moment
    //UpdateVirtualEnvironments();
    
    // Update the virtual environment plot
    PlotEnvironment();
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called after every simulation step.                                                      */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PostStep(){
    // Save experiment data to the specified log file
    // check if quorum is reached
    std::fill(m_tCommitmentState.begin(), m_tCommitmentState.end(), 0);
    for(unsigned int i=0;i< m_tKilobotsEntities.size();i++){
        m_tCommitmentState[((unsigned int) m_tKBs[i]->commitement)]++;
        // not initial state (crappy option) and uncommitted and more than 0 robots and more than
        // robots than needed for quorum
        if((m_tKBs[i]->commitement!=1) && (m_tKBs[i]->commitement!=0) && (m_fQuorumRobots>0)
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
        printf("[LOOPFUNCTION] Clock at %d ... \n", GetSpace().GetSimulationClock());
    }
    
    // quit simulation if quorum reached
    if(m_bQuorumReached==true){
        printf("[LOOPFUNCTION] reached quorum \n");
        GetSimulator().Terminate();
    }
    
    m_unDataSavingCounter++;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the position of the kilobot.                                                          */
/*-----------------------------------------------------------------------------------------------*/
CVector2 CKilogrid::GetKilobotPosition(CKilobotEntity& c_kilobot_entity){
    CVector2 vecKP(c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                   c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    return vecKP;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the orientation of the kilobot.                                                       */
/*-----------------------------------------------------------------------------------------------*/
CRadians CKilogrid::GetKilobotOrientation(CKilobotEntity& c_kilobot_entity){
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
UInt16 CKilogrid::GetKilobotId(CKilobotEntity& c_kilobot_entity){
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the LED Color of the kilobot.                                                         */
/*-----------------------------------------------------------------------------------------------*/
CColor CKilogrid::GetKilobotLedColor(CKilobotEntity &c_kilobot_entity){
    return c_kilobot_entity.GetLEDEquippedEntity().GetLED(0).GetColor();
}


/*-----------------------------------------------------------------------------------------------*/
/* Creates a list of kilobot robot entities.                                                     */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::GetKilobotsEntities(){
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
void CKilogrid::GetDebugInfo(){
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
void CKilogrid::SetupInitialKilobotsStates(){
    // Get the Kilobots entities from the space.
    GetKilobotsEntities();
    
    // Create Kilobots individual messages
    m_tMessages=TKilobotsMessagesVector(m_tKilobotsEntities.size());
    
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        SetupInitialKilobotState(*m_tKilobotsEntities[it]);
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Implementation of SetupInitialKilobotsStates                                                  */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::SetupInitialKilobotState(CKilobotEntity& c_kilobot_entity){
}


/*-----------------------------------------------------------------------------------------------*/
/* Sets up the kilogrid environment.                                                             */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::SetupVirtualEnvironments(TConfigurationNode& t_tree){
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
        for(int c=0; c < m_tOptions.size();c++){
            checksum += m_tOptions[c].quality;
        }
        if (checksum != 576){
            printf("[LOOPFUNCTION] Error in number of qualities \n");
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
        printf("[LOOPFUNCTION] SOMETHING WRONG IN SETUP TILES \n");
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Sets the experiment variables.                                                                */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::GetExperimentVariables(TConfigurationNode& t_tree){
    
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
    
    // Get the time for one kilobot message
    MinTimeBetweenTwoMsg = 1.0;
    
    // Get the gps cell size - oriented on kilogrid so basically its hard coded
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "gps_cellsX", m_unGpsCellsX, m_unGpsCellsX);
    m_unGpsCellsY = m_unGpsCellsX * 2; // arena like table 1 x 2 m
    m_fCellLength=GetSpace().GetArenaSize().GetX()/m_unGpsCellsX;
    
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "initComRange", INIITALCOMRANGE, INIITALCOMRANGE);
    
    
}


/*-----------------------------------------------------------------------------------------------*/
/* Collects the information the robots send to the kilogrid (here we decided to not go via cell  */
/* because than we would have to loop through ever cell and there every robot -> reality cell    */
/* can on reception set variables)                                                               */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::receiveKilobotsMessages(){
    // this one is a little hack for the simulation - so basically we set the robots positions
    // so that we then can ask if robot is over certain cell - in reality we just have this info
    // because the robot stands physically over the grid
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        robot_positions[GetKilobotId(*m_tKilobotsEntities[it])] = PositionToGPS(GetKilobotPosition(*m_tKilobotsEntities[it]));
    }
    
    // loop through all cells
    for(int x = 0; x < 20; x++){
        for(int y = 0; y < 40; y++){
            receiveKilobotMessage(x, y);
        }
    }
    
    // debug section
    // - printing the grid to the terminal to see if it works
//    if (till){
//        for(int x=0; x < 20; x++){
//            for(int y=0; y < 40; y++){
//                printf("%d", message_grid[x][y].size());
//            }
//            printf("\n");
//        }
//        printf("-------------------------------------------------------\n");
//    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Implementation of receiveKilobotsMessages.                                                    */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::receiveKilobotMessage(int x, int y){
    // we check which robots are above the cell to receive a msg from them - only needed in the sim
    TKilobotEntitiesVector currentRobots;
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        if(robot_positions[GetKilobotId(*m_tKilobotsEntities[it])].GetX() == x and robot_positions[GetKilobotId(*m_tKilobotsEntities[it])].GetY() == y){
            currentRobots.push_back(m_tKilobotsEntities[it]);
        }
    }
    
    // loop over all the robots in this cell
    for(UInt16 it=0; it < currentRobots.size();it++){
        if (m_tKBs[GetKilobotId(*currentRobots[it])]->broadcast_flag == 1){
            int com_rng = m_tKBs[GetKilobotId(*currentRobots[it])]->com_range;
            for(int x_it = x - com_rng; x_it <= x + com_rng; x_it++){
                for (int y_it = y - com_rng; y_it <= y + com_rng; y_it++){
                    // check borders
                    if(x_it >= 0 && x_it < 20 && y_it >= 0 && y_it < 40){
                        // check distance -> use L2/euclidean norm!
                        if(sqrt(pow(fabs(x_it-x),2) + pow(fabs(y_it-y),2)) < com_rng){
                            message_grid[x_it][y_it].push_back(m_tKBs[GetKilobotId(*currentRobots[it])]->commitement);
                        }
                    }
                }
            }
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* This function handles sending information for each cell.                                      */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::sendKilobotsMessages(){
    
    // loop through all cells
    for(int x = 0; x < 20; x++){
        for(int y = 0; y < 40; y++){
            sendKilobotMessage(x, y);
        }
    }
    
//    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
//        // Update the virtual sensor of a kilobot based on its current state
//        UpdateVirtualSensor(*m_tKilobotsEntities[it]);
//    }
    init_flag = false;
    
    // reset message grid bc we send all the msg
    for(int x=0; x < 20; x++){
        for(int y=0; y < 40; y++){
            message_grid[x][y].clear();
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Implementation of UpdateVirtualSensors.                                                       */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::sendKilobotMessage(int x, int y){
    //CKilobotEntity &c_kilobot_entity
    // first of all check if robot is above the cell -- this is only needed in simulation bc we need
    // to know which robots we have to address, in reality we would just broadcast and if robot on
    // top it would receive
    TKilobotEntitiesVector currentRobots;
    for(UInt16 it=0;it< m_tKilobotsEntities.size();it++){
        if(robot_positions[GetKilobotId(*m_tKilobotsEntities[it])].GetX() == x and robot_positions[GetKilobotId(*m_tKilobotsEntities[it])].GetY() == y){
            currentRobots.push_back(m_tKilobotsEntities[it]);
        }
    }
    // now we calculate which message to send for every robot - this is also only necessary in sim
    for(UInt16 it=0; it < currentRobots.size();it++){
        // usage *currentRobots[it]
        // send initial messages - this is also needed in reality
        if(init_flag){
            int tmp_sum = 0;
            int initial_commitment = 0;
            int initial_quality = 0;
            for(int i=0; i < m_tOptions.size();i++){
                tmp_sum += m_tOptions[i].initRobotPopulation;
                if(GetKilobotId(*currentRobots[it])<tmp_sum){
                    initial_commitment = i + 1;  // set inital commitment
                    initial_quality = (UInt8) ((m_tOptions[i].quality/576) * 255);
                    m_tCommitmentState[i+1]++;
                    break;
                }
            }
            // send initial commitment + initial quality
            m_tMessages[GetKilobotId(*currentRobots[it])].type = 10;
            m_tMessages[GetKilobotId(*currentRobots[it])].data[0] = initial_commitment;
            m_tMessages[GetKilobotId(*currentRobots[it])].data[1] = initial_quality;
            m_tMessages[GetKilobotId(*currentRobots[it])].data[2] = m_tOptions.size();
            m_tMessages[GetKilobotId(*currentRobots[it])].data[3] = PositionToOption(GetKilobotPosition(*currentRobots[it]))+1;
            m_tMessages[GetKilobotId(*currentRobots[it])].data[4] = INIITALCOMRANGE;

            // sends msg to the robot
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(*currentRobots[it],&m_tMessages[GetKilobotId(*currentRobots[it])]);
            continue;
        }
        // boradcast message from other robot - virtualized
        if(message_grid[x][y].size() > 0){
            unsigned int commitement_to_send = message_grid[x][y][m_pcRNG->Uniform(CRange<UInt32>(0, message_grid[x][y].size()))];
            m_tMessages[GetKilobotId(*currentRobots[it])].type = 24;
            m_tMessages[GetKilobotId(*currentRobots[it])].data[0] = commitement_to_send;
            
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(*currentRobots[it],&m_tMessages[GetKilobotId(*currentRobots[it])]);
            continue;
        }
        // send position message aka GRID_MSG
        if(m_fTimeInSeconds - tLastTimeMessagedGridCell[x][y] > MinTimeBetweenTwoMsg){
            // Set type of the message 22 means msg from the kilogrid
            m_tMessages[GetKilobotId(*currentRobots[it])].type = 22;
            // send option of the robot
            // +1 bc arrays start at 0 and options start at 1
            m_tMessages[GetKilobotId(*currentRobots[it])].data[0] = GridCellToOption(CVector2(x, y))+1;
            // send x position
            m_tMessages[GetKilobotId(*currentRobots[it])].data[1] = (UInt8) x;
            // send y position
            m_tMessages[GetKilobotId(*currentRobots[it])].data[2] = (UInt8) y;
    
            // sends msg to the robot
            GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(*currentRobots[it],&m_tMessages[GetKilobotId(*currentRobots[it])]);
            continue;
        }
    }
    // timer need to be resetted even if there was no robot in order to have a cycle in the gridcells
    if(m_fTimeInSeconds - tLastTimeMessagedGridCell[x][y] > MinTimeBetweenTwoMsg){
        tLastTimeMessagedGridCell[x][y] = m_fTimeInSeconds;
    }
    
}


/*-----------------------------------------------------------------------------------------------*/
/* NOT NEEDED AT THE MOMENT                                                                      */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::UpdateVirtualEnvironments(){
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
void CKilogrid::UpdatesVirtualEnvironmentsBasedOnKilobotState(CKilobotEntity &c_kilobot_entity){
    /* Here the virtual environment are updated based on the kilobot "kilobot_entity" state */
}


/*-----------------------------------------------------------------------------------------------*/
/* Visualization of the floor color.                                                             */
/*-----------------------------------------------------------------------------------------------*/
CColor CKilogrid::GetFloorColor(const CVector2 &vec_position_on_plane) {
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
void CKilogrid::PlotEnvironment(){
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
void CKilogrid::AddOption(TConfigurationNode& t_node){
    m_sOption sOption;
    
    GetNodeAttribute(t_node, "id", sOption.id);
    
    GetNodeAttribute(t_node, "quality", sOption.quality);
    
    GetNodeAttribute(t_node, "color", sOption.color);
    
    GetNodeAttribute(t_node, "AppearanceTime", sOption.AppearanceTime);
    
    GetNodeAttribute(t_node, "DisappearanceTime", sOption.DisappearanceTime);
    
    GetNodeAttribute(t_node, "QualityChangeTime", sOption.QualityChangeTime);
    
    GetNodeAttribute(t_node, "qualityAfterChange", sOption.qualityAfterChange);
    
    GetNodeAttribute(t_node, "initRobotPopulation", sOption.initRobotPopulation);
    
    
    if(sOption.AppearanceTime!=0)
        interestingTimes.push_back(sOption.AppearanceTime);
    
    if(sOption.DisappearanceTime!=0)
        interestingTimes.push_back(sOption.DisappearanceTime);
    
    m_tOptions.push_back(sOption);
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns a gps position (deiscrete) to a given continious position.                            */
/*-----------------------------------------------------------------------------------------------*/
CVector2 CKilogrid::PositionToGPS( CVector2 t_position ) {
    return CVector2(Ceil(t_position.GetX()/m_fCellLength)-1,
                    Ceil(t_position.GetY()/m_fCellLength)-1);
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the Option of the given position.                                                     */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CKilogrid::PositionToOption(CVector2 t_position){
    float x = t_position.GetX()*20;
    float y = t_position.GetY()*20;
    
    if (x >= 20) x = 19;
    if (y >= 40) y = 39;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    
    return grid[int(x)][int(y)];
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the Option of a given cell.                                                           */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CKilogrid::GridCellToOption(CVector2 t_positionCell){
    float x = t_positionCell.GetX();
    float y = t_positionCell.GetY();
    if (x >= 20) x = 19;
    if (y >= 40) y = 39;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    
    return grid[int(x)][int(y)];
}


// ??
REGISTER_LOOP_FUNCTIONS(CKilogrid, "kilogrid_loop_functions")
