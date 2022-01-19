//
// Created by Till Konrad Aust on 17.12.21.
//
// in case of simulation set flag
#define SIMULATION

#ifdef SIMULATION
#define MSG_SEND_TRIES 1
#define MAX_RESET_TIMER 1
#define MIN_TIME_BETWEEN_MSG 1
#else
#define MSG_SEND_TRIES 10
#define MAX_RESET_TIMER 5
#endif

#include "kilogrid_stub.h"

/*-----------------------------------------------------------------------------------------------*/
/* Initialization of the Loopfunctions.                                                          */
/*-----------------------------------------------------------------------------------------------*/
CKilogrid::CKilogrid():
CLoopFunctions() {}

CKilogrid::~CKilogrid() {}



/*-----------------------------------------------------------------------------------------------*/
/* Init method runs before every experiment starts.                                              */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Init(TConfigurationNode &t_tree) {
    m_pcRNG = CRandom::CreateRNG("argos");

    // get all kilobots
    get_kilobots_entities();

    // get experiment variables from the .argos file
//    setup_virtual_environments(t_tree);

    // get the debug info structs aka communication with the kilogrid
    GetDebugInfo();

    // init map and other configuration for the simulation
    read_configuration(t_tree);
    for(int x_it = 0; x_it < 10; x_it++){
        for(int y_it = 0; y_it < 20; y_it++){
            setup(x_it, y_it);
        }
    }
    // initialize some helpers to track the kilobots - only needed in sim
    robot_positions.resize(kilobot_entities.size());
    logg_commitment_state.resize(number_of_options+1);  // uncommitted is also a commitment state thus +1


    // init logging
    output_logg.open(data_file_name, std::ios_base::trunc | std::ios_base::out);
    output_logg << "time";
    for(unsigned int i=0;i<logg_commitment_state.size();i++){
        output_logg << ";" << i;
    }
    output_logg<< std::endl;
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment is resetted.                                                  */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Reset() {
    // Close data file
    output_logg.close();

    // Reopen the file, erasing its contents
    output_logg.open(data_file_name, std::ios_base::trunc | std::ios_base::out);
    // write the log file header (it is not mendatory)
    output_logg << "time";
    for(unsigned int i=0;i<logg_commitment_state.size();i++){
        output_logg << ";" << i;
    }
    output_logg<< std::endl;
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment ended.                                                        */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Destroy() {
    output_logg.close();

}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called before every simulation step.                                                     */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PreStep(){
    // collect all the data from the robots - and virtualize them
    virtual_message_reception();

    // simulate IR message reception for each cell
    for(int x_it = 0; x_it < 10; x_it++){
        for(int y_it = 0; y_it < 20; y_it++){
            if(module_memory[x_it][y_it].robot_messages.size() > 0){
                // IMPORTANT: cellid, distance and crc error are not used in simulation: here they are
                // dummy values
                IR_rx(x_it, y_it, module_memory[x_it][y_it].robot_messages[m_pcRNG->Uniform(CRange<UInt32>(0,module_memory[x_it][y_it].robot_messages.size()))], cell_id[0], d, 0);
                module_memory[x_it][y_it].robot_messages.clear();
            }
        }
    }

    // simulate reception of can messages
    for(int x_it = 0; x_it < 10; x_it++){
        for(int y_it = 0; y_it < 20; y_it++){
            // if we have multiple messages at the same time select a random one
            if(module_memory[x_it][y_it].received_cell_messages.size() > 0){
                // shuffle and take first element (this way because we also consider case == 1)
                CAN_rx(x_it, y_it, &module_memory[x_it][y_it].received_cell_messages[m_pcRNG->Uniform(CRange<UInt32>(0,module_memory[x_it][y_it].received_cell_messages.size()))]);
                // clear list afterwards
                module_memory[x_it][y_it].received_cell_messages.clear();
            }
        }
    }

    // run the loop
    for(int x_it = 0; x_it < 10; x_it++){
        for(int y_it = 0; y_it < 20; y_it++){
            loop(x_it, y_it);
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called after every simulation step.                                                      */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PostStep(){
    // Save experiment data to the specified log file
    // check if quorum is reached
//    int inactive_robots = 0;
    std::fill(logg_commitment_state.begin(), logg_commitment_state.end(), 0);
    for(unsigned int i=0;i< kilobot_entities.size();i++){
        logg_commitment_state[((unsigned int) debug_info_kilobots[i]->commitement)]++;
//        if (debug_info_kilobots[i]->inactive){
//            inactive_robots++;
//        }
    }

    // if quroum reached, time to write something down, max time passed
    if( (data_saving_counter%DATA_SAVING_FREQUENCY == 0) || (GetSpace().GetSimulationClock() >= GetSimulator().GetMaxSimulationClock()) ){
        output_logg << GetSpace().GetSimulationClock();
        for(unsigned int i=0;i< logg_commitment_state.size();i++){
            output_logg << ";" << logg_commitment_state[i];
        }
        output_logg<< std::endl;
    }

    // reset init flag if not every robot is inited -> robots cannot get uncommitted
    if (logg_commitment_state[0] > 0) {
        printf("[LOOPFUNCTION] not all robots are inited; repeat... \n");
        for (int x_it = 0; x_it < 10; x_it++) {
            for (int y_it = 0; y_it < 20; y_it++) {
                module_memory[x_it][y_it].init_flag = false;

            }
        }
    }

    // for viz -> that i can see the progression
    if(GetSpace().GetSimulationClock() % 1000 == 0){
        printf("[LOOPFUNCTION] Clock at %d ... \n", GetSpace().GetSimulationClock());
    }

    // quit simulation if quorum reached
//    if(m_bQuorumReached==true){
//        printf("[LOOPFUNCTION] reached quorum \n");
//        GetSimulator().Terminate();
//    }

    data_saving_counter++;

    // debug section
//    printf("Distribution ");
//    for(unsigned int i=0;i< logg_commitment_state.size();i++){
//        printf("%d ", logg_commitment_state[i]);
//    }
//    printf("inactive robots %d \n", inactive_robots);
//    printf("\n%d------------------------------------------------------------------------------\n", GetSpace().GetSimulationClock());
}


/*-----------------------------------------------------------------------------------------------*/
/* This function reads the config file for the kilogrid and saves the content to configuration.  */
/* Further it reads other configurations from the argos file such as datafilename...             */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::read_configuration(TConfigurationNode& t_node){
    // getting config script name from .argos file
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_node,"variables");
    GetNodeAttribute(tExperimentVariablesNode, "config_file", config_file_name);
    GetNodeAttribute(tExperimentVariablesNode, "data_file", data_file_name);
    GetNodeAttribute(tExperimentVariablesNode, "initial_commitment", initial_commitment);
    GetNodeAttribute(tExperimentVariablesNode, "number_of_options", number_of_options);
    GetNodeAttribute(tExperimentVariablesNode, "initial_communication_range", initial_communication_range);

    // variables used for reading
    int tmp_x_module;
    int tmp_y_module;
    int tmp_value;
    int read_mode = 0; // 1 is address, 2 is data
    std::string tmp_str;

    // reading
    input.open(config_file_name);
    for( std::string line; getline( input, line ); ){
        // for each line
        if(!line.empty() && line[0] != '#'){  // exclude comments and empty lines

            //printf("%s\n",line.c_str()); // <- this way you can pring the line out!
            if (line == "address"){
                read_mode = 1;
            }else if(line == "data"){
                read_mode = 2;
            } else{
                // reading address
                if(read_mode == 1){
                    // isolating numbers
                    tmp_str = line;
                    tmp_str = tmp_str.substr(7);  // remove module
                    std::string::size_type p  = tmp_str.find('-');
                    tmp_y_module = std::stoi(tmp_str.substr(p+1));
                    tmp_x_module = std::stoi(tmp_str);
                }else if(read_mode == 2){ // read data
                    tmp_value = std::stoi(line, 0, 16);
                    configuration[tmp_x_module][tmp_y_module].push_back(tmp_value);
                }
            }
        }
    }
}


void CKilogrid::set_LED_with_brightness(int x, int y, cell_num_t cn, color_t color, brightness_t brightness){
    module_memory[x][y].cell_colour[cn] = color;
    GetSpace().GetFloorEntity().SetChanged();
}


/*-----------------------------------------------------------------------------------------------*/
/* Visualization of the floor color.                                                             */
/*-----------------------------------------------------------------------------------------------*/
CColor CKilogrid::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor=CColor::WHITE;
    int id = position2option(vec_position_on_plane);

    // checks if the option is in time
    if(id == 0){
        // this is wall
        cColor = CColor::WHITE;
    }else if(id == 1){
        cColor=CColor::RED;
    }else if(id == 2){
        cColor=CColor::GREEN;
    }else if(id == 3){
        cColor=CColor::BLUE;
    }else if(id == 4){
        cColor=CColor::BROWN;
    }else if(id == 5){
        cColor=CColor::BLACK;
    }

    return cColor;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the Option of the given position.                                                     */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CKilogrid::position2option(CVector2 t_position){
    int module_x = t_position.GetX()*10;
    int module_y = t_position.GetY()*10;

    uint8_t cell = 0;
    int x = t_position.GetX()*20;
    int y = t_position.GetY()*20;

    if (x >= 20) x = 19;
    if (y >= 40) y = 39;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    // get the cell
    if(x % 2 == 0 && y % 2 == 1){
        cell = 0;
    }else if(x % 2 == 1 && y % 2 == 1){
        cell = 1;
    }else if(x % 2 == 0 && y % 2 == 0){
        cell = 2;
    }else if(x % 2 == 1 && y % 2 == 0){
        cell = 3;
    }

    return module_memory[module_x][module_y].cell_colour[cell];
}

CVector2 CKilogrid::position2cell(CVector2 t_position){
    int x = t_position.GetX()*20;
    int y = t_position.GetY()*20;

    if (x >= 20) x = 19;
    if (y >= 40) y = 39;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    return CVector2(x,y);
}


CVector2 CKilogrid::position2module(CVector2 t_position) {
    float x = t_position.GetX() * 10;
    float y = t_position.GetY() * 10;

    if (x >= 10) x = 9;
    if (y >= 20) y = 19;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    return CVector2(int(x),int(y));
}

CVector2 CKilogrid::module2cell(CVector2 module_pos, cell_num_t cn){
    int cur_x = module_pos.GetX() * 2;
    int cur_y = module_pos.GetY() * 2;

    switch(cn){
        case 0:
            cur_y += 1;
            break;
        case 1:
            cur_x += 1;
            cur_y += 1;
            break;
        case 2:
            break;
        case 3:
            cur_x += 1;
    }

    return CVector2(cur_x, cur_y);
}

void CKilogrid::get_kilobots_entities(){
    // Get the map of all kilobots from the space
    CSpace::TMapPerType& mapKilobots=GetSpace().GetEntitiesByType("kilobot");
    // Go through them
    for(CSpace::TMapPerType::iterator it = mapKilobots.begin();it != mapKilobots.end();++it) {
        kilobot_entities.push_back(any_cast<CKilobotEntity*>(it->second));
    }
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
/* Returns the id of the kilobot.                                                                */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CKilogrid::GetKilobotId(CKilobotEntity& c_kilobot_entity){
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}

void CKilogrid::virtual_message_reception(){
    // for debug
//    int msg_op[3] = {0, 0, 0};
    // get position information, later used for sending stuff!!
    for(UInt16 it = 0; it < kilobot_entities.size(); it++) {
        robot_positions[GetKilobotId(*kilobot_entities[it])] = position2cell(
                GetKilobotPosition(*kilobot_entities[it]));
    }

    for(UInt16 it = 0; it < debug_info_kilobots.size(); it++) {
        if (debug_info_kilobots[it]->broadcast_flag == 1) {
            int module_x = int(robot_positions[GetKilobotId(*kilobot_entities[it])].GetX())/2;
            int module_y = int(robot_positions[GetKilobotId(*kilobot_entities[it])].GetY())/2;
            IR_message_t* tmp_msg = new IR_message_t;
            tmp_msg->type = debug_info_kilobots[it]->type;
            tmp_msg->data[0] = debug_info_kilobots[it]->data0;
            tmp_msg->data[1] = debug_info_kilobots[it]->data1;
            tmp_msg->data[2] = debug_info_kilobots[it]->data2;
            tmp_msg->data[3] = debug_info_kilobots[it]->data3;
            tmp_msg->data[4] = debug_info_kilobots[it]->data4;
            tmp_msg->data[5] = debug_info_kilobots[it]->data5;
            tmp_msg->data[6] = debug_info_kilobots[it]->data6;
            tmp_msg->data[7] = debug_info_kilobots[it]->data7;
            module_memory[module_x][module_y].robot_messages.push_back(tmp_msg);

            // debug
//            msg_op[tmp_msg->data[2]-1]++;
        }
    }

//    printf("msg distribution ");
//    for(int debug_i = 0; debug_i < 3;debug_i++){
//        printf(" %d", msg_op[debug_i]);
//    }
//    printf("\n");
}


void CKilogrid::set_IR_message(int x, int y, IR_message_t &m, cell_num_t cn) {
    // get correct cell
    CVector2 cell_pos = module2cell(CVector2(x,y), cn);

    // get kilobots on this cell
    kilobot_entities_vector current_robots;

    for(uint8_t it=0;it < kilobot_entities.size();it++){
        if(robot_positions[GetKilobotId(*kilobot_entities[it])].GetX() == cell_pos.GetX()
        and robot_positions[GetKilobotId(*kilobot_entities[it])].GetY() == cell_pos.GetY()){
            current_robots.push_back(kilobot_entities[it]);
        }
    }

    // send to all kilobots on this cell
    for(UInt16 it=0; it < current_robots.size(); it++){
        message_t message_to_send;
        message_to_send.type = m.type;
        message_to_send.data[0] = m.data[0];
        message_to_send.data[1] = m.data[1];
        message_to_send.data[2] = m.data[2];
        message_to_send.data[3] = m.data[3];
        message_to_send.data[4] = m.data[4];
        message_to_send.data[5] = m.data[5];
        message_to_send.data[6] = m.data[6];
        message_to_send.data[7] = m.data[7];
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(*current_robots[it], &message_to_send);
    }
}

/*-----------------------------------------------------------------------------------------------*/
/* Set up the getting of debug information from the robot (e.g. the robots state).               */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::GetDebugInfo(){
    debug_info_kilobots.clear();
    for(UInt16 it=0;it< kilobot_entities.size();it++){
        // Check if there is a message to send to the kilobot "i" and get the message

        /*
         * When the 'reset' method is called on the kilobot controller, the
         * kilobot state is destroyed and recreated. Thus, we need to
         * recreate the list of controllers and debugging info from scratch
         * as well.
         */
        // Create a pointer to the current kilobot
        CCI_KilobotController* pcKBC = &dynamic_cast<CCI_KilobotController&>(kilobot_entities[it]->GetControllableEntity().GetController());
        // Create debug info for controller
        debug_info_t* ptDebugInfo = pcKBC->DebugInfoCreate<debug_info_t>();
        // Append to list
        debug_info_kilobots.push_back(ptDebugInfo);
    }
}

void CKilogrid::init_CAN_message(CAN_message_t* cell_msg){
    uint8_t d;

    cell_msg->id = 0;
    cell_msg->header.rtr = 0;
    cell_msg->header.length = 8;

    for(d = 0; d < 8; d++) cell_msg->data[d] = 0;
}


uint8_t CKilogrid::CAN_message_tx(CAN_message_t *m, kilogrid_address_t add) {
    // this is a hack because we can only virtually set msgs
    // TODO is this message coppied or just the pointer
    // module_memory[add.x][add.y].received_cell_message = m;
    module_memory[add.x][add.y].received_cell_messages.push_back(*m);

    return 1;
}




/*-----------------------------------------------------------------------------------------------*/
/*  HERE STARTS THE METHODS TO IMPLEMENT!!!!!                                                    */
/*                                                                                               */
/*-----------------------------------------------------------------------------------------------*/

// TODO here we implement the logic for what should run aka this methods should
// be implemented such as on the real kilogrid

// is called before the experiment - draws the environment for example
// for the real kilogrid you need to remove the x and y
// for example configuration[0] instead of configuration[x][y][0]
// also you have to remove the module memory!!
void CKilogrid::setup(int x, int y){
    // the real line is
    // cell_x[0] = (configuration[0] * 2);
    module_memory[x][y].cell_x[0] = (configuration[x][y][0] * 2);
    module_memory[x][y].cell_x[1] = (configuration[x][y][0] * 2 + 1);
    module_memory[x][y].cell_x[2] = (configuration[x][y][0] * 2);
    module_memory[x][y].cell_x[3] = (configuration[x][y][0] * 2 + 1);

    module_memory[x][y].cell_y[0] = (configuration[x][y][1] * 2 + 1);
    module_memory[x][y].cell_y[1] = (configuration[x][y][1] * 2 + 1);
    module_memory[x][y].cell_y[2] = (configuration[x][y][1] * 2);
    module_memory[x][y].cell_y[3] = (configuration[x][y][1] * 2);

    module_memory[x][y].cell_role[0] = (configuration[x][y][2]);
    module_memory[x][y].cell_role[1] = (configuration[x][y][2]);
    module_memory[x][y].cell_role[2] = (configuration[x][y][2]);
    module_memory[x][y].cell_role[3] = (configuration[x][y][2]);

    module_memory[x][y].cell_colour[0] = color_t (configuration[x][y][3]);
    module_memory[x][y].cell_colour[1] = color_t (configuration[x][y][4]);
    module_memory[x][y].cell_colour[2] = color_t (configuration[x][y][5]);
    module_memory[x][y].cell_colour[3] = color_t (configuration[x][y][6]);

    module_memory[x][y].status_msg_counter = MIN_TIME_BETWEEN_MSG;

    // set id of received can message to 0 in order to initialize it -> maybe only needed in sim because
    // in reality we do not have the issue bc its a callback
    // module_memory[x][y].received_cell_message->id = 0;

    for(int i = 0; i < 4; i++){
        set_LED_with_brightness(x, y, cell_id[i], module_memory[x][y].cell_colour[i], HIGH);
    }
}


// this method implements the loop function
void CKilogrid::loop(int x, int y){
    // the real line is
    // test_counter += 1;
    // module_memory[x][y].test_counter += 1;

    // send initial message
    if(!module_memory[x][y].init_flag){
        module_memory[x][y].init_flag = true;
        for (int f = 0; f < 4; f++){
            module_memory[x][y].ir_message_tx->type = INIT_MSG;
            module_memory[x][y].ir_message_tx->crc = 0;
            module_memory[x][y].ir_message_tx->data[0] = module_memory[x][y].cell_x[f];
            module_memory[x][y].ir_message_tx->data[1] = module_memory[x][y].cell_y[f];
            module_memory[x][y].ir_message_tx->data[2] = initial_commitment; // TODO adjustable for each robot
            module_memory[x][y].ir_message_tx->data[3] = 70; // TODO do dynamic: initial commitment quality
            module_memory[x][y].ir_message_tx->data[4] = number_of_options;
            module_memory[x][y].ir_message_tx->data[5] = module_memory[x][y].cell_colour[f];  // tested cast to unit_8 not needed
            module_memory[x][y].ir_message_tx->data[6] = initial_communication_range;

            set_IR_message(x, y, *module_memory[x][y].ir_message_tx, cell_id[f]);
        }
    } else {
        for (int c = 0; c < 4; c++) {
            module_memory[x][y].cell_op[c] = module_memory[x][y].received_cell_op[c];
            module_memory[x][y].status_msg_counter += 1;

            // send messages - forward before sending status
            if (module_memory[x][y].cell_op[c] > 0) {
                module_memory[x][y].can_kilo_uid = module_memory[x][y].received_can_kilo_uid;

                module_memory[x][y].ir_message_tx->type = VIRTUAL_AGENT_MSG;
                module_memory[x][y].ir_message_tx->crc = 0;
                module_memory[x][y].ir_message_tx->data[0] = module_memory[x][y].cell_x[c];
                module_memory[x][y].ir_message_tx->data[1] = module_memory[x][y].cell_y[c];
                module_memory[x][y].ir_message_tx->data[2] = module_memory[x][y].cell_op[c];
                module_memory[x][y].ir_message_tx->data[3] = module_memory[x][y].can_kilo_uid;
                set_IR_message(x, y, *module_memory[x][y].ir_message_tx, cell_id[c]);

                // reset after certain time
                module_memory[x][y].reset_timer[c] += 1;
                if (module_memory[x][y].reset_timer[c] >= MAX_RESET_TIMER) {
                    module_memory[x][y].cell_op[c] = 0;
                    module_memory[x][y].received_cell_op[c] = 0;
                }
            } else {
                // sending status
                if(module_memory[x][y].status_msg_counter >= MIN_TIME_BETWEEN_MSG){
                    module_memory[x][y].status_msg_counter = 0;
                    module_memory[x][y].ir_message_tx->type = GRID_MSG;
                    module_memory[x][y].ir_message_tx->crc = 0;
                    module_memory[x][y].ir_message_tx->data[0] = module_memory[x][y].cell_x[c];
                    module_memory[x][y].ir_message_tx->data[1] = module_memory[x][y].cell_y[c];
                    module_memory[x][y].ir_message_tx->data[2] = module_memory[x][y].cell_colour[c];
                    module_memory[x][y].ir_message_tx->data[3] = module_memory[x][y].cell_role[c];
                    set_IR_message(x, y, *module_memory[x][y].ir_message_tx, cell_id[c]);
                }
            }

            // process the received IR messages
            if (module_memory[x][y].received_IR_msg_cell == c) {
                // fix parameters so they do not get overwritten while processing #callbackmagic
                module_memory[x][y].com_range = module_memory[x][y].received_com_range;
                module_memory[x][y].option = module_memory[x][y].received_option;
                module_memory[x][y].my_x = module_memory[x][y].received_x;
                module_memory[x][y].my_y = module_memory[x][y].received_y;
                module_memory[x][y].kilo_uid = module_memory[x][y].received_kilo_uid;

                // resetting the sending grid
                for(uint8_t i_it = 0; i_it < 10; i_it++){
                    for(uint8_t k_it = 0; k_it < 20; k_it++){
                        for(uint8_t l_it = 0; l_it < 4; l_it++){
                            module_memory[x][y].sending_grid[i_it][k_it][l_it] = 0;
                        }
                    }
                }

                // calculate receiving cells
                for(int x_it = module_memory[x][y].my_x - module_memory[x][y].com_range; x_it <= module_memory[x][y].my_x + module_memory[x][y].com_range; x_it++){
                    for(int y_it = module_memory[x][y].my_y - module_memory[x][y].com_range; y_it <= module_memory[x][y].my_y + module_memory[x][y].com_range; y_it++){
                        // check borders
                        if(x_it >= 0 && x_it < 20 && y_it >= 0 && y_it < 40){
                            // check distance -> use L2/euclidean norm!
                            if(sqrt(pow(fabs(x_it-module_memory[x][y].my_x),2) + pow(fabs(y_it-module_memory[x][y].my_y),2)) < module_memory[x][y].com_range){
                                // which cells do we have to address
                                if(y_it % 2 == 1 && x_it % 2 == 0){
                                    module_memory[x][y].sending_grid[(int)(x_it/2)][(int)(y_it/2)][0] = module_memory[x][y].option;
                                }else if(y_it % 2 == 1 && x_it % 2 == 1){
                                    module_memory[x][y].sending_grid[(int)(x_it/2)][(int)(y_it/2)][1] = module_memory[x][y].option;
                                }else if(y_it % 2 == 0 && x_it % 2 == 0){
                                    module_memory[x][y].sending_grid[(int)(x_it/2)][(int)(y_it/2)][2] = module_memory[x][y].option;
                                }else if(y_it % 2 == 0 && x_it % 2 == 1){
                                    module_memory[x][y].sending_grid[(int)(x_it/2)][(int)(y_it/2)][3] = module_memory[x][y].option;
                                }
                            }
                        }
                    }
                }

                // iterate through all cells - to check if we need to send a msg to this cell
                module_memory[x][y].com_range_module = (uint8_t)(module_memory[x][y].com_range/2 + 1);
                module_memory[x][y].send_success_sum = 0;
                module_memory[x][y].my_x_module = (uint8_t)(module_memory[x][y].my_x/2);
                module_memory[x][y].my_y_module = (uint8_t)(module_memory[x][y].my_y/2);
                for(int x_it = module_memory[x][y].my_x_module - module_memory[x][y].com_range_module; x_it < module_memory[x][y].my_x_module + module_memory[x][y].com_range_module; x_it++){
                    for(int y_it = module_memory[x][y].my_y_module - module_memory[x][y].com_range_module; y_it < module_memory[x][y].my_y_module + module_memory[x][y].com_range_module; y_it++){
                        // check borders - modules
                        if(x_it >= 0 && x_it < 10 && y_it >= 0 && y_it < 20){
                            module_memory[x][y].send_flag = 0;
                            CAN_message_t tmp_can_msg;
                            init_CAN_message(&tmp_can_msg);

                            tmp_can_msg.id = 55;  // dont know if id is important - maybe to check if msg arrived twice or so - max 65,535
                            tmp_can_msg.data[0] = 55; // maybe this is the msg type (must be larger than 25 to dont overwrite something and less than 64 see communication/CAN.h)
                            for (uint8_t cell_it = 0; cell_it < 4; cell_it++){
                                tmp_can_msg.data[cell_it + 1] = module_memory[x][y].sending_grid[x_it][y_it][cell_it];
                                if(module_memory[x][y].sending_grid[x_it][y_it][cell_it] != 0){
                                    module_memory[x][y].send_flag = 1;
                                }
                            }
                            tmp_can_msg.data[5] = module_memory[x][y].kilo_uid;

                            module_memory[x][y].cell_address.type = ADDR_INDIVIDUAL; // see communication/kilogrid.h for further information
                            module_memory[x][y].cell_address.x = x_it;  // is the position of a module imo??
                            module_memory[x][y].cell_address.y = y_it;

                            if(module_memory[x][y].send_flag == 1){
                                module_memory[x][y].send_success_sum = module_memory[x][y].send_success_sum + CAN_message_tx(&tmp_can_msg, module_memory[x][y].cell_address);
#ifndef SIMULATION
                                _delay_ms(10);  // probably needed for message transmission
#endif
                            }
                        }
                    }
                }

                // apperently the module cannot send itself a msg so we have to set the broad cast manualy
                module_memory[x][y].cell_op[0] = module_memory[x][y].option;
                module_memory[x][y].cell_op[1] = module_memory[x][y].option;
                module_memory[x][y].cell_op[2] = module_memory[x][y].option;
                module_memory[x][y].cell_op[3] = module_memory[x][y].option;

                // also init reset timer
                module_memory[x][y].reset_timer[0] = 0;
                module_memory[x][y].reset_timer[1] = 0;
                module_memory[x][y].reset_timer[2] = 0;
                module_memory[x][y].reset_timer[3] = 0;

                // reset after sending a msg
                module_memory[x][y].received_IR_msg_cell = 10;
            }
        }
    }
}


void CKilogrid::IR_rx(int x, int y, IR_message_t *m, cell_num_t c, distance_measurement_t *d, uint8_t CRC_error) {
    if (!CRC_error && m->type == MSG_T_VIRTUAL_ROBOT_MSG) {
        // message from robot to kilogrid: broadcast
        module_memory[x][y].msg_number_current = m->data[4];
        if (module_memory[x][y].msg_number_current != module_memory[x][y].msg_number) {
            // case new message
            module_memory[x][y].msg_number = module_memory[x][y].msg_number_current;
            // logic here
            module_memory[x][y].received_x = m->data[0];
            module_memory[x][y].received_y = m->data[1];
            module_memory[x][y].received_option = m->data[2];
            module_memory[x][y].received_com_range = m->data[3];
            module_memory[x][y].received_kilo_uid = m->data[5];

            // check which cell was addressed
            for(int c = 0; c < 4; c++){
                if(module_memory[x][y].received_x == module_memory[x][y].cell_x[c] && module_memory[x][y].received_y == module_memory[x][y].cell_y[c]){
                    module_memory[x][y].received_IR_msg_cell = c;
                }
            }
        } else {
            // message already seen -> discard
            //printf("msg received alrdy seen at %d %d from %d with option %d \n", x, y, m->data[5], m->data[2]);
            return;
        }
#ifndef SIMULATION
    }else if(!CRC_error && m->type == TRACKING){ // here we have to write down the logging
        CAN_message_t *msg = next_CAN_message();
        if (msg != NULL) { // if the buffer is not full
            serialize_tracking_message(msg, c, m->data);
        }
#endif
    }else{
        printf("ERROR: unknown IR message type at module %d %d \n", x, y);
    }
}

void CKilogrid::CAN_rx(int x, int y, CAN_message_t *m){
    if (m->data[0] == 55){  // set msg
        module_memory[x][y].received_cell_op[0] = m->data[1];
        module_memory[x][y].received_cell_op[1] = m->data[2];
        module_memory[x][y].received_cell_op[2] = m->data[3];
        module_memory[x][y].received_cell_op[3] = m->data[4];
        module_memory[x][y].received_can_kilo_uid = m->data[5];
//        printf("received can msg at %d %d from robot %d with content %d %d %d %d \n", x, y, m->data[5], module_memory[x][y].received_cell_op[0], module_memory[x][y].received_cell_op[1], module_memory[x][y].received_cell_op[2], module_memory[x][y].received_cell_op[3]);

        for(uint8_t cell_it_cb = 0; cell_it_cb < 4; cell_it_cb++){
            if(module_memory[x][y].received_cell_op[cell_it_cb] != 0){
                module_memory[x][y].reset_timer[cell_it_cb] = 0;
            }
        }
    }else {
        printf("ERROR: unknown CAN message type at module %d %d \n", x, y);
    }
}


// ??
REGISTER_LOOP_FUNCTIONS(CKilogrid, "kilogrid_loop_functions")