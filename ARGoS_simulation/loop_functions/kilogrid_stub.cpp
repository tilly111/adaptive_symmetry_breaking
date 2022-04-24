//
// Created by Till Konrad Aust on 17.12.21.
//
// in case of simulation set flag
#define SIMULATION

#ifdef SIMULATION
#define MSG_SEND_TRIES 1
#define MAX_RESET_TIMER 1
#define MIN_TIME_BETWEEN_MSG 0
#else
#define MSG_SEND_TRIES 10
#define MAX_RESET_TIMER 5
#endif

#include "kilogrid_stub.h"

/*-----------------------------------------------------------------------------------------------*/
/* Initialization of the Loopfunctions.                                                          */
/*-----------------------------------------------------------------------------------------------*/
CKilogrid::CKilogrid() :
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
    for (int x_it = 0; x_it < 10; x_it++) {
        for (int y_it = 0; y_it < 20; y_it++) {
            setup(x_it, y_it);
        }
    }
    GetSpace().GetFloorEntity().SetChanged();
    // initialize some helpers to track the kilobots - only needed in sim
    robot_positions.resize(kilobot_entities.size());
    logg_commitment_state.resize(
            number_of_options + 1);  // uncommitted is also a commitment state thus +1


    // init logging
    output_logg.open(data_file_name, std::ios_base::trunc | std::ios_base::out);
    output_logg << "time";
    for (unsigned int i = 0; i < logg_commitment_state.size(); i++) {
        output_logg << ";" << i;
    }
    output_logg << std::endl;

    //init logging of quality estimate
    option1_quality_logg.open("/Users/tillkonradaust/Desktop/ProjectsXCode.nosync/argos3-test/data_cluster/env_17/op1_samples_" + std::to_string(initial_commitment) + "_1.txt", std::ios_base::trunc | std::ios_base::out);
    option2_quality_logg.open("/Users/tillkonradaust/Desktop/ProjectsXCode.nosync/argos3-test/data_cluster/env_17/op2_samples_" + std::to_string(initial_commitment) + "_1.txt", std::ios_base::trunc | std::ios_base::out);
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment is resetted.                                                  */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Reset() {
    // Close data file
    output_logg.close();
    option1_quality_logg.close();
    option2_quality_logg.close();

    // Reopen the file, erasing its contents
    output_logg.open(data_file_name, std::ios_base::trunc | std::ios_base::out);
    // write the log file header (it is not mendatory)
    output_logg << "time";
    for (unsigned int i = 0; i < logg_commitment_state.size(); i++) {
        output_logg << ";" << i;
    }
    output_logg << std::endl;

    option1_quality_logg.open("/Users/tillkonradaust/Desktop/ProjectsXCode.nosync/argos3-test/data_cluster/env_17/op1_samples_" + std::to_string(initial_commitment) + "_1.txt", std::ios_base::trunc | std::ios_base::out);
    option2_quality_logg.open("/Users/tillkonradaust/Desktop/ProjectsXCode.nosync/argos3-test/data_cluster/env_17/op2_samples_" + std::to_string(initial_commitment) + "_1.txt", std::ios_base::trunc | std::ios_base::out);
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment ended.                                                        */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Destroy() {
    output_logg.close();
    option1_quality_logg.close();
    option2_quality_logg.close();

}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called before every simulation step.                                                     */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PreStep() {
    // collect all the data from the robots - and virtualize them
    virtual_message_reception();

    // simulate IR message reception for each cell
    for (int x_it = 0; x_it < 10; x_it++) {
        for (int y_it = 0; y_it < 20; y_it++) {
            if (module_memory[x_it][y_it].robot_messages.size() > 0) {
                // IMPORTANT: cellid, distance and crc error are not used in simulation: here they are
                // dummy values
                IR_rx(x_it, y_it, module_memory[x_it][y_it].robot_messages[m_pcRNG->Uniform(
                              CRange<UInt32>(0, module_memory[x_it][y_it].robot_messages.size()))],
                      cell_id[0], d, 0);
                module_memory[x_it][y_it].robot_messages.clear();
            }
        }
    }

    // simulate reception of can messages
    for (int x_it = 0; x_it < 10; x_it++) {
        for (int y_it = 0; y_it < 20; y_it++) {
            // forward all messages, problem is, that it is the same order for all the nodes,
            //  leading to the problem that all nodes receive the same message as last and thus
            //  only forward this message to the Kilobots (result: all Kilobots receive the same
            //  message). That is why we need to random shuffle here!
            while (module_memory[x_it][y_it].received_cell_messages.size() > 0) {
                CAN_rx(x_it, y_it, &module_memory[x_it][y_it].received_cell_messages[m_pcRNG->Uniform(
                        CRange<UInt32>(0, module_memory[x_it][y_it].received_cell_messages.size()))]);
                module_memory[x_it][y_it].received_cell_messages.clear();
            }
        }
    }

    // run the loop
    for (int x_it = 0; x_it < 10; x_it++) {
        for (int y_it = 0; y_it < 20; y_it++) {
            loop(x_it, y_it);
        }
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called after every simulation step.                                                      */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PostStep() {
    // Save experiment data to the specified log file
    // check if quorum is reached
//    float avg_neighbours = 0;
    bool wrong_init = false;
    std::fill(logg_commitment_state.begin(), logg_commitment_state.end(), 0);
    for (unsigned int i = 0; i < kilobot_entities.size(); i++) {
        // reset if not initialised right
        if (((unsigned int) debug_info_kilobots[i]->commitement) == 20) {
            wrong_init = true;
            break;
        } else {  // track commitment
            logg_commitment_state[((unsigned int) debug_info_kilobots[i]->commitement)]++;
        }
        // track avg number of neighbours
        //  ATTENTION: only applies to static communication range
        //  ATTENTION: also breaks for N != 50
//        for (unsigned int j = 0; j < kilobot_entities.size(); j++) {
//            // dont count yourself
//            if (j != i) {
//                // in communication range
//                if (sqrt(pow(fabs(debug_info_kilobots[i]->x_pos - debug_info_kilobots[j]->x_pos),
//                             2) +
//                         pow(fabs(debug_info_kilobots[i]->y_pos - debug_info_kilobots[j]->y_pos),
//                             2)) < initial_communication_range) {
//                    avg_neighbours += 1;
//                }
//            }
//        }
//        // track quality estimates of the robots
//        // discovered new quality
//        if (((unsigned int) debug_info_kilobots[i]->quality_flag) == 1) {
//            if (((unsigned int) debug_info_kilobots[i]->quality_option) == 1){ // option 1
//                option1_quality_logg << debug_info_kilobots[i]->quality_quality << std::endl;
//            }else if (((unsigned int) debug_info_kilobots[i]->quality_option) == 2) { // option 2
//                option2_quality_logg << debug_info_kilobots[i]->quality_quality << std::endl;
//            }
//        }
    }

    // time to write something down, max time passed
    if ((data_saving_counter % DATA_SAVING_FREQUENCY == 0) ||
        (GetSpace().GetSimulationClock() >= GetSimulator().GetMaxSimulationClock())) {
        output_logg << GetSpace().GetSimulationClock();
        for (unsigned int i = 0; i < logg_commitment_state.size(); i++) {
            output_logg << ";" << logg_commitment_state[i];
        }
//        output_logg << ";" << avg_neighbours/50;
        output_logg << std::endl;
    }

    // reset init flag if not every robot is inited -> robots cannot get uncommitted
    if (wrong_init) {
        printf("[LOOPFUNCTION] not all robots are inited; repeat... \n");
        GetSimulator().Reset();
        for (int x_it = 0; x_it < 10; x_it++) {
            for (int y_it = 0; y_it < 20; y_it++) {
                module_memory[x_it][y_it].init_flag = false;

            }
        }
    }

    /// for viz -> that i can see the progression
    if (GetSpace().GetSimulationClock() % 1000 == 0) {
        printf("[LOOPFUNCTION] Clock at %d ... \n", GetSpace().GetSimulationClock());
//        for (int nicht_till = 0; nicht_till < 101;nicht_till++){
//            printf("%d ", sample[nicht_till][0]);
//        }
//        printf("\n");
//        for (int nicht_till = 0; nicht_till < 101;nicht_till++){
//            printf("%d ", sample[nicht_till][1]);
//        }
//        printf("\n");
    }
    /// print distribution of sampling
//    if (GetSpace().GetSimulationClock() == 76799){
//        for (int nicht_till = 0; nicht_till < 101;nicht_till++){
//            printf("%d ", sample[nicht_till][0]);
//        }
//        printf("\n");
//        for (int nicht_till = 0; nicht_till < 101;nicht_till++){
//            printf("%d ", sample[nicht_till][1]);
//        }
//        printf("\n");
//    }
//    LOG << counter_q << std::endl;

    // quit simulation if quorum reached
//    if(m_bQuorumReached==true){
//        printf("[LOOPFUNCTION] reached quorum \n");
//        GetSimulator().Terminate();
//    }

    data_saving_counter++;

    // TODO change to desired value! -> how to implement in the kilogrid probably saving multiple
    //  during the init phase
    // change after 20 min = 38400 (20*60*1000/(1000/32)) = min*60*32
    // change after 10 min = 19200 (20*60*32) = min*60*32
//    if (GetSpace().GetSimulationClock() == 19200) {
//        printf("changing environment \n");
//        change_environment();
//        GetSpace().GetFloorEntity().SetChanged();
//    }


    // debug section
//    printf("Distribution ");
//    for(unsigned int i=0;i< logg_commitment_state.size();i++){
//        printf("%d ", logg_commitment_state[i]);
//    }
//    printf("inactive robots %d \n", inactive_robots);
//    printf("\n%d------------------------------------------------------------------------------\n", GetSpace().GetSimulationClock());
}


/*-----------------------------------------------------------------------------------------------*/
/* For changing the environment .. very experimental!                                            */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::change_environment() {
    // clear configuration first bc the clown I am made it as list and pushes to back
    for(int x_it = 0; x_it < 10; x_it++){
        for(int y_it = 0; y_it < 20; y_it++){
            configuration[x_it][y_it].clear();
        }
    }

    // variables used for reading
    int tmp_x_module;
    int tmp_y_module;
    int tmp_value;
    int read_mode = 0; // 1 is address, 2 is data
    std::string tmp_str;

    // reading new config
    input.open(second_config_file_name);
    for (std::string line; getline(input, line);) {
        // for each line
        if (!line.empty() && line[0] != '#') {  // exclude comments and empty lines

            //printf("%s\n",line.c_str()); // <- this way you can pring the line out!
            if (line == "address") {
                read_mode = 1;
            } else if (line == "data") {
                read_mode = 2;
            } else {
                // reading address
                if (read_mode == 1) {
                    // isolating numbers
                    tmp_str = line;
                    tmp_str = tmp_str.substr(7);  // remove module
                    std::string::size_type p = tmp_str.find('-');
                    tmp_y_module = std::stoi(tmp_str.substr(p + 1));
                    tmp_x_module = std::stoi(tmp_str);
                } else if (read_mode == 2) { // read data
                    tmp_value = std::stoi(line, 0, 16);
                    configuration[tmp_x_module][tmp_y_module].push_back(tmp_value);
                }
            }
        }
    }
    input.close();

    // setting new config
    for (int x_it = 0; x_it < 10; x_it++) {
        for (int y_it = 0; y_it < 20; y_it++) {
            setup(x_it, y_it);
        }
    }

}

/*-----------------------------------------------------------------------------------------------*/
/* This function reads the config file for the kilogrid and saves the content to configuration.  */
/* Further it reads other configurations from the argos file such as datafilename...             */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::read_configuration(TConfigurationNode &t_node) {
    // getting config script name from .argos file
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_node, "variables");
    GetNodeAttribute(tExperimentVariablesNode, "config_file", config_file_name);
    GetNodeAttribute(tExperimentVariablesNode, "second_config", second_config_file_name);
    GetNodeAttribute(tExperimentVariablesNode, "data_file", data_file_name);
    GetNodeAttribute(tExperimentVariablesNode, "initial_commitment", initial_commitment);
    GetNodeAttribute(tExperimentVariablesNode, "initial_commitment_quality", initial_commitment_quality);
    GetNodeAttribute(tExperimentVariablesNode, "number_of_options", number_of_options);
    GetNodeAttribute(tExperimentVariablesNode, "initial_communication_range",
                     initial_communication_range);
    GetNodeAttribute(tExperimentVariablesNode, "max_communication_range", max_communication_range);


    // variables used for reading
    int tmp_x_module;
    int tmp_y_module;
    int tmp_value;
    int read_mode = 0; // 1 is address, 2 is data
    std::string tmp_str;

    // reading
    input.open(config_file_name);
    for (std::string line; getline(input, line);) {
        // for each line
        if (!line.empty() && line[0] != '#') {  // exclude comments and empty lines

            //printf("%s\n",line.c_str()); // <- this way you can pring the line out!
            if (line == "address") {
                read_mode = 1;
            } else if (line == "data") {
                read_mode = 2;
            } else {
                // reading address
                if (read_mode == 1) {
                    // isolating numbers
                    tmp_str = line;
                    tmp_str = tmp_str.substr(7);  // remove module
                    std::string::size_type p = tmp_str.find('-');
                    tmp_y_module = std::stoi(tmp_str.substr(p + 1));
                    tmp_x_module = std::stoi(tmp_str);
                } else if (read_mode == 2) { // read data
                    tmp_value = std::stoi(line, 0, 16);
                    configuration[tmp_x_module][tmp_y_module].push_back(tmp_value);
                }
            }
        }
    }
    input.close();
}


void CKilogrid::set_LED_with_brightness(int x, int y, cell_num_t cn, color_t color,
                                        brightness_t brightness) {
    module_memory[x][y].cell_colour[cn] = color;
//    GetSpace().GetFloorEntity().SetChanged();
}


/*-----------------------------------------------------------------------------------------------*/
/* Visualization of the floor color.                                                             */
/*-----------------------------------------------------------------------------------------------*/
CColor CKilogrid::GetFloorColor(const CVector2 &vec_position_on_plane) {
    CColor cColor = CColor::WHITE;
    int id = position2option(vec_position_on_plane);

    // checks if the option is in time
    if (id == 0) {
        // this is wall
        cColor = CColor::WHITE;
    } else if (id == 1) {
        cColor = CColor::RED;
    } else if (id == 2) {
        cColor = CColor::GREEN;
    } else if (id == 3) {
        cColor = CColor::BLUE;
    } else if (id == 4) {
        cColor = CColor::BROWN;
    } else if (id == 5) {
        cColor = CColor::BLACK;
    }

    return cColor;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the Option of the given position.                                                     */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CKilogrid::position2option(CVector2 t_position) {
    int module_x = t_position.GetX() * 10;
    int module_y = t_position.GetY() * 10;

    uint8_t cell = 0;
    int x = t_position.GetX() * 20;
    int y = t_position.GetY() * 20;

    if (x >= 20) x = 19;
    if (y >= 40) y = 39;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    // get the cell
    if (x % 2 == 0 && y % 2 == 1) {
        cell = 0;
    } else if (x % 2 == 1 && y % 2 == 1) {
        cell = 1;
    } else if (x % 2 == 0 && y % 2 == 0) {
        cell = 2;
    } else if (x % 2 == 1 && y % 2 == 0) {
        cell = 3;
    }

    return module_memory[module_x][module_y].cell_colour[cell];
}

CVector2 CKilogrid::position2cell(CVector2 t_position) {
    int x = t_position.GetX() * 20;
    int y = t_position.GetY() * 20;

    if (x >= 20) x = 19;
    if (y >= 40) y = 39;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    return CVector2(x, y);
}


CVector2 CKilogrid::position2module(CVector2 t_position) {
    float x = t_position.GetX() * 10;
    float y = t_position.GetY() * 10;

    if (x >= 10) x = 9;
    if (y >= 20) y = 19;
    if (x < 0) x = 0;
    if (y < 0) y = 0;

    return CVector2(int(x), int(y));
}

CVector2 CKilogrid::module2cell(CVector2 module_pos, cell_num_t cn) {
    int cur_x = module_pos.GetX() * 2;
    int cur_y = module_pos.GetY() * 2;

    switch (cn) {
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

void CKilogrid::get_kilobots_entities() {
    // Get the map of all kilobots from the space
    CSpace::TMapPerType &mapKilobots = GetSpace().GetEntitiesByType("kilobot");
    // Go through them
    for (CSpace::TMapPerType::iterator it = mapKilobots.begin(); it != mapKilobots.end(); ++it) {
        kilobot_entities.push_back(any_cast<CKilobotEntity *>(it->second));
    }
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the position of the kilobot.                                                          */
/*-----------------------------------------------------------------------------------------------*/
CVector2 CKilogrid::GetKilobotPosition(CKilobotEntity &c_kilobot_entity) {
    CVector2 vecKP(c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                   c_kilobot_entity.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    return vecKP;
}


/*-----------------------------------------------------------------------------------------------*/
/* Returns the id of the kilobot.                                                                */
/*-----------------------------------------------------------------------------------------------*/
UInt16 CKilogrid::GetKilobotId(CKilobotEntity &c_kilobot_entity) {
    std::string strKilobotID((c_kilobot_entity).GetControllableEntity().GetController().GetId());
    return std::stoul(strKilobotID.substr(2));
}

void CKilogrid::virtual_message_reception() {
    // for debug
//    int msg_op[3] = {0, 0, 0};
    // get position information, later used for sending stuff!!
    for (UInt16 it = 0; it < kilobot_entities.size(); it++) {
        robot_positions[GetKilobotId(*kilobot_entities[it])] = position2cell(
                GetKilobotPosition(*kilobot_entities[it]));
    }

    for (UInt16 it = 0; it < debug_info_kilobots.size(); it++) {
        if (debug_info_kilobots[it]->broadcast_flag == 1) {
            int module_x = int(robot_positions[GetKilobotId(*kilobot_entities[it])].GetX()) / 2;
            int module_y = int(robot_positions[GetKilobotId(*kilobot_entities[it])].GetY()) / 2;
            IR_message_t *tmp_msg = new IR_message_t;
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
    CVector2 cell_pos = module2cell(CVector2(x, y), cn);

    // get kilobots on this cell
    kilobot_entities_vector current_robots;

    for (uint16_t it = 0; it < kilobot_entities.size(); it++) {
        if (robot_positions[GetKilobotId(*kilobot_entities[it])].GetX() == cell_pos.GetX()
            and robot_positions[GetKilobotId(*kilobot_entities[it])].GetY() == cell_pos.GetY()) {
            current_robots.push_back(kilobot_entities[it]);
        }
    }

    // send to all kilobots on this cell
    for (UInt16 it = 0; it < current_robots.size(); it++) {
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
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(
                *current_robots[it], &message_to_send);
    }
}

/*-----------------------------------------------------------------------------------------------*/
/* Set up the getting of debug information from the robot (e.g. the robots state).               */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::GetDebugInfo() {
    debug_info_kilobots.clear();
    for (UInt16 it = 0; it < kilobot_entities.size(); it++) {
        // Check if there is a message to send to the kilobot "i" and get the message

        /*
         * When the 'reset' method is called on the kilobot controller, the
         * kilobot state is destroyed and recreated. Thus, we need to
         * recreate the list of controllers and debugging info from scratch
         * as well.
         */
        // Create a pointer to the current kilobot
        CCI_KilobotController *pcKBC = &dynamic_cast<CCI_KilobotController &>(kilobot_entities[it]->GetControllableEntity().GetController());
        // Create debug info for controller
        debug_info_t *ptDebugInfo = pcKBC->DebugInfoCreate<debug_info_t>();
        // Append to list
        debug_info_kilobots.push_back(ptDebugInfo);
    }
}

void CKilogrid::init_CAN_message(CAN_message_t *cell_msg) {
    uint8_t d;

    cell_msg->id = 0;
    cell_msg->header.rtr = 0;
    cell_msg->header.length = 8;

    for (d = 0; d < 8; d++) cell_msg->data[d] = 0;
}


uint8_t CKilogrid::CAN_message_tx(CAN_message_t *m, kilogrid_address_t add) {
    // this is a hack because we can only virtually set msgs
    // TODO is this message coppied or just the pointer

    if (add.type == ADDR_INDIVIDUAL) {
        module_memory[add.x][add.y].received_cell_messages.push_back(*m);
    }else if (add.type == ADDR_BROADCAST_TO_MODULE) {
        for (int can_tx_it_x = 0; can_tx_it_x < 10; can_tx_it_x++) {
            for (int can_tx_it_y = 0; can_tx_it_y < 20; can_tx_it_y++) {
                module_memory[can_tx_it_x][can_tx_it_y].received_cell_messages.push_back(*m);
            }
        }
    }

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
void CKilogrid::setup(int x, int y) {
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
    module_memory[x][y].cell_role[1] = (configuration[x][y][3]);
    module_memory[x][y].cell_role[2] = (configuration[x][y][4]);
    module_memory[x][y].cell_role[3] = (configuration[x][y][5]);

    module_memory[x][y].cell_colour[0] = color_t(configuration[x][y][6]);
    module_memory[x][y].cell_colour[1] = color_t(configuration[x][y][7]);
    module_memory[x][y].cell_colour[2] = color_t(configuration[x][y][8]);
    module_memory[x][y].cell_colour[3] = color_t(configuration[x][y][9]);

    module_memory[x][y].status_msg_counter = MIN_TIME_BETWEEN_MSG;

    // set id of received can message to 0 in order to initialize it -> maybe only needed in sim because
    // in reality we do not have the issue bc its a callback
    // module_memory[x][y].received_cell_message->id = 0;

    for (int i = 0; i < 4; i++) {
//        if (module_memory[x][y].cell_role[i] == 0) {
//            set_LED_with_brightness(x, y, cell_id[i], WHITE, HIGH);
//        }
//        else if(module_memory[x][y].cell_role[i] == 21) {
//            set_LED_with_brightness(x, y, cell_id[i], RED, HIGH);
//        }
//        else if (module_memory[x][y].cell_role[i] == 42) {
//            set_LED_with_brightness(x, y, cell_id[i], GREEN, HIGH);
//        }
        set_LED_with_brightness(x, y, cell_id[i], module_memory[x][y].cell_colour[i], HIGH);

    }
}


// this method implements the loop function
void CKilogrid::loop(int x, int y) {
    // send initial message
    if (!module_memory[x][y].init_flag) {
        module_memory[x][y].init_flag = true;
        for (int f = 0; f < 4; f++) {
            module_memory[x][y].ir_message_tx->type = INIT_MSG;
            module_memory[x][y].ir_message_tx->crc = 0;
            module_memory[x][y].ir_message_tx->data[0] = module_memory[x][y].cell_x[f];
            module_memory[x][y].ir_message_tx->data[1] = module_memory[x][y].cell_y[f];
            module_memory[x][y].ir_message_tx->data[2] = initial_commitment; // TODO adjustable for each robot
            module_memory[x][y].ir_message_tx->data[3] = initial_commitment_quality;
            module_memory[x][y].ir_message_tx->data[4] = number_of_options;
            module_memory[x][y].ir_message_tx->data[5] = module_memory[x][y].cell_colour[f];  // tested cast to unit_8 not needed
            module_memory[x][y].ir_message_tx->data[6] = initial_communication_range;
            module_memory[x][y].ir_message_tx->data[7] = max_communication_range;

            set_IR_message(x, y, *module_memory[x][y].ir_message_tx, cell_id[f]);
        }
        return;
    }
    // normal loop
    int c;
    // processing ir messages
    if (module_memory[x][y].received_ir) {
        module_memory[x][y].received_ir = false;

        for (c = 0; c < 4; c++) {
            // msg structure: robot_gps_x, robot_gps_y, robot_commitment, communication_range, msg_number_send, kilo_uid
            if (module_memory[x][y].tmp_ir_data[c][2] != 0){  // commitment != 0
                module_memory[x][y].received_x[c] = module_memory[x][y].tmp_ir_data[c][0];
                module_memory[x][y].received_y[c] = module_memory[x][y].tmp_ir_data[c][1];
                module_memory[x][y].received_commitment[c] = module_memory[x][y].tmp_ir_data[c][2];
                module_memory[x][y].received_communication_range[c] = module_memory[x][y].tmp_ir_data[c][3];
                module_memory[x][y].received_kilo_uid[c] = module_memory[x][y].tmp_ir_data[c][5] + module_memory[x][y].tmp_ir_data[c][6];  // TODO: does this work
                // forward robot msg
                module_memory[x][y].can_msg_to_send = true;
            }
            // clear
            for(uint8_t  k_it = 0; k_it < 4; k_it++){
                module_memory[x][y].tmp_ir_data[c][k_it] = 0;
            }
        }
    }
    // send can message if needed...
    if (module_memory[x][y].can_msg_to_send) {
        module_memory[x][y].can_msg_to_send = false;
        for (c = 0; c < 4; c++) {
            // send for every cell
            if (module_memory[x][y].received_commitment[c] != 0) {
                CAN_message_t tmp_can_msg;
                init_CAN_message(&tmp_can_msg);

                module_memory[x][y].cell_address.type = ADDR_BROADCAST_TO_MODULE;
                module_memory[x][y].cell_address.x = 0;  // is the position of a module imo??
                module_memory[x][y].cell_address.y = 0;

                tmp_can_msg.data[0] = CAN_USER_MESSAGE;  // type of user message
                tmp_can_msg.data[1] = module_memory[x][y].received_x[c]; // x sender
                tmp_can_msg.data[2] = module_memory[x][y].received_y[c]; // y sender
                tmp_can_msg.data[3] = module_memory[x][y].received_communication_range[c]; // range
                tmp_can_msg.data[4] = module_memory[x][y].received_commitment[c]; // information
                uint8_t tmp_kilo_uid_1;
                uint8_t tmp_kilo_uid_2;
                if (module_memory[x][y].received_kilo_uid[c] > 255){
                    tmp_kilo_uid_1 = 255;
                    tmp_kilo_uid_2 = module_memory[x][y].received_kilo_uid[c] - 255;
                } else {
                    tmp_kilo_uid_1 = module_memory[x][y].received_kilo_uid[c];
                    tmp_kilo_uid_2 = 0;
                }
                tmp_can_msg.data[5] = tmp_kilo_uid_1;
                tmp_can_msg.data[6] = tmp_kilo_uid_2;
                tmp_can_msg.data[7] = 0;

                CAN_message_tx(&tmp_can_msg, module_memory[x][y].cell_address);

                // reset info
                module_memory[x][y].received_commitment[c] = 0;
            }
        }
    }
    // process can messages
    if (module_memory[x][y].received_can) {
        module_memory[x][y].received_can = false;
        // if cell is in range act
        for(c = 0; c < 4; c++) {
            if (sqrt(pow(fabs(module_memory[x][y].tmp_can_data[1] - module_memory[x][y].cell_x[c]), 2) + pow(fabs(module_memory[x][y].tmp_can_data[2] - module_memory[x][y].cell_y[c]), 2)) < module_memory[x][y].tmp_can_data[3]) {
                module_memory[x][y].ir_msg_to_send[c] = true;
                module_memory[x][y].opt_to_send_ir[c] = module_memory[x][y].tmp_can_data[4];
                module_memory[x][y].received_can_kilo_uid[c] = module_memory[x][y].tmp_can_data[5] + module_memory[x][y].tmp_can_data[6];
            } else{
                module_memory[x][y].opt_to_send_ir[c] = module_memory[x][y].cell_colour[c];  // this should be nothing, only called for cells when receiving a can message, and it is not in range

            }
        }
    }

    // send ir message .. either forward message (if there is one) or send status
    for (int c = 0; c < 4; c++) {
        // send messages - forward before sending status
        if (module_memory[x][y].ir_msg_to_send[c]) {
            module_memory[x][y].ir_msg_to_send[c] = false;
            // module_memory[x][y].can_kilo_uid = module_memory[x][y].received_can_kilo_uid[c];

            module_memory[x][y].ir_message_tx->type = VIRTUAL_AGENT_MSG;
            module_memory[x][y].ir_message_tx->crc = 0;
            module_memory[x][y].ir_message_tx->data[0] = module_memory[x][y].cell_x[c];
            module_memory[x][y].ir_message_tx->data[1] = module_memory[x][y].cell_y[c];
            module_memory[x][y].ir_message_tx->data[2] = module_memory[x][y].opt_to_send_ir[c];
            uint8_t tmp_kilo_uid_1;
            uint8_t tmp_kilo_uid_2;
            if (module_memory[x][y].received_can_kilo_uid[c] > 255){
                tmp_kilo_uid_1 = 255;
                tmp_kilo_uid_2 = module_memory[x][y].received_kilo_uid[c] - 255;
            } else {
                tmp_kilo_uid_1 = module_memory[x][y].received_kilo_uid[c];
                tmp_kilo_uid_2 = 0;
            }
            module_memory[x][y].ir_message_tx->data[3] = tmp_kilo_uid_1;
            module_memory[x][y].ir_message_tx->data[4] = tmp_kilo_uid_2;
            set_IR_message(x, y, *module_memory[x][y].ir_message_tx, cell_id[c]);
        } else {
            // sending status
            if (module_memory[x][y].status_msg_counter >= MIN_TIME_BETWEEN_MSG) {
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
    }

    // increase counter
    module_memory[x][y].status_msg_counter += 1;
}


void CKilogrid::IR_rx(int x, int y, IR_message_t *m, cell_num_t c, distance_measurement_t *d,
                      uint8_t CRC_error) {
    if (!CRC_error && m->type == MSG_T_VIRTUAL_ROBOT_MSG) {
        // check which cell -> normally handled by the kilogrid software
        uint8_t c_tmp = 0;
        for (int c_ir_it = 0; c_ir_it < 4; c_ir_it++) {
            if (m->data[0] == module_memory[x][y].cell_x[c_ir_it] && m->data[1] == module_memory[x][y].cell_y[c_ir_it]) {
                c_tmp = c_ir_it;
            }
        }

        // message from robot to kilogrid: broadcast
        for (uint8_t tmp_it = 0; tmp_it < 8; tmp_it++){
            module_memory[x][y].tmp_ir_data[c_tmp][tmp_it] = m->data[tmp_it];
        }
        module_memory[x][y].received_ir = true;
#ifndef SIMULATION
        }else if(!CRC_error && m->type == TRACKING){ // here we have to write down the logging
            CAN_message_t *msg = next_CAN_message();
            if (msg != NULL) { // if the buffer is not full
                serialize_tracking_message(msg, c, m->data);
            }
#endif
    } else {
        printf("ERROR: unknown IR message type at module %d %d \n", x, y);
    }
}

// just temporary store the data, because it is a callback
void CKilogrid::CAN_rx(int x, int y, CAN_message_t *m) {
    if (m->data[0] == CAN_USER_MESSAGE) {  // set msg
        for(uint8_t cell_can_it = 0; cell_can_it < 4; cell_can_it++){
            if (sqrt(pow(fabs(m->data[1] - module_memory[x][y].cell_x[cell_can_it]), 2) + pow(fabs(m->data[2] - module_memory[x][y].cell_y[cell_can_it]), 2)) < m->data[3]) {
                for (uint8_t tmp_it = 0; tmp_it < 8; tmp_it++) {
                    module_memory[x][y].tmp_can_data[tmp_it] = m->data[tmp_it];
                }
                module_memory[x][y].received_can = true;
                return;
            }
        }
    } else {
        printf("ERROR: unknown CAN message type at module %d %d \n", x, y);
    }
}

// not needed for the real kilogrid
REGISTER_LOOP_FUNCTIONS(CKilogrid, "kilogrid_loop_functions")