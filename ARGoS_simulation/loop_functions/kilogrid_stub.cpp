//
// Created by Till Konrad Aust on 17.12.21.
//

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
// This method should initalize all the robot enteties in order to work with them
// as you would with the real kilogrid
// what else ?!?
void CKilogrid::Init(TConfigurationNode &t_tree) {
    // get all kilobots

    // init map
    read_configuration(t_tree);
    for(int x_it = 0; x_it < 10; x_it++){
        for(int y_it = 0; y_it < 20; y_it++){
            setup(x_it, y_it);
        }
    }

}

/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment is resetted.                                                  */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Reset() {
    // TODO implement
}

/*-----------------------------------------------------------------------------------------------*/
/* Gets called when the experiment ended.                                                        */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::Destroy() {
    // TODO implement
}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called before every simulation step.                                                     */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PreStep(){
    // TODO implement

    // implement callback - collect all the data

    // run the loop
    for(int x_it = 0; x_it < 10; x_it++){
        for(int y_it = 0; y_it < 20; y_it++){
            loop(x_it, y_it);
        }
    }

    // forward all messages

}


/*-----------------------------------------------------------------------------------------------*/
/* Gets called after every simulation step.                                                      */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::PostStep(){
    // TODO implement

    // send messages

    // clear all
}


/*-----------------------------------------------------------------------------------------------*/
/* This function reads the config file for the kilogrid and saves the content to configuration.  */
/*-----------------------------------------------------------------------------------------------*/
void CKilogrid::read_configuration(TConfigurationNode& t_node){
    // getting config script name from .argos file
    TConfigurationNode& tExperimentVariablesNode = GetNode(t_node,"variables");
    GetNodeAttribute(tExperimentVariablesNode, "config_file", config_file_name);

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

    module_memory[x][y].cell_colour[0] = (configuration[x][y][3]);
    module_memory[x][y].cell_colour[1] = (configuration[x][y][4]);
    module_memory[x][y].cell_colour[2] = (configuration[x][y][5]);
    module_memory[x][y].cell_colour[3] = (configuration[x][y][6]);

    module_memory[x][y].test_counter = 0;

    for(int i = 0; i < 4; i++){
        set_LED_with_brightness(x, y, cell_id[i], (color_t)(module_memory[x][y].cell_colour[i]), HIGH);
    }
}


// this method implements the loop function
void CKilogrid::loop(int x, int y){
    module_memory[x][y].test_counter += 1;

    if(module_memory[x][y].test_counter > 20){
        module_memory[x][y].test_counter = 0;
        for (int c = 0; c < 4; c++){
            int tmp_int = (module_memory[x][y].cell_colour[c]+1)%4;
            set_LED_with_brightness(x, y, cell_id[c], (color_t)(tmp_int), HIGH);
        }
    }
}

void CKilogrid::set_LED_with_brightness(int x, int y, cell_num_t cn, color_t color, brightness_t brightness){
    module_memory[x][y].cell_colour[cn] = int(color);
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
    uint8_t cell = position2cell(t_position);

    return module_memory[module_x][module_y].cell_colour[cell];
}

uint8_t CKilogrid::position2cell(CVector2 t_position){
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

    return cell;
}



// this method implements the IR_rx method for all models
//void virtual_IR_rx(){
//    // TODO adjust
//    // module_memory
//}

// ??
REGISTER_LOOP_FUNCTIONS(CKilogrid, "kilogrid_loop_functions")