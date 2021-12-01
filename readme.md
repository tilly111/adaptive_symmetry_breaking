# Adaptive Symmetry Breaking 

## How to build the code
```
mkdir build
cd build
cmake ../ARGoS_simulation
make 
```
This should build it. TODO  

## How to run experiments

### local

```
cd argos3-test
sh ARGoS_simulation/data_generation_scripts/<experiment-to-run>.sh <start> <end> <model_type> <visualization> <number_of_options> <on_cluster>
```

A list of parameters to configure your experiment:  

|Name               |Values |Description|
|---                |---    |---        |
|`start`            |int number|Start-index of first experiment.|
|`end`              |int number|End-index of last experiment.|
|`model_type`       |local, global, adaptive|Communication type.|
|`visualization`    |0,1|Visualization flag (Visualization=1)|
|`number_of_options`|3,5|Number of different options.|
|`on_cluster`       |0,1|Flag for running on the cluster (Cluster=1)|

This runs the experiments local. (You have to proper build it first!)
An example run, e.g., to test the behavior is:

```
sh ARGoS_simulation/data_generation_scripts/runexps_kilogird.sh 0 0 local 1 3 0
```

### Run experiments on the cluster 
First you need to load modules/source everything.

```
module load cmake 
source Programs/argos3/set_argos_env.sh
```

Next you can pull the current version of this repo

```
git pull
```

Then you need to renew the link of argos3-kilobot plugin (first remove it)

```
rm argos3
ln -s ~/Programs/argos3-kilobot/src/ ARGoS_simulation/argos3
```

now you can go build the project

```
cd build
cmake ../ARGoS_simulation
make
cd ..
```

Now you can go into the scripts folder

```
cd ARGoS_simulation/data_generation_scripts/
```

There first you have to run the create argos file in order to generate the experiments 

```
sh cluster_create_argos_files.sh  0 99 adaptive 3
```

Afterwards you can submit the jobs.

```
sh runjobs.sh
```

The results can be found in the data cluster folder.

## TODO 
Installation instruction for 
- installation on cluster ?
- installation on new cluster ?
- installation of sidepkg aka lua5.3 mein erzfeind

## Installation argos3 & Kilobot-plugin on Newmajorana
### argos3
Load required tools.

```
module load cmake 
```

gcc is already installed, we skip lua because I do not have the permission to install it ~ we have various problems with it.
If not already done create Programs folder 

```
mkdir Programs 
cd Programs 
```

Now we get argos3.

```
git clone https://github.com/ilpincy/argos3
cd argos3
```

I moved to a older version (which works fine for me ~ the current version gave me some trouble).

```
git checkout 86f60d26b2ad148c0455a290cd6bd6fa2109172d
```

Now create build folder.

```
mkdir build_simulator
cd build_simulator
```

Use cmake.

```
cmake -DCMAKE_INSTALL_PREFIX=$HOME/Programs/argos3/install -DCMAKE_BUILD_TYPE=Release -DARGOS_INSTALL_LDSOCONF=OFF -DARGOS_DOCUMENTATION=OFF ../src
make -j4
make install
```

Now you need to create a env file for argos3. 
I did it in the argos3/folder

```
cd argos3/
nano set_argos_env.sh
```
The content should be the following (YOU HAVE TO CHANGE PATH TO YOUR USERNAME).
```
export PKG_CONFIG_PATH=/home/taust/Programs/argos3/install/lib/pkgconfig  
export ARGOS_PLUGIN_PATH=/home/taust/Programs/argos3/install/lib/argos3  
export LD_LIBRARY_PATH=$ARGOS_PLUGIN_PATH:$LD_LIBRARY_PATH  
export C_INCLUDE_PATH=/home/taust/Programs/argos3/install/include:/home/taust/Programs/argos3/install/include:  
export CPLUS_INCLUDE_PATH=/home/taust/Programs/argos3/install/include:/home/taust/Programs/argos3/install/include:  
export PATH=/home/taust/Programs/argos3/install/bin:$PATH
```

Now you can source argos3.

```
source set_argos_env.sh
```

The following command should now return version 3.0.0-beta56.

```
argos3 -v
```

### Kilobot-Plugin
First you have to source and load modules, if not already done 

```
module load cmake
source set_argos_env
```

Now we can download the plugin.

```
cd Programs
git clone https://github.com/ilpincy/argos3-kilobot.git
cd argos3-kilobot
```

Now we have to renew the link (first remove it).

```
cd src/
rm argos3
cd ..
ln -s ~/Programs/argos3-kilobot/src/ src/argos3
```

In the next step we adjust some CMakeFiles to get rid of runtime errors and stuff. Further you have to remove all dependencies for lua because we are missing lua-devel. 
The following should be sufficient. 
Replace the content of ARGoSBuildChecks.cmake:

```
nano src/cmake/ARGoSBuildChecks.cmake
```

with 

```
#
# Find the ARGoS package
#
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
set(CMAKE_INSTALL_PREFIX ${ARGOS_PREFIX} CACHE STRING "Install path prefix, prepended onto install directories." FORCE)

#
# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
#
include(ARGoSCheckQTOpenGL)

#
# Find Lua
#
#find_package(Lua53 REQUIRED)

#
# Look for librt, necessary on some platforms
#
find_package(RT)

#
# Set ARGoS include dir
#
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})

#
# Set ARGoS link dir
#
link_directories(${ARGOS_LIBRARY_DIRS})
```

Next we replace

```
rm src/plugins/robots/kilobot/CMakeLists.txt
nano src/plugins/robots/kilobot/CMakeLists.txt
```

with content

```
#
# kilobot headers
#
# argos3/plugins/robots/kilobot/control_interface
set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CONTROLINTERFACE
  control_interface/ci_kilobot_communication_actuator.h
  control_interface/ci_kilobot_communication_sensor.h
  control_interface/ci_kilobot_controller.h
  control_interface/ci_kilobot_led_actuator.h
  control_interface/ci_kilobot_light_sensor.h
  control_interface/kilolib.h
  control_interface/debug.h
  control_interface/message.h
  control_interface/message_crc.h)
# argos3/plugins/robots/kilobot/simulator
if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR
    simulator/dynamics2d_kilobot_model.h
    simulator/pointmass3d_kilobot_model.h
    simulator/kilobot_entity.h
    simulator/kilobot_measures.h
    simulator/kilobot_led_default_actuator.h
    simulator/kilobot_light_rotzonly_sensor.h
    simulator/kilobot_communication_default_actuator.h
    simulator/kilobot_communication_default_sensor.h
    simulator/kilobot_communication_entity.h
    simulator/kilobot_communication_medium.h)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# kilobot sources
#
set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
  ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CONTROLINTERFACE}
  control_interface/ci_kilobot_communication_actuator.cpp
  control_interface/ci_kilobot_communication_sensor.cpp
  control_interface/ci_kilobot_controller.cpp
  control_interface/ci_kilobot_led_actuator.cpp
  control_interface/ci_kilobot_light_sensor.cpp)

if(ARGOS_BUILD_FOR_SIMULATOR)
  set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
    ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT}
    ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}
    simulator/dynamics2d_kilobot_model.cpp
    simulator/pointmass3d_kilobot_model.cpp
    simulator/kilobot_entity.cpp
    simulator/kilobot_led_default_actuator.cpp
    simulator/kilobot_light_rotzonly_sensor.cpp
    simulator/kilobot_communication_default_actuator.cpp
    simulator/kilobot_communication_default_sensor.cpp
    simulator/kilobot_communication_entity.cpp
    simulator/kilobot_communication_medium.cpp)
  # Compile the graphical visualization only if the necessary libraries have been found
  if(ARGOS_QTOPENGL_FOUND)
    include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
    set(ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR
      ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}
      simulator/qtopengl_kilobot.h)
    set(ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT
      ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT}
      simulator/qtopengl_kilobot.h
      simulator/qtopengl_kilobot.cpp)
  endif(ARGOS_QTOPENGL_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Create kilobot plugin
#
add_library(argos3plugin_${ARGOS_BUILD_FOR}_kilobot SHARED ${ARGOS3_SOURCES_PLUGINS_ROBOTS_KILOBOT})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_kilobot
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_dynamics2d
  argos3plugin_${ARGOS_BUILD_FOR}_pointmass3d)
if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_kilobot argos3plugin_${ARGOS_BUILD_FOR}_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
if(RT_FOUND)
  target_link_libraries(argos3plugin_simulator_kilobot ${RT_LIBRARY})
endif(RT_FOUND)

#
# Create kilolib
#
if(ARGOS_BUILD_FOR_SIMULATOR)
  add_library(argos3plugin_simulator_kilolib
    control_interface/kilolib.c
    control_interface/message_crc.c)
  if(RT_FOUND)
    target_link_libraries(argos3plugin_simulator_kilolib ${RT_LIBRARY})
  endif(RT_FOUND)
endif(ARGOS_BUILD_FOR_SIMULATOR)

#
# Installation
#
install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_CONTROLINTERFACE} DESTINATION include/argos3/plugins/robots/kilobot/control_interface)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(FILES ${ARGOS3_HEADERS_PLUGINS_ROBOTS_KILOBOT_SIMULATOR}      DESTINATION include/argos3/plugins/robots/kilobot/simulator)
endif(ARGOS_BUILD_FOR_SIMULATOR)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_kilobot
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)

if(ARGOS_BUILD_FOR_SIMULATOR)
  install(TARGETS argos3plugin_simulator_kilolib
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib/argos3
    ARCHIVE DESTINATION lib/argos3)
endif(ARGOS_BUILD_FOR_SIMULATOR)
```

Now we are ready to build.
Therefore go to argos3-kilobot/

```
mkdir build
cd build
```

Now cmake (some of this options are not required - they throw a warning but you can ignore it).

```
cmake -DCMAKE_INSTALL_PREFIX=$HOME/Programs/argos3/install -DCMAKE_BUILD_TYPE=Release -DARGOS_INCLUDE_DIR=$HOME/Programs/argos3/install/include -DARGOS_CORE_LIBRARY=$HOME/Programs/argos3/install/lib/argos3/libargos3core_simulator.so -DARGOS_DYNAMICS2D_LIBRARY=$HOME/Programs/argos3/install/lib/argos3/libargos3plugin_simulator_dynamics2d.so -DARGOS_DYNAMICS3D_LIBRARY=$HOME/Programs/argos3/install/lib/argos3/libargos3plugin_simulator_dynamics3d.so -DARGOS_ENTITIES_LIBRARY=$HOME/Programs/argos3/install/lib/argos3/libargos3plugin_simulator_entities.so -DARGOS_GENERICROBOT_LIBRARY=$HOME/Programs/argos3/install/lib/argos3/libargos3plugin_simulator_genericrobot.so -DARGOS_POINTMASS3D_LIBRARY=$HOME/Programs/argos3/install/lib/argos3/libargos3plugin_simulator_pointmass3d.so -DARGOS_QTOPENGL_LIBRARY=$HOME/Programs/argos3/install/lib/argos3/libargos3plugin_simulator_qtopengl.so ../src
```

Now make

```
make -j4
make install
```

This should install these two components on newmajorana. 

### Notes regarding the implementation
Message types 
- 10 initial message - send by kilogrid
- 21 message from other robot (local broadcasting...)
- 22 message from kilogrid - sends position and option the robot is on
- 23 message from kilogrid - handles global communication
- 24 message from kilogrid - handles inter robot communication in the adaptive case. 

## Work with the cluster 

### Data transfer

To coppy working directory to cluster do:

```
cd argos3-test 
```

Now we can differ for the whole code do:

```
scp -r ARGoS_simulation taust@majorana.ulb.ac.be:~/Programs/argos3-test/
```

If you only replaced some code you can just copy these folders 

```
scp -r ARGoS_simulation/behaviours taust@majorana.ulb.ac.be:~/Programs/argos3-test/ARGoS_simulation/behaviours/
scp -r ARGoS_simulation/behaviours taust@majorana.ulb.ac.be:~/Programs/argos3-test/ARGoS_simulation/loop_functions
```

If you want to get back data

```
cd plotting 

scp -r taust@majorana.ulb.ac.be:~/Programs/argos3-test/data_cluster from_cluster
```

## Testing the cluster 
some useful comments:
starting an experiment:

```
sh ARGoS_simulation/data_generation_scripts/cluster_test.sh 0 0 local 0 3 1
```

reiner test text 
