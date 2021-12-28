include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(kilogrid_local MODULE kilogrid.h kilogrid_local.cpp)
target_link_libraries(kilogrid_local
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)

add_library(kilogrid_global MODULE kilogrid.h kilogrid_global.cpp)
target_link_libraries(kilogrid_global
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)

add_library(kilogrid_adaptive MODULE kilogrid.h kilogrid_adaptive.cpp)
target_link_libraries(kilogrid_adaptive
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)

add_library(kilogrid_stub MODULE kilogrid_stub.h kilogrid_stub.cpp)
target_link_libraries(kilogrid_stub
        argos3core_simulator
        argos3plugin_simulator_dynamics2d
        argos3plugin_simulator_entities
        argos3plugin_simulator_media
        argos3plugin_simulator_kilobot
        argos3plugin_simulator_kilolib
        )
