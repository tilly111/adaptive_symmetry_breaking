if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/argos3/plugins/robots/kilobot/control_interface)

add_executable(agent_stub agent.h agent_stub.c)
if(UNIX AND NOT APPLE)
  target_link_libraries(agent_stub  argos3plugin_simulator_kilolib rt m)
else(UNIX AND NOT APPLE)
  target_link_libraries(agent_stub  argos3plugin_simulator_kilolib)
endif()

endif(ARGOS_BUILD_FOR_SIMULATOR)
