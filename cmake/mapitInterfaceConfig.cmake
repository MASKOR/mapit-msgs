 # - Config file for the mapit::core package
# It defines the following variables
#  MAPIT_INTERFACE_INCLUDE_DIRS - include directories for mapit::core
#  MAPIT_INTERFACE_LIBRARIES    - libraries to link against
 
# Compute paths
#get_filename_component(MAPIT_INTERFACE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
#set(MAPIT_INTERFACE_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")
 
# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET upns_interface AND NOT upns_interface_BINARY_DIR)
  include("${MAPIT_INTERFACE_CMAKE_DIR}/mapitInterfaceTargets.cmake")
endif()
 
# These are IMPORTED targets created by mapitInterfaceTargets.cmake
#set(MAPIT_INTERFACE_LIBRARIES upns_interface)
