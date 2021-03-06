 # - Config file for the mapit::core package
# It defines the following variables
#  MAPIT_INTERFACE_INCLUDE_DIRS - include directories for mapit::core
#  MAPIT_INTERFACE_LIBRARIES    - libraries to link against
 
# Compute paths
#get_filename_component(MAPIT_INTERFACE_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(MAPIT_INTERFACE_CMAKE_DIR ${CMAKE_CURRENT_LIST_DIR})
#set(MAPIT_INTERFACE_INCLUDE_DIRS "@CONF_INCLUDE_DIRS@")

include(CMakeFindDependencyMacro)
find_dependency(Protobuf REQUIRED) 

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET mapit::interface AND NOT mapit::interface_BINARY_DIR)
  include("${MAPIT_INTERFACE_CMAKE_DIR}/mapit_interfaceTargets.cmake")
endif()
 
# These are IMPORTED targets created by mapitInterfaceTargets.cmake
#set(MAPIT_INTERFACE_LIBRARIES mapit_interface)
