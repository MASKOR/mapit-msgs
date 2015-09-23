project(upns_interface)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

# proto-files are compiled to *.pb.h and *.pb.cc
# output files will be located in ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
# As this is a subdirectory, the parent cmake does not know about
# MESSAGES_SRCS MESSAGES_HDRS and thus does not know about any output or any headers
# add include_directories(${CMAKE_CURRENT_BINARY_DIR}) to the parent cmake
# in project-header files, add #include "<PROJECT_NAME>/<message_name>.pb.h" to your sources
# e.g. #include "upns_interface/services.pb.h"

file(GLOB ProtoFiles ${PROJECT_SOURCE_DIR}/messages/*.proto)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(MESSAGES_SRCS MESSAGES_HDRS ${ProtoFiles})

add_library(${PROJECT_NAME} ${MESSAGES_SRCS} ${MESSAGES_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_PROTOFILES ${PROJECT_SOURCE_DIR}/messages/*.proto)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add proto files to project" SOURCES ${UPNS_PROTOFILES})
