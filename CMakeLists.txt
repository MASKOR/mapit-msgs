project(upns_interface)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

file(GLOB ProtoFiles ${PROJECT_SOURCE_DIR}/messages/*.proto)

file(GLOB Outpath ${PROJECT_SOURCE_DIR}/generated)

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Outpath})
protobuf_generate_cpp(MESSAGES_SRCS MESSAGES_HDRS ${ProtoFiles})
#protobuf_generate_cpp(MESSAGES_SRCS MESSAGES_HDRS ${PROJECT_SOURCE_DIR}/messages ${ProtoFiles} ${Outpath})

add_library(${PROJECT_NAME} ${MESSAGES_SRCS} ${MESSAGES_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

SET(MESSAGES_HDRS ${MESSAGES_HDRS} PARENT_SCOPE)

#add_custom_target ( ${PROJECT_NAME} ALL
#    DEPENDS ${MESSAGES_SRCS} ${MESSAGES_HDRS} )

#include_directories(${PROJECT_SOURCE_DIR}/src/)

#file(GLOB_RECURSE UPNS_INTERFACE_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)

#add_library(${PROJECT_NAME} ${SRC_LIST} ${UPNS_INTERFACE_SOURCES} )

#target_link_libraries(${PROJECT_NAME} Qt5::Core )

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_PROTOFILES ${PROJECT_SOURCE_DIR}/messages/*.proto)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add proto files to project" SOURCES ${UPNS_PROTOFILES})
