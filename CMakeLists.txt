project(upns_interface)
cmake_minimum_required(VERSION 2.8.12) # target_include_directories
aux_source_directory(. SRC_LIST)

file(GLOB ProtoFiles ${PROJECT_SOURCE_DIR}/messages/*.proto)

set(PROTOBUF_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})

######## Protobuf
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

# If you get errors with protobuf in this cmake, try the official way by uncommenting the next line starting with protobuf_...
# TODO: results in /bin/sh: --cpp_out: Kommando nicht gefunden.
# protobuf_generate_cpp(MESSAGES_SRCS MESSAGES_HDRS ${ProtoFiles})

# As a workaround this does the trick and generated protobuf message headers

foreach(FIL ${ProtoFiles})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    add_custom_command(
        OUTPUT "${FIL_WE}.pb.cc"
               "${FIL_WE}.pb.h"
        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out ${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/messages ${ABS_FIL}
        DEPENDS ${ABS_FIL}
        COMMENT "Running C++ protocol buffer compiler on ${FIL}"
        VERBATIM )

    list(APPEND MESSAGES_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.cc")
    list(APPEND MESSAGES_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.h")

endforeach()

set_source_files_properties(${MESSAGES_SRCS} ${MESSAGES_HDRS} PROPERTIES GENERATED TRUE)

########

#Note: On Win32 this can not be a 'shared' library as it would create a dll without any exported symbols! Thus, make it a static linked library
add_library(${PROJECT_NAME} ${MESSAGES_SRCS} ${MESSAGES_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# all projects depending on this will include PROTOBUF_OUTPUT_DIR
target_include_directories(${PROJECT_NAME} PUBLIC ${PROTOBUF_OUTPUT_DIR})

# make proto visible in qtcreator
file(GLOB_RECURSE UPNS_PROTOFILES ${PROJECT_SOURCE_DIR}/messages/*.proto)
add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add proto files to project" SOURCES ${UPNS_PROTOFILES})
