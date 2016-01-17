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

#set(PROTOBUF_OUTPUT_DIR ${PROJECT_SOURCE_DIR}/generated)
set(PROTOBUF_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR})


set(UPNS_INTERFACE_INCLUDE ${PROTOBUF_OUTPUT_DIR})

#find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})

######## Protobuf + gRpc
#protobuf_generate_cpp(MESSAGES_SRCS MESSAGES_HDRS ${ProtoFiles})

#use_grpc()
use_protobuf()

foreach(FIL ${ProtoFiles})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    message("PROTOBUF generatres: " ${FIL_WE})
#    add_custom_command(
#        OUTPUT "${FIL_WE}.grpc.pb.cc"
#             "${FIL_WE}.grpc.pb.h"
#        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
#        ARGS --grpc_out=${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/messages --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN} ${ABS_FIL}
#        DEPENDS ${ABS_FIL}
#        COMMENT "Running grpc protocol buffer compiler on ${FIL}"
#        VERBATIM )

    add_custom_command(
        OUTPUT "${FIL_WE}.pb.cc"
               "${FIL_WE}.pb.h"
        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out ${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR}/messages ${ABS_FIL}
        DEPENDS ${ABS_FIL}
        COMMENT "Running C++ protocol buffer compiler on ${FIL}"
        VERBATIM )


#    list(APPEND MESSAGES_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.grpc.pb.cc")
#    list(APPEND MESSAGES_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.grpc.pb.h")

    list(APPEND MESSAGES_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.cc")
    list(APPEND MESSAGES_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.h")

endforeach()

set_source_files_properties(${MESSAGES_SRCS} ${MESSAGES_HDRS} PROPERTIES GENERATED TRUE)
#set(MESSAGES_SRCS ${MESSAGES_SRCS} PARENT_SCOPE)
#set(MESSAGES_HDRS ${MESSAGES_HDRS} PARENT_SCOPE)

########
include_directories(${UPNS_INTERFACE_INCLUDE})

#Note: On Win32 this can not be a 'shared' library as it would create a dll without any exported symbols! Thus, make it a static linked library
add_library(${PROJECT_NAME} ${MESSAGES_SRCS} ${MESSAGES_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})

# all projects depending on this will include PROTOBUF_OUTPUT_DIR
target_include_directories(${PROJECT_NAME} PUBLIC ${PROTOBUF_OUTPUT_DIR})

# make qml visible in qtcreator
file(GLOB_RECURSE UPNS_PROTOFILES ${PROJECT_SOURCE_DIR}/messages/*.proto)

add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add proto files to project" SOURCES ${UPNS_PROTOFILES})
