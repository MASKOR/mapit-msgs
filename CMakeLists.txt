project(interface)
cmake_minimum_required(VERSION 2.8.12) # target_include_directories
aux_source_directory(. SRC_LIST)

file(GLOB ProtoFiles *.proto)
set(GENERATED_INCLUDE_INFIX generated/include)
set(GENERATED_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${GENERATED_INCLUDE_INFIX})
set(PROTOBUF_OUTPUT_DIR ${GENERATED_INCLUDE_DIR}/${MAPIT_INSTALL_INCLUDE}/msgs)

# create directory for headers
file(MAKE_DIRECTORY ${PROTOBUF_OUTPUT_DIR})

######## Protobuf
find_package(Protobuf QUIET REQUIRED)

# If you get errors with protobuf in this cmake, try the official way by uncommenting the next line starting with protobuf_...
# Note: this will not output headers in the correct folder. Find a workaround...
# TODO: results in /bin/sh: --cpp_out: Kommando nicht gefunden.
# TODO: Does not automatically find Protobuf_PROTOC_EXECUTABLE (Case is important).
#protobuf_generate_cpp(MESSAGES_SRCS MESSAGES_HDRS ${ProtoFiles})

# As a workaround this does the trick and generated protobuf message headers

foreach(FIL ${ProtoFiles})
    get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
    get_filename_component(FIL_WE ${FIL} NAME_WE)

    add_custom_command(
        OUTPUT "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.cc"
               "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.h"
        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --cpp_out ${PROTOBUF_OUTPUT_DIR} --proto_path ${CMAKE_CURRENT_SOURCE_DIR} ${ABS_FIL}
        DEPENDS ${FIL}
        COMMENT "Running C++ protocol buffer compiler on ${FIL}"
        VERBATIM )

    list(APPEND MESSAGES_SRCS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.cc")
    list(APPEND MESSAGES_HDRS "${PROTOBUF_OUTPUT_DIR}/${FIL_WE}.pb.h")

endforeach()

set_source_files_properties(${MESSAGES_SRCS} ${MESSAGES_HDRS} PROPERTIES GENERATED TRUE)

########

#Note: On Win32 this can not be a 'shared' library as it would create a dll without any exported symbol! Thus, make it a static linked library
add_library(${PROJECT_NAME} ${MESSAGES_SRCS} ${MESSAGES_HDRS})
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE TRUE)

# all projects depending on this will include PROTOBUF_OUTPUT_DIR
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${GENERATED_INCLUDE_DIR}>
                                                  $<INSTALL_INTERFACE:include>
                                                  ${Protobuf_INCLUDE_DIRS})

# make proto visible in qtcreator
file(GLOB_RECURSE UPNS_PROTOFILES ${PROJECT_SOURCE_DIR}/*.proto)
add_custom_target(${PROJECT_NAME}_ADDITIONAL_PROJECT_FILES ALL ${CMAKE_COMMAND} -E echo "Add proto files to project" SOURCES ${UPNS_PROTOFILES})

#file(GLOB_RECURSE UPNS_INTERFACE_PUBLIC_HEADER_FILES "${PROTOBUF_OUTPUT_DIR}/*.h")

#install(TARGETS ${PROJECT_NAME} DESTINATION lib/${CMAKE_LIBRARY_ARCHITECTURE})
#install(FILES ${UPNS_INTERFACE_PUBLIC_HEADER_FILES} DESTINATION include/${CMAKE_LIBRARY_ARCHITECTURE}/${MAPIT_INSTALL_INCLUDE}/msgs)

include(GenerateExportHeader)

generate_export_header(${PROJECT_NAME})
#set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${MAPIT_VERSION})
#set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${MAPIT_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY
  INTERFACE_mapit_interface_MAJOR_VERSION ${MAPIT_MAJOR_VERSION})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MAPIT_MAJOR_VERSION
)

install(TARGETS ${PROJECT_NAME} EXPORT upns_interface_targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
install(FILES ${MESSAGES_HDRS} DESTINATION "include/${MAPIT_INSTALL_INCLUDE}/msgs" )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_interfaceConfigVersion.cmake"
  VERSION 1
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT upns_interface_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/mapit_interfaceTargets.cmake"
  NAMESPACE mapit::
)

configure_file(cmake/mapit_interfaceConfig.cmake
  "${CMAKE_CURRENT_BINARY_DIR}/mapit_interfaceConfig.cmake"
  COPYONLY
)

set(ConfigPackageLocation lib/cmake/mapit)
install(EXPORT upns_interface_targets
  FILE
    mapit_interfaceTargets.cmake
  NAMESPACE
    mapit::
  DESTINATION
    ${ConfigPackageLocation}
)
install(
  FILES
    cmake/mapit_interfaceConfig.cmake
    "${CMAKE_CURRENT_BINARY_DIR}/mapit_interfaceConfigVersion.cmake"
  DESTINATION
    ${ConfigPackageLocation}
  COMPONENT
    Devel
)
