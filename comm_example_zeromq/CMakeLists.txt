cmake_minimum_required (VERSION 2.8.5)
project (comm_test_zmq)

# cpp11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

#boost
find_package(Boost 1.40.0 REQUIRED system)
include_directories(${Boost_INCLUDE_DIR})
set (COMM_LIBS ${COMM_LIBS} ${Boost_LIBRARIES})

#protobuf
find_package(Protobuf QUIET REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
set (COMM_LIBS ${COMM_LIBS} ${PROTOBUF_LIBRARIES})

#zeromq
find_package(ZMQ REQUIRED)
include_directories(${ZeroMQ_INCLUDE_DIR})
set(COMM_LIBS ${COMM_LIBS} ${ZeroMQ_LIBRARIES})

MESSAGE(STATUS "\n\nZMQ_LIBRARIES: ${ZeroMQ_LIBRARIES}\n\n")

#protobuf msgs
include_directories(${CMAKE_CURRENT_BINARY_DIR})
file(GLOB ProtoFiles ${PROJECT_SOURCE_DIR}/../messages/*.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${ProtoFiles})
set (COMM_EXE ${COMM_EXE} ${PROTO_SRCS} ${PROTO_HDRS})

#pcl
find_package(PCL QUIET 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set (COMM_LIBS ${COMM_LIBS} ${PCL_LIBRARIES})

add_executable(client
    client.cpp
    zmq_pair.cpp
    ${COMM_EXE}
)
target_link_libraries(client ${COMM_LIBS})

add_executable(server
    server.cpp
    zmq_pair.cpp
    ${COMM_EXE}
)
target_link_libraries(server ${COMM_LIBS})
