cmake_minimum_required(VERSION 3.10)

project(BezierTest VERSION 1.0)

SET(Glog_DIR /home/holo/HouseClean_ws/output/native/relwithdebinfo/target/3rdparty/lib/cmake/glog)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" -Wall -lrt -03)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})

include_directories(${Glog_DIR})
include_directories(/usr/include/python2.7)
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

find_package(Boost 1.65 REQUIRED COMPONENTS system thread python)
find_package(Glog REQUIRED)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREADED ON)
SET(Boost_USE_STATIC_RUNTIME OFF)

configure_file(__init__.py ${PROJECT_SOURCE_DIR}/build/__init__.py COPYONLY)

add_library(${PROJECT_NAME} SHARED bezier_curve.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${Boost_LIBRARIES} -lpython2.7 glog -fPIC)

ADD_DEFINITIONS("-Wall")
