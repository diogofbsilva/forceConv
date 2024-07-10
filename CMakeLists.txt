cmake_minimum_required(VERSION 3.16)
project(forceConversion)

add_subdirectory(iDynTree)

find_package(iDynTree REQUIRED)

# Set include directories for iDynTree
include_directories(${iDynTree_INCLUDE_DIRS})

#include(/home/vislab/robotology-superbuild/src/ICUB/conf/iCubInstallBasicPackageFiles.cmake)

# Set C++ standard to C++11 (or higher if needed)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the iDyn library
file(GLOB IDYN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../iDyn/src/*.cpp")
add_library(iDyn ${IDYN_SOURCES})

target_include_directories(iDyn PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../iDyn/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${CMAKE_CURRENT_SOURCE_DIR}/../iDyn/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../ctrlLib/include  # Include ctrlLib headers
    ${CMAKE_CURRENT_SOURCE_DIR}/../skinDynLib/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../iKin/include
)

# Add the iDyn library
file(GLOB IKINN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../iKin/src/iKinFwd.cpp")
add_library(iKin ${IKINN_SOURCES})

target_include_directories(iKin PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../iKin/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${CMAKE_CURRENT_SOURCE_DIR}/../ctrlLib/include  # Include ctrlLib headers
)


# Add the iDyn library
file(GLOB CTRL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../ctrlLib/src/math.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/../ctrlLib/src/adaptWinPolyEstimator.cpp")
add_library(ctrl ${CTRL_SOURCES})

target_include_directories(ctrl PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../ctrlLib/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Add the skinDynLib library
file(GLOB SKINDYNLIB_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/../skinDynLib/src/*.cpp")
add_library(skinDynLib ${SKINDYNLIB_SOURCES})

target_include_directories(skinDynLib PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../skinDynLib/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${CMAKE_CURRENT_SOURCE_DIR}/../skinDynLib/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../ctrlLib/include  # Include ctrlLib headers
)

# Find YARP if not already found
find_package(YARP REQUIRED)

# Link necessary libraries
target_link_libraries(ctrl PRIVATE YARP::YARP_OS)
target_link_libraries(iKin PRIVATE YARP::YARP_OS YARP::YARP_math YARP::YARP_sig YARP::YARP_dev)
target_link_libraries(iDyn PRIVATE ctrl iKin YARP::YARP_OS YARP::YARP_math YARP::YARP_sig YARP::YARP_dev)
target_link_libraries(skinDynLib PRIVATE YARP::YARP_OS YARP::YARP_math YARP::YARP_sig YARP::YARP_dev)

# Add the executable
add_executable(forceConv main.cpp forceConv.cpp forceConv.h)

# Link iDyn and skinDynLib libraries
target_link_libraries(forceConv PRIVATE ctrl iDyn iKin skinDynLib YARP::YARP_OS YARP::YARP_math YARP::YARP_sig YARP::YARP_dev YARP::YARP_init iDynTree::idyntree-core iDynTree::idyntree-high-level iDynTree::idyntree-icub iDynTree::idyntree-estimation iDynTree::idyntree-inverse-kinematics)
