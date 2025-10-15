cmake_minimum_required(VERSION 3.30)
project(DronePositionHoldSimulation)

set(CMAKE_CXX_STANDARD 23)

add_executable(DronePositionHoldSimulation src/main.cpp
        src/Drone.cpp
        src/CameraOpticalFlow.cpp)

target_include_directories(DronePositionHoldSimulation PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

add_subdirectory(lib/SimulationAPI)

target_link_libraries(DronePositionHoldSimulation PRIVATE SimulationAPI)

target_compile_definitions(SimulationAPI PUBLIC
        -DSIM_REMOTEAPICLIENT_OBJECTS
)
