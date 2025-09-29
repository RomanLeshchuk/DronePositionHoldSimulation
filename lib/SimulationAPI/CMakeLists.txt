cmake_minimum_required(VERSION 3.30)
project(SimulationAPI)

set(CMAKE_CXX_STANDARD 23)

find_package(Boost REQUIRED)
find_package(cppzmq REQUIRED)
find_package(jsoncons REQUIRED)
find_package(OpenCV REQUIRED)

add_library(SimulationAPI STATIC
        src/RemoteAPIClient.cpp
)

target_include_directories(SimulationAPI PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(SimulationAPI PUBLIC
        Boost::boost
        cppzmq
        jsoncons
        ${OpenCV_LIBS}
)
