cmake_minimum_required(VERSION 3.5)
project(hybrid)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(SFML REQUIRED COMPONENTS graphics window system)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${SFML_INCLUDE_DIRS}
)

set(SOURCES
    src/main.cpp
    src/ompl_planner.cpp
    src/hybrid_astar.cpp
    src/node_3d.cpp
    src/utils.cpp
    src/astar.cpp
    src/transform_stack.cpp
    src/grid_map.cpp
    src/rs_path.cpp
)

add_executable(hybrid ${SOURCES})
target_link_libraries(hybrid ${OMPL_LIBRARIES} ${Boost_LIBRARIES} sfml-graphics sfml-window sfml-system)

message(STATUS "OMPL include dirs: ${OMPL_INCLUDE_DIRS}")
message(STATUS "OMPL libraries: ${OMPL_LIBRARIES}")

message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libraries: ${Boost_LIBRARIES}")

message(STATUS "SFML include dirs: ${SFML_INCLUDE_DIRS}")
message(STATUS "SFML libraries: ${SFML_LIBRARIES}")

