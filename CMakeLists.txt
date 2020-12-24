cmake_minimum_required(VERSION 2.8)

project(mdss)

# Enforce aggressive optimization 
set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -fPIC")

# Generate debug information
set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")

find_package(Eigen3 REQUIRED)
message("Found eigen: " ${EIGEN3_INCLUDE_DIR})

include_directories(src/Application)
include_directories(src/SolverCore)
include_directories(src/Subsystems)
include_directories(src/UtilityFunctions)
include_directories(src)


message("Cmake source dir:\n - " ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE SOLVER_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/SolverCore/*.cpp)
file(GLOB_RECURSE SUBSYSTEM_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/Subsystems/*.cpp)
file(GLOB_RECURSE UTILITY_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/UtilityFunctions/*.cpp)
file(GLOB_RECURSE APPLICATION_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/Application/*.cpp)

message("\nFound solver sources:")
foreach(IT IN LISTS SOLVER_SOURCES)
  message(" - " ${IT})
endforeach()

message("\nFound subsystem sources:")
foreach(IT IN LISTS SUBSYSTEM_SOURCES)
  message(" - " ${IT})
endforeach()

message("\nFound utility sources:")
foreach(IT IN LISTS UTILITY_SOURCES)
  message(" - " ${IT})
endforeach()

message("\nFound application sources:")
foreach(IT IN LISTS APPLICATION_SOURCES)
  message(" - " ${IT})
endforeach()

add_library(
  SOLVER_LIBRARIES
  SHARED
  ${SOLVER_SOURCES}
)

add_library(
  SUBSYSTEM_LIBRARIES
  SHARED
  ${SUBSYSTEM_SOURCES}
)

add_library(
  UTILITY_LIBRARIES
  SHARED
  ${UTILITY_SOURCES}
)

add_library(
  APPLICATION_LIBRARIES
  SHARED
  ${APPLICATION_SOURCES}
)

add_executable(solver_test src/solver_test.cpp ${Eigen_LIBRARIES} ${SOLVER_SOURCES} ${SUBSYSTEM_SOURCES} ${UTILITY_SOURCES})
target_link_libraries(solver_test ${Eigen_LIBRARIES} ${SOLVER_LIBRARIES} ${SUBSYSTEM_LIBRARIES} ${UTILITY_LIBRARIES})

add_executable(c172test C172test/C172test.cpp ${Eigen_LIBRARIES} ${SOLVER_SOURCES} ${SUBSYSTEM_SOURCES} ${UTILITY_SOURCES} ${APPLICATION_SOURCES})
target_link_libraries(c172test ${Eigen_LIBRARIES} ${SOLVER_LIBRARIES} ${SUBSYSTEM_LIBRARIES} ${UTILITY_LIBRARIES} ${APPLICATION_LIBRARIES})