cmake_minimum_required (VERSION 3.10)

project(sillm)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

# if set, use dev mode, which has a lot of logging and timing.
# add_definitions(-DDEV)
# if set, use action model with no rotation.
add_definitions(-DNO_ROT)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

include_directories("inc")
file(GLOB_RECURSE SOURCES "src/*.cpp")
# Properly filter out entry files using CMake regex
list(FILTER SOURCES EXCLUDE REGEX ".*src/entry/.*\\.cpp$")

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem iostreams)
find_package(spdlog REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
# Find OpenMP
find_package(OpenMP REQUIRED)
# Find Threads
find_package(Threads REQUIRED)
# Find fmt
find_package(fmt REQUIRED)

# Create a shared library for common code
add_library(lib_core STATIC ${SOURCES})
set_target_properties(lib_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(lib_core PUBLIC ${Boost_LIBRARIES})
target_link_libraries(lib_core PUBLIC OpenMP::OpenMP_CXX)
target_link_libraries(lib_core PUBLIC spdlog::spdlog)
target_link_libraries(lib_core PUBLIC Threads::Threads)
target_link_libraries(lib_core PUBLIC fmt::fmt)

add_executable(discrete_driver "src/entry/discrete_driver.cpp")
target_link_libraries(discrete_driver PUBLIC lib_core)

add_executable(lsmart_driver "src/entry/lsmart_driver.cpp")
target_link_libraries(lsmart_driver PUBLIC lib_core rpc)
