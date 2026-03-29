project(sillm)

cmake_minimum_required (VERSION 2.6)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

# if set, use dev mode, which has a lot of logging and timing.
# add_definitions(-DDEV)
# if set, use action model with no rotation.
add_definitions(-DNO_ROT)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories("inc")
file(GLOB_RECURSE SOURCES "src/*.cpp")
add_executable(sillm ${SOURCES})

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem iostreams)
find_package(spdlog REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

# Find OpenMP
find_package(OpenMP REQUIRED)

# Find Threads
find_package(Threads REQUIRED)          # portable way

target_link_libraries(sillm ${Boost_LIBRARIES} rpc pthread spdlog::spdlog ${OpenMP_CXX_LIBRARIES})

target_compile_options(sillm PRIVATE -pthread)   # adds -D_REENTRANT automatically
