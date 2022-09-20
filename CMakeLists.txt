cmake_minimum_required(VERSION 3.0)
project(EnergyOptNLP CXX C)
set (CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(yaml-cpp)

# add_subdirectory(DAG-scheduling_Verucchi)
# include_directories(${CMAKE_SOURCE_DIR}/DAG-scheduling_Verucchi/include)
include_directories(${CMAKE_SOURCE_DIR}/np-schedulability-analysis/include )
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/np-schedulability-analysis/include )
include_directories(${CMAKE_SOURCE_DIR})

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
find_package(CppUnitLite REQUIRED)
find_package( OpenCV REQUIRED )
include_directories(${Boost_INCLUDE_DIR})
include_directories(includeMoe)

find_package(ifopt)

# file(GLOB sources "*.cpp" "*.h")
file(GLOB sources "sources/*.cpp" "sources/*.h")
set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
add_library(${CONVENIENCE_LIB_NAME} SHARED ${sources})
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)



# linking with dependencies
target_link_libraries(${CONVENIENCE_LIB_NAME} ${Boost_LIBRARIES})
target_link_libraries(${CONVENIENCE_LIB_NAME} CppUnitLite)
target_link_libraries(${CONVENIENCE_LIB_NAME} ${OpenCV_LIBS} )
target_link_libraries(${CONVENIENCE_LIB_NAME} yaml-cpp)
target_link_libraries(${CONVENIENCE_LIB_NAME} tbb)
target_link_libraries(${CONVENIENCE_LIB_NAME} ifopt::ifopt_ipopt) 


add_subdirectory(sources)
add_subdirectory(tests)

