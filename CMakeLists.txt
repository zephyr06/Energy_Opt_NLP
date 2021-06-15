cmake_minimum_required(VERSION 3.0)
project(EnergyOptNLP CXX C)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
include_directories("/home/zephyr/Library/gtsam/gtsam/3rdparty/Eigen/unsupported")
# find_package( OpenCV REQUIRED )
# find_package( doubleExpresso REQUIRED)

# find_package(Boost 1.43 COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex)

file(GLOB sources "*.cpp" "*.h")
set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
add_library(${CONVENIENCE_LIB_NAME} SHARED ${sources})
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)

# linking with dependencies
target_link_libraries(${CONVENIENCE_LIB_NAME} ${Boost_LIBRARIES})
# target_link_libraries(${CONVENIENCE_LIB_NAME} ${OpenCV_LIBS} )

add_subdirectory(tests)
add_subdirectory(scripts)
