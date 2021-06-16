# gtsamAddTestsGlob(EnergyOptNLP "test*.cpp" "" "${CONVENIENCE_LIB_NAME}")
ADD_EXECUTABLE(tmain testUtils.cpp)
TARGET_LINK_LIBRARIES(tmain ${CONVENIENCE_LIB_NAME})
