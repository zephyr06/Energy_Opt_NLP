#include <CppUnitLite/TestHarness.h>

#include "sources/EnergyOptimization/EnergyFactor.h"
#include "sources/TaskModel/DAG_Task.h"
TEST(a, b) {
    using namespace rt_num_opt;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    YAML::Node config = YAML::LoadFile(path);
    YAML::Node tasksNode = config["tasks"];
    EXPECT_LONGS_EQUAL(20, tasksNode[0]["period"].as<int>());
    EXPECT_LONGS_EQUAL(20, tasksNode[0]["deadline"].as<long int>());

    YAML::Node vert = tasksNode[0]["vertices"];
    EXPECT_LONGS_EQUAL(4, vert[1]["executionTime"].as<int>());
    // std::cout << YAML::_Tag << std::endl;
}
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
