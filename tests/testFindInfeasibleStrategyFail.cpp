#include <CppUnitLite/TestHarness.h>

#include "sources/BatchTestutils.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Utils/FeasibleSolutionEstimate.h"
#include "sources/Utils/Parameters.h"
using namespace rt_num_opt;

// TEST(iterate, v4) {
//     BeginTimer("main");
//     std::string path =
//         "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v16.yaml";
//     rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
//     EXPECT(WhetherTaskSetSchedulableInAllSolutionSpace(tasksN));
//     EndTimer("main");
//     PrintTimer();
// }

// TEST(iterate, v3) {
//     BeginTimer("main");
//     std::string path =
//         "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v13.yaml";
//     rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
//     EXPECT(WhetherTaskSetSchedulableInAllSolutionSpace(tasksN));
//     EndTimer("main");
//     PrintTimer();
// }
// TEST(iterate, v3) {
//     BeginTimer("main");
//     std::string path =
//         "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v17.yaml";
//     rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
//     EXPECT(WhetherTaskSetSchedulableInAllSolutionSpace(tasksN));
//     EndTimer("main");
//     PrintTimer();
// }

TEST(iterate, v1) {
    BeginTimer("main");
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v14.yaml";
    rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
    EXPECT(WhetherTaskSetSchedulableInAllSolutionSpace(tasksN));
    EndTimer("main");
    PrintTimer();
}

TEST(iterate, v2) {
    BeginTimer("main");
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v15.yaml";
    rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
    EXPECT(!WhetherTaskSetSchedulableInAllSolutionSpace(tasksN));
    EndTimer("main");
    PrintTimer();
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
