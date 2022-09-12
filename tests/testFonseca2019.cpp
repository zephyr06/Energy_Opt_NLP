#include <CppUnitLite/TestHarness.h>

#include "sources/RTA/RTA_Nasri19.h"

// TEST(read, v1)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v33.csv";
//     auto tasksetVerucchi = rt_num_opt::TransformTaskSetNumOpt2dagSched(path);

//     EXPECT_DOUBLES_EQUAL(90 + 69 + 4, tasksetVerucchi.tasks[0].getLength(), 1e-3);
// }

// TEST(GP_FP_FTP_Nasri19, v1)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v34.csv";
//     auto tasksetVerucchi = rt_num_opt::TransformTaskSetNumOpt2dagSched(path);
//     int n_proc = 2;
//     std::cout << std::endl
//               << "\tFonseca 2019 constrained (GP-FP-FTP): " << dagSched::GP_FP_FTP_Nasri19(tasksetVerucchi, n_proc) << std::endl;
// }

// TEST(GP_FP_FTP_Nasri19, v2)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v25.csv";
//     auto tasksetVerucchi = rt_num_opt::TransformTaskSetNumOpt2dagSched(path);
//     int n_proc = 2;
//     std::cout << std::endl
//               << "\tFonseca 2019 constrained (GP-FP-FTP): " << dagSched::GP_FP_FTP_Nasri19(tasksetVerucchi, n_proc) << std::endl;
// }

// TEST(GP_FP_FTP_Nasri19, v3)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n8_v1.csv";
//     auto tasksetVerucchi = rt_num_opt::TransformTaskSetNumOpt2dagSched(path);
//     int n_proc = 4;
//     std::cout << std::endl
//               << "\tFonseca 2019 constrained (GP-FP-FTP): " << dagSched::GP_FP_FTP_Nasri19(tasksetVerucchi, n_proc) << std::endl;
//     EXPECT_LONGS_EQUAL(15, dagSched::RTA_Nasri19(tasksetVerucchi, n_proc)[0]);
// }
// TEST(rta, v1)
// {
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n8_v1.csv";
//     rt_num_opt::DAG_Model dagTasks = rt_num_opt::ReadDAG_Task(path, "RM");
//     rt_num_opt::RTA_Nasri19 r(dagTasks);
//     rt_num_opt::core_m_dag = 4;
//     EXPECT_LONGS_EQUAL(15, r.RTA_Common_Warm(0, 7));
//     EXPECT_LONGS_EQUAL(0, r.RTA_Common_Warm(0, 6));
// }

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
