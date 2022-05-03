

#include <CppUnitLite/TestHarness.h>

#include "sources/TaskModel/Tasks.h"
#include "sources/RTA/RTA_LL.h"
#include "sources/RTA/RTA_Melani.h"
#include "sources/Utils/Parameters.h"
#include "sources/BatchTestutils.h"
using namespace rt_num_opt;
using namespace std;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;
TEST(ReadTaskSet, p1)
{

    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v1.csv";

    TaskSet taskset1 = ReadTaskSet(path, "RM");

    CHECK_EQUAL(500, taskset1[0].period);
    CHECK_EQUAL(550, taskset1[1].period);
    CHECK_EQUAL(880, taskset1[2].period);
    CHECK_EQUAL(2, taskset1[0].overhead);
}

TEST(parameters, a2)
{
    int a = 3;
    MatrixXd A(a, a);
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    MatrixXd B = A;
    MatrixXd C = A * B;
    CHECK_EQUAL(30, C(0, 0));
}

TEST(read_dag, v1)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v25.csv";

    auto dagTasks = ReadDAG_Task(path, "RM");
    // AssertEqualScalar(2, dagTasks.mapPrev.size());
    AssertEqualScalar(3, GetDependentTasks(dagTasks, 0).size());
    AssertEqualScalar(3, GetDependentTasks(dagTasks, 1).size());
    AssertEqualScalar(0, GetDependentTasks(dagTasks, 2).size());
}
TEST(dag, v2)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v25.csv";

    auto dagTasks = ReadDAG_Task(path, "orig");
    double longest = dagTasks.CriticalPath();
    AssertEqualScalar(35, longest, 1e-6, __LINE__);
}

TEST(dag, v3)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v26.csv";

    auto dagTasks = ReadDAG_Task(path, "orig");
    double longest = dagTasks.CriticalPath();
    AssertEqualScalar(96, longest, 1e-6, __LINE__);
}
TEST(dag, v4)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v27.csv";

    auto dagTasks = ReadDAG_Tasks(path, "orig");
    AssertEqualScalar(124, dagTasks.volumeVec_[3], 1e-6, __LINE__);
    AssertEqualScalar(98, dagTasks.longestVec_[3], 1e-6, __LINE__);
    AssertEqualScalar(184, dagTasks.volumeVec_[0], 1e-6, __LINE__);
    AssertEqualScalar(116, dagTasks.longestVec_[0], 1e-6, __LINE__);
    AssertEqualScalar(200, dagTasks.tasks_[1].period, 1e-6, __LINE__);
}
TEST(dag, v5)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v30.csv";

    auto dagTasks = ReadDAG_Tasks(path, "orig");
    AssertEqualScalar(778.401, dagTasks.weightVec_[0], 1e-6, __LINE__);
    AssertEqualScalar(916.583, dagTasks.weightVec_[1], 1e-6, __LINE__);
    AssertEqualScalar(161.206, dagTasks.weightVec_[4], 1e-6, __LINE__);
}
TEST(ReadFrequencyModeRatio, v1)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v29.csv";

    auto dagTasks = ReadTaskSet(path, "orig");
    AssertEqualScalar(0.296547, frequencyRatio);
}
// TEST(ReadBaselineResult, v1)
// {
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number/periodic-set-000-syntheticJobs";
//     baselineLLCompare = 2;
//     auto sth = ReadBaselineResult(path, 5);
//     AssertEqualScalar(64.116, sth.first);
//     AssertEqualScalar(646110010.878333, sth.second);
// }
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
