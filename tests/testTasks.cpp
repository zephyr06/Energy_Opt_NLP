

#include <CppUnitLite/TestHarness.h>

#include "../sources/Tasks.h"
#include "../sources/RTA_LL.h"
#include "../sources/Parameters.h"
#include "../sources/Optimize.h"
#include "../sources/DAG_Task.h"

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

    auto dagTasks = ReadDAG_Tasks(path, "RM");
    // AssertEqualScalar(2, dagTasks.mapPrev.size());
    AssertEqualScalar(3, GetDependentTasks(dagTasks, 0).size());
    AssertEqualScalar(3, GetDependentTasks(dagTasks, 1).size());
    AssertEqualScalar(0, GetDependentTasks(dagTasks, 2).size());
}
TEST(dag, v2)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v25.csv";

    auto dagTasks = ReadDAG_Tasks(path, "orig");
    double longest = dagTasks.CriticalPath();
    AssertEqualScalar(35, longest, 1e-6, __LINE__);
}

TEST(dag, v3)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v26.csv";

    auto dagTasks = ReadDAG_Tasks(path, "orig");
    double longest = dagTasks.CriticalPath();
    AssertEqualScalar(96, longest, 1e-6, __LINE__);
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
