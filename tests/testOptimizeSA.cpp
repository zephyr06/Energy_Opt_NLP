#include "../sources/OptimizeSA.h"
using namespace std;
using namespace std::chrono;
using namespace rt_num_opt;

TEST(SA, v1)
{
    TaskSet tasks = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv", "orig");
    TaskSetNormal tasksN(tasks);
    // auto sth = Generate_WAP(tasks);
    // bool success;
    // std::tie(success, A_Global, P_Global) = sth;
    // if (not success)
    // {
    //     CoutWarning("Unschedulable task set in SA test!");
    // }
    auto start = std::chrono::high_resolution_clock::now();
    auto res = OptimizeSchedulingSA<TaskSetNormal, RTA_LL>(tasksN);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    cout << "The time taken is: " << double(duration.count()) / 1e6 << "seconds" << endl;
    cout << "The error before optimization is " << Color::green << res.initialError << Color::def << endl;
    cout << "The error after optimization is " << Color::green << res.optimizeError << Color::def << endl;
    cout << "The result after optimization is " << Color::blue << res.optimizeVariable << Color::def << endl;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
