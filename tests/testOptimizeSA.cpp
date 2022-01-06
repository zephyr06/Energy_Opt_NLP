#include "../sources/OptimizeSA.h"
#include "../sources/Generate_WAP.h"
using namespace std::chrono;
TEST(SA, v1)
{
    TaskSet tasks = ReadTaskSet("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv", "orig");
    // auto sth = Generate_WAP(tasks);
    // bool success;
    // std::tie(success, A_Global, P_Global) = sth;
    // if (not success)
    // {
    //     CoutWarning("Unschedulable task set in SA test!");
    // }
    auto start = chrono::high_resolution_clock::now();
    auto res = OptimizeSchedulingSA<RTA_LL>(tasks);
    auto stop = chrono::high_resolution_clock::now();
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
