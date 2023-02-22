#include "sources/EnergyOptimization/OptimizeSA.h"
#include "sources/RTA/RTA_Nasri19.h"

using namespace std;
using namespace std::chrono;
using namespace rt_num_opt;

TEST(SA, v1)
{
    std::string path = ("/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".yaml");
    rt_num_opt::DAG_Nasri19 tasksN = rt_num_opt::ReadDAGNasri19_Tasks(path);
    auto start = std::chrono::high_resolution_clock::now();
    auto res = OptimizeSchedulingSA<rt_num_opt::DAG_Nasri19, rt_num_opt::RTA_Nasri19>(tasksN);
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
