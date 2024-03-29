#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
using namespace ControlOptimize;
using namespace std;
// turn off this code because it's not consistent with DAG-control, need to
// change the I/O part on the coeff to make it work TEST(case1, v1) {
//     BeginTimer("main");
//     noiseModelSigma = 1;
//     if (optimizerType >= 5)
//         optimizerType = 2;
//     std::string path1 =
//         "/home/zephyr/Programming/others/YechengRepo/Experiment/"
//         "ControlPerformance/TestCases/NSweep/" +
//         controlPath + ".txt";
//     TaskSet tasks;
//     VectorDynamic coeff;
//     std::tie(tasks, coeff) = ReadControlCase(path1);
//     std::vector<bool> maskForElimination(tasks.size(), false);
//     VectorDynamic periodInitial1 = GenerateVectorDynamic(tasks.size());
//     // periodInitial1 << 815, 815, 815, 815, 591, 815, 815, 815, 815, 815,
//     815,
//     // 815, 815, 815, 815, 815, 204, 815, 815; periodInitial1 << 68, 300,
//     300,
//     // 300, 300; UpdateTaskSetPeriod(tasks, periodInitial1); Reorder(tasks,
//     // coeff, "RM"); maskForElimination[1] = 1; auto sth =
//     // OptimizeTaskSetIterativeWeight<FactorGraphInManifold<RTA_LL>>(tasks,
//     // coeff, maskForElimination);
//     auto sth2 =
//         OptimizeTaskSetIterative<FactorGraphInManifold<TaskSetNormal,
//         RTA_LL>,
//                                  TaskSetNormal, RTA_LL>(
//             tasks, coeff, maskForElimination, TaskSetNormal(tasks));

//     // FindEliminatedVariables(tasks, maskForElimination);
//     // AssertEqualVectorExact({true, false, false, false, false},
//     // maskForElimination);
//     UpdateTaskSetPeriod(tasks, sth2.first);
//     cout << "Actual objective function is "
//          << FactorGraphInManifold<TaskSetNormal, RTA_LL>::RealObj(
//                 tasks, coeff, TaskSetNormal(tasks))
//          << endl;
//     EndTimer("main");
//     PrintTimer();
// }

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
