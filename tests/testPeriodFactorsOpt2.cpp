#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
using namespace ControlOptimize;
using namespace std;
TEST(case1, v1) {
    BeginTimer("main");
    noiseModelSigma = 1;
    if (optimizerType >= 5) optimizerType = 2;
    std::string path1 =
        "/home/zephyr/Programming/others/YechengRepo/Experiment/"
        "ControlPerformance/TestCases/NSweep/" +
        controlPath + ".txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    VectorDynamic periodInitial1 = GenerateVectorDynamic(tasks.size());
    // periodInitial1 << 815, 815, 815, 815, 591, 815, 815, 815, 815, 815, 815,
    // 815, 815, 815, 815, 815, 204, 815, 815; periodInitial1 << 68, 300, 300,
    // 300, 300; UpdateTaskSetPeriod(tasks, periodInitial1); Reorder(tasks,
    // coeff, "RM"); maskForElimination[1] = 1; auto sth =
    // OptimizeTaskSetIterativeWeight<FactorGraphInManifold>(tasks, coeff,
    // maskForElimination);
    auto sth2 = OptimizeTaskSetIterative<FactorGraphInManifold>(
        tasks, coeff, maskForElimination);

    // FindEliminatedVariables(tasks, maskForElimination);
    // AssertEqualVectorExact({true, false, false, false, false},
    // maskForElimination);
    UpdateTaskSetPeriod(tasks, sth2.first);
    cout << "Actual objective function is "
         << FactorGraphInManifold::RealObj(tasks, coeff) << endl;
    EndTimer("main");
    PrintTimer();
    std::cout << count1 << ", " << count2 << std::endl;
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
