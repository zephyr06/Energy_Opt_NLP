#include "sources/ControlOptimization/ControlOptimize.h"
#include "sources/BatchControlOptimize.h"
#include "sources/Utils/Parameters.h"
using namespace std;
using namespace std::chrono;

using namespace rt_num_opt;
using Opt_LL = Energy_Opt<TaskSetNormal, RTA_LL>;
using namespace ControlOptimize;
using namespace std;

using namespace gtsam;

TEST(error, v1)
{
    whether_ls = 1;
    noiseModelSigma = 1;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination[0] = 1;
    VectorDynamic initial = GenerateVectorDynamic(5);
    initial << 68.0034,
        321.249,
        400.807,
        131.088,
        308.676;
    UpdateTaskSetPeriod(tasks, initial);
    gtsam::NonlinearFactorGraph graph = FactorGraphInManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    auto initialEstimateFG = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    auto sth = graph.linearize(initialEstimateFG)->jacobian();
    MatrixDynamic jacobianCurr = sth.first;
    // std::cout << "Current Jacobian matrix:" << endl;
    // std::cout << jacobianCurr << endl;
    // std::cout << "Current b vector: " << endl;
    // std::cout << sth.second << endl;
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(12, 4);
    jacobianExpect(0, 0) = 275;
    jacobianExpect(3, 1) = 217;
    jacobianExpect(6, 2) = 489;
    jacobianExpect(9, 3) = 285;
    AssertEigenEqualMatrix(jacobianExpect, jacobianCurr);
    AssertEqualScalar(321.249 * -1 * 275, sth.second(0, 0));
    AssertEqualScalar(-50 * 9334, sth.second(1, 0));
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
