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
    whether_ls = 0;
    exactJacobian = 0;
    noiseModelSigma = 1;
    if (optimizerType >= 5)
        optimizerType = 2;
    std::string path1 = "/home/zephyr/Programming/others/YechengRepo/Experiment/ControlPerformance/TestCases/NSweep/N5/Case0.txt";
    TaskSet tasks;
    VectorDynamic coeff;
    std::tie(tasks, coeff) = ReadControlCase(path1);
    std::vector<bool> maskForElimination(tasks.size(), false);
    maskForElimination[0] = 1;
    VectorDynamic initial = GenerateVectorDynamic(5);
    initial << 130.00,
        321,
        400,
        131,
        308;
    UpdateTaskSetPeriod(tasks, initial);
    gtsam::NonlinearFactorGraph graph = FactorGraphInManifold::BuildControlGraph(maskForElimination, tasks, coeff);
    auto initialEstimateFG = FactorGraphInManifold::GenerateInitialFG(tasks, maskForElimination);
    auto sth = graph.linearize(initialEstimateFG)->jacobian();
    MatrixDynamic jacobianCurr = sth.first;
    std::cout << "Current Jacobian matrix:" << endl;
    std::cout << jacobianCurr << endl;
    std::cout << "Current b vector: " << endl;
    std::cout << sth.second << endl;
    MatrixDynamic jacobianExpect = GenerateMatrixDynamic(8, 4);
    jacobianExpect(0, 0) = 0.18457;
    jacobianExpect(2, 1) = 0.16567;
    jacobianExpect(4, 2) = 0.34628;
    jacobianExpect(6, 3) = 0.38635;
    gtsam::assert_equal(jacobianExpect, jacobianCurr, 1e-3);
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
