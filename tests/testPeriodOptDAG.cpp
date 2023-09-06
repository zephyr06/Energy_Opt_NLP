#include "sources/ControlOptimization/ControlOptimizeNasri19.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/Tools/profilier.h"
using namespace rt_num_opt;
using namespace ControlOptimize;
using namespace std;
// TEST(RTA_Time, v1) {
//     int REPEAT = 20;
//     BeginTimer("main");
//     std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" +
//                        testDataSetName + ".yaml";
//     std::cout << "Optimization file path: " << path << "\n";
//     rt_num_opt::DAG_Nasri19 tasks_dag =
//     rt_num_opt::ReadDAGNasri19_Tasks(path); TaskSet tasks = tasks_dag.tasks_;
//     VectorDynamic coeff = ReadControlCoeff(path);
//     RTA_Nasri19 r(tasks_dag);
//     BeginTimer("RTA_PA_1");
//     for (int i = 0; i < REPEAT; i++) {
//         auto rta = r.ResponseTimeOfTaskSet();
//     }
//     std::cout << "Maximum RTA1: " << r.ResponseTimeOfTaskSet().maxCoeff()
//               << "\n";
//     EndTimer("RTA_PA_1");
//     tasks_dag.AssignPriorityControl(coeff);
//     RTA_Nasri19 r2(tasks_dag);
//     BeginTimer("RTA_PA_2");
//     for (int i = 0; i < REPEAT; i++) auto rta = r2.ResponseTimeOfTaskSet();
//     std::cout << "Maximum RTA2: " << r2.ResponseTimeOfTaskSet().maxCoeff()
//               << "\n";
//     EndTimer("RTA_PA_2");
//     EndTimer("main");
//     PrintTimer();
// }

TEST(case1, v1) {
    BeginTimer("main");

    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" +
                       testDataSetName + ".yaml";
    std::cout << "Optimization file path: " << path << "\n";
    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = tasks_dag.tasks_;
    VectorDynamic coeff = ReadControlCoeff(path);

    std::vector<bool> maskForElimination(tasks_dag.SizeDag(), false);

    auto sth =
        OptimizeTaskSetIterative<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19, RTA_Nasri19>(tasks_dag, coeff,
                                                           maskForElimination);

    // FindEliminatedVariables(tasks, maskForElimination);
    // AssertEqualVectorExact({true, false, false, false, false},
    // maskForElimination);
    cout << "Initial objective function is "
         << FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(
                rt_num_opt::ReadDAGNasri19_Tasks(path), coeff)
         << endl;
    RTA_Nasri19 r(tasks_dag);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    cout << "After optimization, the RTA vector is " << rta << endl;
    cout << "After optimization, the period vector is " << sth.first << endl;
    for (Task task : tasks_dag.tasks_) task.print();
    cout << "After optimization, the objective function is " << sth.second
         << endl;
    EndTimer("main");
    PrintTimer();
    std::cout << count1 << ", " << count2 << std::endl;
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
