#include "sources/ControlOptimization/ControlOptimizeNasri19.h"
#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
#include "sources/MatrixConvenient.h"
#include "sources/Tools/testMy.h"
using namespace rt_num_opt;
using namespace std;

TEST(ControlObjFactor, v1) {
    core_m_dag = 4;
    PeriodRoundQuantum = 1e3;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/"
        "test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = tasks_dag.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff.setOnes();
    for (uint i = 0; i < coeff.rows() / 2; i++)
        coeff(2 * i + 1, 0) = coeff(2 * i + 1, 0) * 10;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(tasks_dag);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";

    std::vector<bool> maskForElimination(tasks_dag.SizeDag(), false);

    FactorGraphNasri<DAG_Nasri19,
                     RTA_Nasri19>::ControlObjFactor control_obj_factor =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::GenerateControlObjFactor(
            maskForElimination, coeff, tasks_dag);

    gtsam::Values initial_estimate =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::GenerateInitialFG(
            tasks_dag, maskForElimination);

    gtsam::Vector error_actual =
        control_obj_factor.unwhitenedError(initial_estimate);
    std::cout << "Actual error: " << error_actual << "\n";
    VectorDynamic error_expect = error_actual;
    error_expect(0) = pow(5000 + 500 * 10, 0.5);
    error_expect(2) = pow(5000 + 1500 * 10, 0.5);
    error_expect(4) = pow(10000 + 200 * 10, 0.5);
    error_expect(6) = pow(10000 + 100 * 10, 0.5);
    EXPECT(gtsam::assert_equal(error_expect, error_actual, 1e-3));

    std::vector<gtsam::Matrix> H;
    H.push_back(GenerateMatrixDynamic(1, 1));
    H.push_back(GenerateMatrixDynamic(1, 1));
    control_obj_factor.unwhitenedError(initial_estimate, H);
    gtsam::Matrix jacob_expect1 = GenerateVectorDynamic(8);
    jacob_expect1 << 0.005, 0, 0.00353553, 0, 0, 0, 0, 0;
    EXPECT(gtsam::assert_equal(jacob_expect1, H[0], 1e-3));
    gtsam::Matrix jacob_expect2 = GenerateVectorDynamic(8);
    jacob_expect2 << 0, 0, 0, 0, 0.00456435, 0, 0.00476731, 0;
    EXPECT(gtsam::assert_equal(jacob_expect2, H[1], 1e-3));
}

TEST(ControlObjFactor, obj) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = tasks_dag.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff.setOnes();
    for (uint i = 0; i < coeff.rows() / 2; i++)
        coeff(2 * i + 1, 0) = coeff(2 * i + 1, 0) * 10;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(tasks_dag);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";

    double obj_exp =
        5000 + 5000 + 10000 + 10000 + 10 * (500 + 1500 + 200 + 100);
    double obj_actual =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(tasks_dag, coeff);
    EXPECT_LONGS_EQUAL(obj_exp, obj_actual);
}

TEST(FindEliminatedVariables, V1) {
    core_m_dag = 4;
    disturb_init = 100;
    PeriodRoundQuantum = 500;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    tasks_dag.UpdatePeriod(0, 1500);
    EXPECT_LONGS_EQUAL(2000, tasks_dag.tasks_[0].period);

    std::vector<bool> maskForElimination(tasks_dag.SizeDag(), false);
    FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::FindEliminatedVariables(
        tasks_dag, maskForElimination);
    EXPECT_LONGS_EQUAL(2, maskForElimination.size());
    EXPECT(maskForElimination[0]);
    EXPECT(!maskForElimination[1]);
}

TEST(OptimizeTaskSetIterative, V1) {
    using namespace rt_num_opt;
    using namespace ControlOptimize;
    core_m_dag = 1;
    PeriodRoundQuantum = 500;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v24.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic coeff = ReadControlCoeff(path);
    std::vector<bool> maskForElimination(dag_tasks.SizeDag(), false);
    auto sth =
        OptimizeTaskSetIterative<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19, RTA_Nasri19>(dag_tasks, coeff,
                                                           maskForElimination);
    // EXPECT_LONGS_EQUAL(PeriodRoundQuantum, sth.first(0));  // or 3000 if
    // PeriodRoundQuantum = 500;
    EXPECT(sth.first(0) <= 4000);
}

TEST(OptimizeTaskSetIterative, V2) {
    using namespace rt_num_opt;
    using namespace ControlOptimize;
    core_m_dag = 2;
    PeriodRoundQuantum = 500;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v24.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    // dag_tasks.InitializePriority();
    for (Task &task_curr : dag_tasks.tasks_) task_curr.priority = task_curr.id;
    dag_tasks.UpdateTasksVecNasri_();
    VectorDynamic coeff = ReadControlCoeff(path);
    std::vector<bool> maskForElimination(dag_tasks.SizeDag(), false);
    auto sth =
        OptimizeTaskSetIterative<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19, RTA_Nasri19>(dag_tasks, coeff,
                                                           maskForElimination);

    EXPECT(sth.first(0) <=
           3500 + PeriodRoundQuantum);  // all the nodes of one DAG are assigned
                                        // to one core
}

TEST(OptimizeTaskSetIterative, V3) {
    using namespace rt_num_opt;
    using namespace ControlOptimize;
    core_m_dag = 2;
    PeriodRoundQuantum = 500;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v25.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    // dag_tasks.InitializePriority();
    for (Task &task_curr : dag_tasks.tasks_) task_curr.priority = task_curr.id;
    dag_tasks.UpdateTasksVecNasri_();
    VectorDynamic coeff = ReadControlCoeff(path);
    std::vector<bool> maskForElimination(dag_tasks.SizeDag(), false);
    auto sth =
        OptimizeTaskSetIterative<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19, RTA_Nasri19>(dag_tasks, coeff,
                                                           maskForElimination);
    // EXPECT_LONGS_EQUAL(1500 + PeriodRoundQuantum,
    //                    sth.first(0));  // two DAGs will be assigned to two
    //                    cores
    EXPECT_LONGS_EQUAL(2000, sth.first(0));
}

TEST(GetTotalJobsWithinHyperPeriod, V1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v29.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    EXPECT_LONGS_EQUAL(11, dag_tasks.GetTotalJobsWithinHyperPeriod());
}
TEST(GetTotalJobsWithinHyperPeriod, V2) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v29.yaml";

    PeriodRoundQuantum = 500;
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    // dag_tasks.UpdatePeriod(0, 3500);
    EXPECT_LONGS_EQUAL(1 * 5 + 1 * (5 + 1),
                       dag_tasks.GetTotalJobsWithinHyperPeriod());
}

TEST(RTA, V2) {
        Period_Round_For_Control_Opt = 0;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    PeriodRoundQuantum = 1000;
    enableReorder = 1;
    core_m_dag = 1;
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic coeff = ReadControlCoeff(path);
    std::vector<TaskPriority> pa = {TaskPriority(3, 0), TaskPriority(2, 0),
                                    TaskPriority(0, 0), TaskPriority(1, 0)};
    UpdateAllTasksPriority(dag_tasks, pa);

    VectorDynamic periods = GenerateVectorDynamic(2);
    // periods << 2802.82, 7556.948;
    periods << 4000, 8000;
    FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::UpdateTaskSetWithPeriodVariable(
        dag_tasks, periods);
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << "RTA: \n" << rta << "\n";
    EXPECT_LONGS_EQUAL(100, rta(3));
}
using namespace ControlOptimize;
TEST(OverallOptimization, V1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    PeriodRoundQuantum = 1000;
    enableReorder = 1;
    core_m_dag = 1;
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic coeff = ReadControlCoeff(path);

    std::vector<bool> maskForElimination(dag_tasks.SizeDag(), false);
    auto sth =
        OptimizeTaskSetIterative<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19, RTA_Nasri19>(dag_tasks, coeff,
                                                           maskForElimination);
    // EXPECT_LONGS_EQUAL(2000, sth.first(0));
    // EXPECT_LONGS_EQUAL(2000, sth.first(1));
    // EXPECT_LONGS_EQUAL(1000, sth.first(2));
    // EXPECT_LONGS_EQUAL(1000, sth.first(3));
    // double optimal_ratio = after_opt / initial_opt;
    EXPECT(sth.second < 0.65);
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
