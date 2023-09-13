#include "sources/ControlOptimization/AdjustPriority.h"
#include "sources/ControlOptimization/ControlOptimizeNasri19.h"
#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
#include "sources/MatrixConvenient.h"
#include "sources/Tools/testMy.h"
using namespace rt_num_opt;
using namespace ControlOptimize;
using namespace std;
TEST(RTA, V1) {
    Obj_Pow = 1;
    Period_Round_For_Control_Opt = 0;
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v20.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    VectorDynamic rta_exp = rta;
    rta_exp << 500, 1500, 200, 100;
    EXPECT(gtsam::assert_equal(rta_exp, rta, 1e-3));
}

TEST(PriorityAssignment, GetObjGradient) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = dag_tasks.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    double weight = -10;
    EXPECT_DOUBLES_EQUAL(10 - weight / (5000 - 500),
                         GetObjGradient(0, tasks, coeff, rta, weight), 1e-6);
    EXPECT_DOUBLES_EQUAL(20 - weight / (5000 - 1500),
                         GetObjGradient(1, tasks, coeff, rta, weight), 1e-6);
    EXPECT_DOUBLES_EQUAL(30 - weight / (10000 - 200),
                         GetObjGradient(2, tasks, coeff, rta, weight), 1e-6);
    EXPECT_DOUBLES_EQUAL(40 - weight / (10000 - 100),
                         GetObjGradient(3, tasks, coeff, rta, weight), 1e-6);
}
TEST(PriorityAssignment, SortPriorityBasedGradient) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = dag_tasks.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    double weight = -10;
    std::vector<TaskPriority> tasks_w_gra =
        SortPriorityBasedGradient(tasks, coeff, rta, weight);
    EXPECT_LONGS_EQUAL(0, tasks_w_gra[0].task_index);
    EXPECT_LONGS_EQUAL(1, tasks_w_gra[1].task_index);
    EXPECT_LONGS_EQUAL(2, tasks_w_gra[2].task_index);
    EXPECT_LONGS_EQUAL(3, tasks_w_gra[3].task_index);
    weight = 1e6;
    tasks_w_gra = SortPriorityBasedGradient(tasks, coeff, rta, weight);
    EXPECT_LONGS_EQUAL(1, tasks_w_gra[0].task_index);
    EXPECT_LONGS_EQUAL(0, tasks_w_gra[1].task_index);
    EXPECT_LONGS_EQUAL(2, tasks_w_gra[2].task_index);
    EXPECT_LONGS_EQUAL(3, tasks_w_gra[3].task_index);
}
TEST(PriorityAssignment, GetPriorityVector) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = dag_tasks.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    dag_tasks.InitializePriority();
    std::vector<TaskPriority> tasks_w_pri = GetPriorityVector(dag_tasks);
    EXPECT_LONGS_EQUAL(0, tasks_w_pri[0].task_index);
    EXPECT_LONGS_EQUAL(1, tasks_w_pri[1].task_index);
    EXPECT_LONGS_EQUAL(2, tasks_w_pri[2].task_index);
    EXPECT_LONGS_EQUAL(3, tasks_w_pri[3].task_index);
}
TEST(PriorityAssignment, In_De_creasePriority) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = dag_tasks.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    dag_tasks.InitializePriority();
    std::vector<TaskPriority> tasks_w_pri = GetPriorityVector(dag_tasks);
    IncreasePriority(0, tasks_w_pri);
    EXPECT_LONGS_EQUAL(0, tasks_w_pri[0].task_index);
    IncreasePriority(1, tasks_w_pri);
    EXPECT_LONGS_EQUAL(0, tasks_w_pri[1].task_index);
    EXPECT_LONGS_EQUAL(1, tasks_w_pri[0].task_index);
    DecreasePriority(1, tasks_w_pri);
    EXPECT_LONGS_EQUAL(0, tasks_w_pri[0].task_index);
    EXPECT_LONGS_EQUAL(1, tasks_w_pri[1].task_index);

    IncreasePriority(3, tasks_w_pri);
    EXPECT_LONGS_EQUAL(2, tasks_w_pri[3].task_index);
    EXPECT_LONGS_EQUAL(3, tasks_w_pri[2].task_index);
    DecreasePriority(3, tasks_w_pri);
    EXPECT_LONGS_EQUAL(3, tasks_w_pri[3].task_index);
    EXPECT_LONGS_EQUAL(2, tasks_w_pri[2].task_index);
}

TEST(PriorityAssignment, ReorderWithGradient_v1) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = dag_tasks.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    double weight = 0;
    std::vector<TaskPriority> priority_vec =
        ReorderWithGradient(dag_tasks, coeff, weight, -1);
    // no changes because the RTA cannot be improved under any priority
    // assignments
    EXPECT_LONGS_EQUAL(0, priority_vec[0].task_index);
    EXPECT_LONGS_EQUAL(1, priority_vec[1].task_index);
    EXPECT_LONGS_EQUAL(2, priority_vec[2].task_index);
    EXPECT_LONGS_EQUAL(3, priority_vec[3].task_index);
}
TEST(PriorityAssignment, ReorderWithGradient_v2) {
    core_m_dag = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = dag_tasks.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    double weight = -10;
    std::vector<TaskPriority> priority_vec =
        ReorderWithGradient(dag_tasks, coeff, weight, -1);
    std::cout << "New RTA:\n";
    DAG_Nasri19 dag_tasks_curr = dag_tasks;
    for (uint i = 0; i < priority_vec.size(); i++) {
        dag_tasks_curr.tasks_[priority_vec[i].task_index].priority = i;
    }
    RTA_Nasri19 r1(dag_tasks_curr);
    rta = r1.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    EXPECT_LONGS_EQUAL(300, rta(2));
    EXPECT_LONGS_EQUAL(100, rta(3));
}

TEST(PriorityAssignment, ReorderWithGradient_v3) {
    core_m_dag = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = dag_tasks.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    double weight = -10;
    std::vector<TaskPriority> priority_vec =
        ReorderWithGradient(dag_tasks, coeff, weight, -1);

    EXPECT_LONGS_EQUAL(3, priority_vec[0].task_index);
    EXPECT_LONGS_EQUAL(2, priority_vec[1].task_index);
    EXPECT_LONGS_EQUAL(0, priority_vec[2].task_index);
    EXPECT_LONGS_EQUAL(1, priority_vec[3].task_index);
}
TEST(ReadControlCoeff, dag_v1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    VectorDynamic coeff_actual = ReadControlCoeff(path);
    VectorDynamic coeff_expected = GenerateVectorDynamic(8);
    coeff_expected << 978, 4531, 719, 4916, 480, 8022, 755, 7236;
    EXPECT(gtsam::assert_equal(coeff_expected, coeff_actual));
}

TEST(ExtractDAGPeriodVec, V1) {
    PeriodRoundQuantum = 1e3;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    vector<bool> maskForElimination(2);
    maskForElimination[0] = true;
    dag_tasks.AdjustPeriod(1, -2000);
    gtsam::Values result =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::GenerateInitialFG(
            dag_tasks, maskForElimination);
    VectorDynamic periodVec =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::ExtractDAGPeriodVec(
            result, dag_tasks);
    EXPECT_LONGS_EQUAL(5000, periodVec(0));
    EXPECT_LONGS_EQUAL(8000, periodVec(1));
}
TEST(ExtractDAGPeriodVec, V2) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    vector<bool> maskForElimination(2);
    maskForElimination[0] = true;
    dag_tasks.AdjustPeriod(0, -2000);
    gtsam::Values result =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::GenerateInitialFG(
            dag_tasks, maskForElimination);
    VectorDynamic periodVec =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::ExtractDAGPeriodVec(
            result, dag_tasks);
    // EXPECT_LONGS_EQUAL(3000, periodVec(0));
    EXPECT(periodVec(0) <= 4000);
    EXPECT_LONGS_EQUAL(10000, periodVec(1));
}

TEST(IsHyperPeriodOverflow, V1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v28.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    EXPECT(!dag_tasks.IsHyperPeriodOverflow());
    dag_tasks.UpdatePeriod(0, 1.07229123e+09);
    EXPECT(!dag_tasks.IsHyperPeriodOverflow());
    dag_tasks.UpdatePeriod(1, 1.08521321e+09);
    EXPECT(dag_tasks.IsHyperPeriodOverflow());
}

TEST(GetSchedulabilityObj, v1) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v20.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    VectorDynamic rta_exp = rta;
    rta_exp << 500, 1500, 200, 100;
    EXPECT(gtsam::assert_equal(rta_exp, rta, 1e-3));
    weight_priority_assignment = 1;
    EXPECT_DOUBLES_EQUAL(log(3500) + log(2500) + log(7800) + log(7900),
                         GetSchedulabilityObj(dag_tasks, rta, 1), 1e-3);
}
TEST(GetSchedulabilityObj, v2) {
    core_m_dag = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v20.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    dag_tasks.UpdatePeriod(0, 100);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    EXPECT(GetSchedulabilityObj(dag_tasks, rta, 1) < -1e49);
}

TEST(SortPriorityBasedGradient, v1) {
    core_m_dag = 4;
    Obj_Pow = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    VectorDynamic rta_exp = rta;
    rta_exp << 500, 1500, 200, 100;
    EXPECT(gtsam::assert_equal(rta_exp, rta, 1e-3));
    weight_priority_assignment = 1;

    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;
    std::vector<TaskPriority> tasks_w_gra = SortPriorityBasedGradient(
        dag_tasks.tasks_, coeff, rta, weight_priority_assignment);
    // four tasks' gradient should be
    EXPECT_LONGS_EQUAL(3, tasks_w_gra[3].task_index);
    // EXPECT_DOUBLES_EQUAL(40 - weight_priority_assignment / (10000 - 100),
    //                      tasks_w_gra[3].priority, 1e-3);
}

TEST(UpdateAllTasksPriority, v1) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    VectorDynamic rta_exp = rta;
    rta_exp << 500, 1500, 200, 100;
    EXPECT(gtsam::assert_equal(rta_exp, rta, 1e-3));
    weight_priority_assignment = 0;

    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;
    // obtain priority assignments based on coeff
    std::vector<TaskPriority> tasks_w_p =
        ReorderWithGradient(dag_tasks, coeff, 0, -1);
    UpdateAllTasksPriority(dag_tasks, tasks_w_p);
    EXPECT(!UpdateAllTasksPriority(dag_tasks, tasks_w_p));
    IncreasePriority(2, tasks_w_p);
    EXPECT(UpdateAllTasksPriority(dag_tasks, tasks_w_p));
}
TEST(AssignPriorityBasedRM, v1) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    dag_tasks.AssignPriorityRM();
    EXPECT_LONGS_EQUAL(5000, dag_tasks.tasks_[0].priority);
    EXPECT_LONGS_EQUAL(5000, dag_tasks.tasks_[1].priority);
    EXPECT_LONGS_EQUAL(10000, dag_tasks.tasks_[2].priority);
    EXPECT_LONGS_EQUAL(10000, dag_tasks.tasks_[3].priority);
}
TEST(AssignPriorityBasedCoeff, v1) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic coeff = ReadControlCoeff(path);
    dag_tasks.AssignPriorityControl(coeff);
    EXPECT_LONGS_EQUAL(10000 - 4531, dag_tasks.tasks_[0].priority);
    EXPECT_LONGS_EQUAL(10000 - 4916, dag_tasks.tasks_[1].priority);
    EXPECT_LONGS_EQUAL(10000 - 8022, dag_tasks.tasks_[2].priority);
    EXPECT_LONGS_EQUAL(10000 - 7236, dag_tasks.tasks_[3].priority);
}
TEST(RoundPeriod, v1) {
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    EXPECT_LONGS_EQUAL(5000, dag_tasks.tasks_[0].period);
}
TEST(RoundPeriod, v2) {
    Period_Round_For_Control_Opt = 1;
    core_m_dag = 4;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    dag_tasks.UpdatePeriod(1, 300);
    EXPECT_LONGS_EQUAL(300, dag_tasks.tasks_[2].period);
    EXPECT_LONGS_EQUAL(300, dag_tasks.tasks_[3].period);
    // dag_tasks.UpdatePeriod(1, 800);
    // EXPECT_LONGS_EQUAL(1000, dag_tasks.tasks_[2].period);
    // EXPECT_LONGS_EQUAL(1000, dag_tasks.tasks_[3].period);
    // dag_tasks.UpdatePeriod(0, 1200);
    // EXPECT_LONGS_EQUAL(2000, dag_tasks.tasks_[0].period);
    // EXPECT_LONGS_EQUAL(2000, dag_tasks.tasks_[1].period);
    // dag_tasks.UpdatePeriod(0, 2100);
    // EXPECT_LONGS_EQUAL(4000, dag_tasks.tasks_[0].period);
    // EXPECT_LONGS_EQUAL(4000, dag_tasks.tasks_[1].period);
    // dag_tasks.UpdatePeriod(0, 3500);
    // EXPECT_LONGS_EQUAL(4000, dag_tasks.tasks_[0].period);
    // EXPECT_LONGS_EQUAL(4000, dag_tasks.tasks_[1].period);
}

TEST(FindTaskIndexFromPAOrder, V1) {
    Priority_assignment_adjustment_threshold = 0;
    core_m_dag = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic coeff = ReadControlCoeff(path);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    dag_tasks.InitializePriority();
    std::vector<TaskPriority> tasks_w_pri = GetPriorityVector(dag_tasks);
    EXPECT_LONGS_EQUAL(0, FindTaskIndexFromPAOrder(0, tasks_w_pri));
    EXPECT_LONGS_EQUAL(1, FindTaskIndexFromPAOrder(1, tasks_w_pri));
    EXPECT_LONGS_EQUAL(2, FindTaskIndexFromPAOrder(2, tasks_w_pri));
    EXPECT_LONGS_EQUAL(3, FindTaskIndexFromPAOrder(3, tasks_w_pri));
}
TEST(ReorderWithGradient, significant_difference) {
    Priority_assignment_adjustment_threshold = 0;
    core_m_dag = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic coeff = ReadControlCoeff(path);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    dag_tasks.InitializePriority();
    std::vector<TaskPriority> tasks_w_pri = GetPriorityVector(dag_tasks);
    EXPECT_LONGS_EQUAL(0, tasks_w_pri[0].task_index);
    EXPECT_LONGS_EQUAL(1, tasks_w_pri[1].task_index);
    EXPECT_LONGS_EQUAL(2, tasks_w_pri[2].task_index);
    EXPECT_LONGS_EQUAL(3, tasks_w_pri[3].task_index);

    double weight = -10;
    std::vector<TaskPriority> tasks_w_gra =
        SortPriorityBasedGradient(dag_tasks.tasks_, coeff, rta, weight);
    EXPECT_LONGS_EQUAL(0, tasks_w_gra[0].task_index);
    EXPECT_LONGS_EQUAL(1, tasks_w_gra[1].task_index);
    EXPECT_LONGS_EQUAL(3, tasks_w_gra[2].task_index);
    EXPECT_LONGS_EQUAL(2, tasks_w_gra[3].task_index);

    // actual PA: 0 1 2 3
    // Gradie PA: 2 3 1 0
    EXPECT(!WhetherTaskNeedPAChange(0, tasks_w_pri, tasks_w_gra, 0));
    EXPECT(!WhetherTaskNeedPAChange(0, tasks_w_pri, tasks_w_gra, 1));
    EXPECT(!WhetherTaskNeedPAChange(0, tasks_w_pri, tasks_w_gra, 2));

    EXPECT(!WhetherTaskNeedPAChange(1, tasks_w_pri, tasks_w_gra, 0));

    EXPECT(WhetherTaskNeedPAChange(2, tasks_w_pri, tasks_w_gra, 0));
    EXPECT(WhetherTaskNeedPAChange(2, tasks_w_pri, tasks_w_gra, 1));
    EXPECT(!WhetherTaskNeedPAChange(2, tasks_w_pri, tasks_w_gra, 2));
    EXPECT(!WhetherTaskNeedPAChange(2, tasks_w_pri, tasks_w_gra, 3));

    EXPECT(WhetherTaskNeedPAChange(3, tasks_w_pri, tasks_w_gra, 0));
    EXPECT(WhetherTaskNeedPAChange(3, tasks_w_pri, tasks_w_gra, 1));
    EXPECT(!WhetherTaskNeedPAChange(3, tasks_w_pri, tasks_w_gra, 2));
    EXPECT(!WhetherTaskNeedPAChange(3, tasks_w_pri, tasks_w_gra, 3));
}

TEST(InitializePriorityAssignment, V1) {
    core_m_dag = 2;
    Obj_Pow = 2;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n10_v11.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic coeff = ReadControlCoeff(path);
    double err = 0;
    InitializePriorityAssignment<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19>(dag_tasks, coeff, 2);
    err = FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks, coeff);
    EXPECT(err >= 2 * 1.84e10 && err <= 2 * 1.86e10);

    InitializePriorityAssignment<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19>(dag_tasks, coeff, 3);
    err = FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks, coeff);
    EXPECT(err >= 2 * 6.73e9 && err <= 2 * 6.75e9);

    InitializePriorityAssignment<FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>,
                                 DAG_Nasri19>(dag_tasks, coeff, 1);
    err = FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks, coeff);
    EXPECT(err >= 2 * 6.73e9 && err <= 2 * 6.75e9);
}
int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
