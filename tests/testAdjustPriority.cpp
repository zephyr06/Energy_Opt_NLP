#include "sources/ControlOptimization/AdjustPriority.h"
#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
#include "sources/MatrixConvenient.h"
#include "sources/Tools/testMy.h"
using namespace rt_num_opt;
using namespace std;
TEST(RTA, V1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v20.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    VectorDynamic rta = GetNasri19RTA(dag_tasks);
    VectorDynamic rta_exp = rta;
    rta_exp << 500, 1500, 200, 100;
    EXPECT(gtsam::assert_equal(rta_exp, rta, 1e-3));
}

TEST(PriorityAssignment, GetObjGradient) {
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
    EXPECT_DOUBLES_EQUAL(10 + weight / (5000 - 500),
                         GetObjGradient(0, tasks, coeff, rta, weight), 1e-6);
    EXPECT_DOUBLES_EQUAL(20 + weight / (5000 - 1500),
                         GetObjGradient(1, tasks, coeff, rta, weight), 1e-6);
    EXPECT_DOUBLES_EQUAL(30 + weight / (10000 - 200),
                         GetObjGradient(2, tasks, coeff, rta, weight), 1e-6);
    EXPECT_DOUBLES_EQUAL(40 + weight / (10000 - 100),
                         GetObjGradient(3, tasks, coeff, rta, weight), 1e-6);
}
TEST(PriorityAssignment, SortPriorityBasedGradient) {
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
    EXPECT_LONGS_EQUAL(2, tasks_w_gra[0].task_index);
    EXPECT_LONGS_EQUAL(3, tasks_w_gra[1].task_index);
    EXPECT_LONGS_EQUAL(0, tasks_w_gra[2].task_index);
    EXPECT_LONGS_EQUAL(1, tasks_w_gra[3].task_index);
}
TEST(PriorityAssignment, GetPriorityVector) {
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
        ReorderWithGradient(dag_tasks, coeff, weight);
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
        ReorderWithGradient(dag_tasks, coeff, weight);
    std::cout << "New RTA:\n";
    DAG_Nasri19 dag_tasks_curr = dag_tasks;
    for (uint i = 0; i < priority_vec.size(); i++) {
        dag_tasks_curr.tasks_[priority_vec[i].task_index].priority = i;
    }
    RTA_Nasri19 r1(dag_tasks_curr);
    rta = r1.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";
    EXPECT_LONGS_EQUAL(200, rta(2));
    EXPECT_LONGS_EQUAL(300, rta(3));
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
