#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
#include "sources/MatrixConvenient.h"
#include "sources/Tools/testMy.h"
using namespace rt_num_opt;
using namespace std;
TEST(RTA, V1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v20.yaml";
    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    RTA_Nasri19 r(tasks_dag);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    VectorDynamic rta_exp = rta;
    rta_exp << 500, 1500, 200, 100;
    EXPECT(gtsam::assert_equal(rta_exp, rta, 1e-3));
}
TEST(ControlObjFactor, v1) {
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
// require x>0
double BarrierReorder(int x) {
    if (x > 0)
        return log(x);
    else
        return 1e8 * -1 * x;
}
double GradientBarrierReorder(int x) {
    if (x > 0)
        return 1 / x;
    else
        return -1 * 1e8;
}
// obj_sorted = obj + w * Barrier(D - R)
// gradient = k_obj + w / (D-R)
struct TaskInfo {
    TaskInfo() {}
    TaskInfo(int i, double p) : task_index(i), priority(p) {}
    int task_index;
    double priority;
};
double GetObjGradient(int task_index, const TaskSet &tasks,
                      const VectorDynamic &coeff, const vector<double> &rta,
                      double weight) {
    return coeff(2 * task_index + 1) +
           weight / (tasks[task_index].deadline - rta.at(task_index));
}
//
double GetObjAfterAdjustPriority(const DAG_Nasri19 &tasks_dag,
                                 const VectorDynamic &coeff,
                                 const std::vector<TaskInfo> &priority_assign) {
    return 0;
}
std::vector<TaskInfo> ReorderWithGradient(const DAG_Nasri19 &tasks_dag,
                                          const VectorDynamic &coeff,
                                          const vector<double> &rta,
                                          double weight) {
    const TaskSet &tasks = tasks_dag.tasks_;
    std::vector<TaskInfo> tasks_w_pri(tasks.size());
    for (uint i = 0; i < tasks.size(); i++)
        tasks_w_pri[i] = TaskInfo(i, tasks[i].priority);
    sort(tasks_w_pri.begin(), tasks_w_pri.end(),
         [](const TaskInfo &t1, const TaskInfo &t2) {
             return t1.priority < t2.priority;
         });
    // DAG_Nasri19 tasks_dag_cur = tasks_dag;
    // tasks_dag_cur.UpdateTaskSet(tasks);
    double obj_base =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(tasks_dag, coeff);

    // begin adjusting priority based on new gradient information
    std::vector<TaskInfo> tasks_w_gra(tasks.size());
    for (uint i = 0; i < tasks.size(); i++)
        tasks_w_gra[i] =
            TaskInfo(i, GetObjGradient(i, tasks, coeff, rta, weight));
    sort(tasks_w_gra.begin(), tasks_w_gra.end(),
         [](const TaskInfo &t1, const TaskInfo &t2) {
             return t1.priority < t2.priority;
         });

    // adjust priority iteratively
    while (!tasks_w_gra.empty()) {
        int task_id_curr = tasks_w_gra.back().task_index;
        tasks_w_gra.pop_back();
        double obj_curr;
        while (true) {
            IncreasePriority(task_id_curr, tasks_w_pri);
            obj_curr =
                GetObjAfterAdjustPriority(tasks_dag, coeff, priority_assign);
            if (obj_curr >= obj_base) {
                DecreasePriority(task_id_curr, tasks_w_pri);
                break;
            } else
                obj_base = obj_curr;
        }
    }
}

TEST(PriorityAssignment, v1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v19.yaml";

    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    TaskSet tasks = tasks_dag.tasks_;
    VectorDynamic coeff = GenerateVectorDynamic(4 * 2);
    coeff << 1, 10, 2, 20, 3, 30, 4, 40;

    std::cout << "RTA analysis:\n";
    RTA_Nasri19 r(tasks_dag);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << rta << "\n";

    TaskSet tasksTry = tasks;
    VectorDynamic coeffTry = coeff;
    std::cout << "Before adjusting priority: \n";
    for (Task &task : tasks) {
        task.print();
    }
    ReorderWithGradient(tasksTry, coeffTry, 1);
    std::cout << "After adjusting priority: \n";
    for (Task &task : tasks) {
        task.print();
    }
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
