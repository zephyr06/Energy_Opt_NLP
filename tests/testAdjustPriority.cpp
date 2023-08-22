#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
#include "sources/MatrixConvenient.h"
#include "sources/Tools/testMy.h"
using namespace rt_num_opt;
using namespace std;
TEST(RTA, V1) {
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v20.yaml";
    rt_num_opt::DAG_Nasri19 dag_tasks = rt_num_opt::ReadDAGNasri19_Tasks(path);
    RTA_Nasri19 r(dag_tasks);
    VectorDynamic rta = r.ResponseTimeOfTaskSet();
    VectorDynamic rta_exp = rta;
    rta_exp << 500, 1500, 200, 100;
    EXPECT(gtsam::assert_equal(rta_exp, rta, 1e-3));
}

// require x>0
double BarrierReorder(double x) {
    if (x > 0)
        return log(x);
    else
        return 1e8 * -1 * x;
}
double GradientBarrierReorder(double x) {
    if (x > 0)
        return 1 / x;
    else
        return -1 * 1e8;
}
// obj_sorted = obj + w * Barrier(D - R)
// gradient = k_obj + w / (D-R)
struct TaskPriority {
    TaskPriority() {}
    TaskPriority(int i, double p) : task_index(i), priority(p) {}
    int task_index;
    double priority;
};
double GetObjGradient(int task_index, const TaskSet &tasks,
                      const VectorDynamic &coeff, const VectorDynamic &rta,
                      double weight) {
    return coeff(2 * task_index + 1) +
           weight / (tasks[task_index].deadline - rta(task_index));
}
// TODO: how to describe priority assignments more elegant?
double GetObjAfterAdjustPriority(
    const DAG_Nasri19 &dag_tasks, const VectorDynamic &coeff,
    const std::vector<TaskPriority> &priority_assign) {
    DAG_Nasri19 dag_tasks_curr = dag_tasks;
    for (uint i = 0; i < priority_assign.size(); i++) {
        dag_tasks_curr.tasks_[priority_assign[i].task_index].priority = i;
    }

    return FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks_curr,
                                                               coeff);
}

void IncreasePriority(int task_index, std::vector<TaskPriority> &tasks_w_pri) {
    // if task_index has highest priority, return
    if (tasks_w_pri[0].task_index == task_index)
        return;
    for (uint i = 1; i < tasks_w_pri.size(); i++) {
        if (tasks_w_pri[i].task_index == task_index) {
            auto temp = tasks_w_pri[i - 1];
            tasks_w_pri[i - 1] = tasks_w_pri[i];
            tasks_w_pri[i] = temp;
            return;
        }
    }
    CoutError("Didn't find the given task_index: " + to_string(task_index));
}
void DecreasePriority(int task_index, std::vector<TaskPriority> &tasks_w_pri) {
    // if task_index has lowest priority, return
    if (tasks_w_pri.back().task_index == task_index)
        return;
    for (int i = static_cast<int>(tasks_w_pri.size()) - 2; i >= 0; i--) {
        if (tasks_w_pri[i].task_index == task_index) {
            auto temp = tasks_w_pri[i + 1];
            tasks_w_pri[i + 1] = tasks_w_pri[i];
            tasks_w_pri[i] = temp;
            return;
        }
    }
    CoutError("Didn't find the given task_index: " + to_string(task_index));
}

std::vector<TaskPriority> GetPriorityVector(const DAG_Nasri19 &dag_tasks) {
    const TaskSet &tasks = dag_tasks.tasks_;
    std::vector<TaskPriority> tasks_w_pri(tasks.size());
    for (uint i = 0; i < tasks.size(); i++)
        tasks_w_pri[i] = TaskPriority(i, tasks[i].priority);
    sort(tasks_w_pri.begin(), tasks_w_pri.end(),
         [](const TaskPriority &t1, const TaskPriority &t2) {
             if (t1.priority != t2.priority)
                 return t1.priority < t2.priority;
             else
                 return t1.task_index < t2.task_index;
             //  CoutError("Should not happen!");
             //  return true;
         });
    return tasks_w_pri;
}
// NOTE: in this special function, for the convenince of manipulation,
// lowest priority tasks appear first, highest priority task appear later
std::vector<TaskPriority> SortPriorityBasedGradient(const TaskSet &tasks,
                                                    const VectorDynamic &coeff,
                                                    const VectorDynamic &rta,
                                                    double weight) {
    std::vector<TaskPriority> tasks_w_gra(tasks.size());
    for (uint i = 0; i < tasks.size(); i++)
        tasks_w_gra[i] =
            TaskPriority(i, GetObjGradient(i, tasks, coeff, rta, weight));
    sort(tasks_w_gra.begin(), tasks_w_gra.end(),
         [](const TaskPriority &t1, const TaskPriority &t2) {
             return t1.priority < t2.priority;
         });
    return tasks_w_gra;
}

// higher priority tasks appear first, lower priority task appear later
std::vector<TaskPriority> ReorderWithGradient(const DAG_Nasri19 &dag_tasks,
                                              const VectorDynamic &coeff,
                                              const VectorDynamic &rta,
                                              double weight) {
    const TaskSet &tasks = dag_tasks.tasks_;
    std::vector<TaskPriority> tasks_w_pri = GetPriorityVector(dag_tasks);
    double obj_base =
        FactorGraphNasri<DAG_Nasri19, RTA_Nasri19>::RealObj(dag_tasks, coeff);

    std::vector<TaskPriority> tasks_w_gra =
        SortPriorityBasedGradient(tasks, coeff, rta, weight);

    // adjust priority iteratively
    while (!tasks_w_gra.empty()) {
        int task_id_curr = tasks_w_gra.back().task_index;
        tasks_w_gra.pop_back();
        double obj_curr;
        while (true) {
            IncreasePriority(task_id_curr, tasks_w_pri);
            obj_curr = GetObjAfterAdjustPriority(dag_tasks, coeff, tasks_w_pri);
            if (obj_curr >= obj_base) {  // if IncreasePriority happens
                if (tasks_w_pri[0].task_index != task_id_curr)
                    DecreasePriority(task_id_curr, tasks_w_pri);
                break;
            } else {
                if (debugMode == 1) {
                    std::cout << "Priority change: task " << task_id_curr
                              << "'s priority increases, and improves the "
                                 "objective function from "
                              << obj_base << " to " << obj_curr << "\n";
                }
                obj_base = obj_curr;
            }
        }
    }
    return tasks_w_pri;
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
        ReorderWithGradient(dag_tasks, coeff, rta, weight);
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
        ReorderWithGradient(dag_tasks, coeff, rta, weight);
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
