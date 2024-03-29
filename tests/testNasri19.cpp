
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <yaml-cpp/yaml.h>

#include "sources/ControlOptimization/AdjustPriority.h"
#include "sources/ControlOptimization/FactorGraph_Nasri19.h"
#include "sources/MatrixConvenient.h"
#include "sources/RTA/RTA_Nasri19.h"
#include "sources/TaskModel/DAG_Nasri19.h"
#include "sources/TaskModel/DAG_Task.h"
#include "sources/TaskModel/ReadWriteYaml.h"
#include "sources/Tools/testMy.h"
#include "sources/Utils/Parameters.h"
using namespace rt_num_opt;
using namespace std;
// TEST(io, read_vector) {
//     rt_num_opt::PeriodRoundQuantum = 1;
//     rt_num_opt::ReadVec("PeriodSetAM");
//     for (auto x : rt_num_opt::PeriodSetAM) {
//         std::cout << x << " ";
//     }
//     std::cout << std::endl;
//     // AssertEqualVectorExact({1, 2, 5, 10, 20, 50, 100, 200, 1000},
//     // rt_num_opt::PeriodSetAM);
// }

TEST(IO, v1) {
    rt_num_opt::PeriodRoundQuantum = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    YAML::Node config = YAML::LoadFile(path);
    YAML::Node tasksNode = config["tasks"];
    EXPECT_LONGS_EQUAL(20, tasksNode[0]["period"].as<int>());
    EXPECT_LONGS_EQUAL(20, tasksNode[0]["deadline"].as<long int>());

    YAML::Node vert = tasksNode[0]["vertices"];
    EXPECT_LONGS_EQUAL(4, vert[1]["executionTime"].as<int>());

    // std::ofstream fout("fileUpdate.yaml");
    // fout << config;
    // fout.close();
}
TEST(io, v2) {
    rt_num_opt::PeriodRoundQuantum = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    auto dags = rt_num_opt::ReadDAG_NasriFromYaml(path);
    EXPECT_LONGS_EQUAL(4, dags[0].tasks_[3].executionTimeOrg);
    EXPECT_LONGS_EQUAL(3, dags[0].tasks_[3].id);
    EXPECT_LONGS_EQUAL(30, dags[1].tasks_[0].period);
    EXPECT_LONGS_EQUAL(30, dags[1].tasks_[1].deadline);
    EXPECT_LONGS_EQUAL(40, dags[2].tasks_[2].deadline);
    EXPECT_LONGS_EQUAL(3, dags[2].tasks_[2].executionTime);
}
TEST(io, v3) {
    rt_num_opt::PeriodRoundQuantum = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    auto dags = rt_num_opt::ReadDAG_NasriFromYaml(path);
    rt_num_opt::VectorDynamic dummy = rt_num_opt::GenerateVectorDynamic1D(0);
    rt_num_opt::WriteDAG_NasriToYaml(
        dags, "/home/zephyr/Programming/Energy_Opt_NLP/build/testOutput1.yaml",
        dummy);
}

TEST(io, no_edge) {
    rt_num_opt::PeriodRoundQuantum = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v23.csv";
    auto dag = rt_num_opt::ReadDAG_Task(path);
    std::vector<rt_num_opt::DAG_Model> dags;
    dags.push_back(dag);
    std::string outputPath =
        "/home/zephyr/Programming/Energy_Opt_NLP/build/testOutput2.yaml";
    rt_num_opt::VectorDynamic dummy = rt_num_opt::GenerateVectorDynamic1D(0);
    rt_num_opt::WriteDAG_NasriToYaml(dags, outputPath, dummy);
    auto dagsRead = rt_num_opt::ReadDAG_NasriFromYaml(outputPath);
    EXPECT_LONGS_EQUAL(200, dagsRead[0].tasks_[0].period);
    EXPECT_LONGS_EQUAL(0, boost::num_edges(dag.graph_));
    EXPECT_LONGS_EQUAL(0, boost::num_edges(dagsRead[0].graph_));
}
TEST(io, Nasri) {
    rt_num_opt::PeriodRoundQuantum = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    std::vector<rt_num_opt::DAG_Model> dags =
        rt_num_opt::ReadDAG_NasriFromYaml(path);
    rt_num_opt::DAG_Nasri19 dagNasri(dags);
    dagNasri.tasks_[0].executionTime = 11;
    auto str = dagNasri.ConvertTasksetToCsv();
}

TEST(rta, Nasri) {
    rt_num_opt::core_m_dag = 3;
    rt_num_opt::PeriodRoundQuantum = 1;
    rt_num_opt::Period_Round_For_Control_Opt = 0;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    std::vector<rt_num_opt::DAG_Model> dags =
        rt_num_opt::ReadDAG_NasriFromYaml(path);
    rt_num_opt::DAG_Nasri19 dagNasri(dags);
    auto str = dagNasri.ConvertTasksetToCsv();
    rt_num_opt::RTA_Nasri19 r(dagNasri);
    rt_num_opt::VectorDynamic rta = r.ResponseTimeOfTaskSet();

    std::cout << rta << std::endl;
    rt_num_opt::VectorDynamic rtaExpect = rt_num_opt::GenerateVectorDynamic(13);
    rtaExpect << 2, 6, 6, 10, 5, 8, 9, 13, 12, 14, 3, 11, 14;
    EXPECT(gtsam::assert_equal(rtaExpect, rta, 1e-3));
    // AssertEigenEqualVector(rtaExpect, rta);
}

TEST(rta, Nasri_no_edge) {
    rt_num_opt::core_m_dag = 3;
    rt_num_opt::PeriodRoundQuantum = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test2.yaml";
    std::vector<rt_num_opt::DAG_Model> dags =
        rt_num_opt::ReadDAG_NasriFromYaml(path);
    rt_num_opt::DAG_Nasri19 dagNasri(dags);
    auto str = dagNasri.ConvertTasksetToCsv();
    rt_num_opt::RTA_Nasri19 r(dagNasri);
    rt_num_opt::VectorDynamic rta = r.ResponseTimeOfTaskSet();

    std::cout << rta << std::endl;
    rt_num_opt::VectorDynamic rtaExpect = rt_num_opt::GenerateVectorDynamic(5);
    rtaExpect << 10, 11, 12, 23, 25;
    // EXPECT(gtsam::assert_equal<rt_num_opt::VectorDynamic>(rtaExpect, rta));
    AssertEigenEqualVector(rtaExpect, rta);
}

TEST(rta, Sync) {
    rt_num_opt::PeriodRoundQuantum = 1;
    rt_num_opt::core_m_dag = 3;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    std::vector<rt_num_opt::DAG_Model> dags =
        rt_num_opt::ReadDAG_NasriFromYaml(path);
    rt_num_opt::DAG_Nasri19 dagNasri(dags);
    rt_num_opt::RTA_Nasri19 r(dagNasri);
    rt_num_opt::VectorDynamic rta = r.ResponseTimeOfTaskSet();
    std::cout << std::endl << rta << std::endl << std::endl;
    rt_num_opt::VectorDynamic rta_expect = rta;
    rta_expect << 2, 6, 6, 10, 5, 8, 9, 13, 12, 14, 3, 11, 14;
    AssertEigenEqualVector(rta_expect, rta);
    dagNasri.tasks_[0].executionTime += 2;
    rt_num_opt::RTA_Nasri19 r2(dagNasri);
    rta = r2.ResponseTimeOfTaskSet();
    rta_expect << 4, 8, 8, 12, 5, 9, 12, 14, 15, 16, 3, 4, 7;
    AssertEigenEqualVector(rta_expect, rta);
    std::cout << rta << std::endl;
}

TEST(read_dag, rounding) {
    rt_num_opt::PeriodRoundQuantum = 1;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v5.yaml";
    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    EXPECT_LONGS_EQUAL(50000, tasks_dag.tasks_[0].period);
    EXPECT_LONGS_EQUAL(50000, tasks_dag.tasks_[0].deadline);
}
TEST(DAG, Adjust_period) {
    rt_num_opt::PeriodRoundQuantum = 1000;
    std::string path =
        "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v18.yaml";
    rt_num_opt::DAG_Nasri19 tasks_dag = rt_num_opt::ReadDAGNasri19_Tasks(path);
    tasks_dag.UpdatePeriod(0, 2000);
    tasks_dag.UpdatePeriod(1, 2000);
    tasks_dag.AdjustPeriod(1, -759);
    EXPECT_LONGS_EQUAL(2000, tasks_dag.tasks_[1].period);
    EXPECT_LONGS_EQUAL(2000, tasks_dag.tasks_[2].period);
    tasks_dag.AdjustPeriod(1, -1100);
    EXPECT_LONGS_EQUAL(1000, tasks_dag.tasks_[2].period);
    tasks_dag.AdjustPeriod(1, -2400);
    EXPECT_LONGS_EQUAL(1, tasks_dag.tasks_[2].period);
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
