
#include <yaml-cpp/yaml.h>
#include <CppUnitLite/TestHarness.h>

#include "sources/TaskModel/DAG_Task.h"
#include "sources/TaskModel/ReadWriteYaml.h"
#include "sources/TaskModel/DAG_Narsi19.h"
#include "sources/RTA/RTA_Narsi19.h"

TEST(IO, v1)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    YAML::Node config = YAML::LoadFile(path);
    YAML::Node tasksNode = config["tasks"];
    EXPECT_LONGS_EQUAL(20, tasksNode[0]["period"].as<int>());
    EXPECT_LONGS_EQUAL(20, tasksNode[0]["deadline"].as<long int>());

    YAML::Node vert = tasksNode[0]["vertices"];
    EXPECT_LONGS_EQUAL(4, vert[1]["executionTime"].as<int>());

    std::ofstream fout("fileUpdate.yaml");
    fout << config;
    fout.close();
}
TEST(io, v2)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    auto dags = rt_num_opt::ReadDAG_NarsiFromYaml(path);
    EXPECT_LONGS_EQUAL(4, dags[0].tasks_[3].executionTimeOrg);
    EXPECT_LONGS_EQUAL(3, dags[0].tasks_[3].id);
    EXPECT_LONGS_EQUAL(30, dags[1].tasks_[0].period);
    EXPECT_LONGS_EQUAL(30, dags[1].tasks_[1].deadline);
    EXPECT_LONGS_EQUAL(40, dags[2].tasks_[2].deadline);
    EXPECT_LONGS_EQUAL(3, dags[2].tasks_[2].executionTime);
}
TEST(io, v3)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    auto dags = rt_num_opt::ReadDAG_NarsiFromYaml(path);
    rt_num_opt::WriteDAG_NarsiToYaml(dags, "testOutput1.yaml");
}

TEST(io, no_edge)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n5_v23.csv";
    auto dag = rt_num_opt::ReadDAG_Task(path);
    std::vector<rt_num_opt::DAG_Model> dags;
    dags.push_back(dag);
    std::string outputPath = "testOutput2.yaml";
    rt_num_opt::WriteDAG_NarsiToYaml(dags, outputPath);
    auto dagsRead = rt_num_opt::ReadDAG_NarsiFromYaml(outputPath);
    EXPECT_LONGS_EQUAL(200, dagsRead[0].tasks_[0].period);
    EXPECT_LONGS_EQUAL(0, boost::num_edges(dag.graph_));
    EXPECT_LONGS_EQUAL(0, boost::num_edges(dagsRead[0].graph_));
}
TEST(io, narsi)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    std::vector<rt_num_opt::DAG_Model> dags = rt_num_opt::ReadDAG_NarsiFromYaml(path);
    rt_num_opt::DAG_Narsi19 dagNarsi(dags);
    dagNarsi.tasks_[0].executionTime = 11;
    auto str = dagNarsi.ConvertTasksetToCsv();
}

TEST(rta, narsi)
{
    std::string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/taskset.yaml";
    std::vector<rt_num_opt::DAG_Model> dags = rt_num_opt::ReadDAG_NarsiFromYaml(path);
    rt_num_opt::DAG_Narsi19 dagNarsi(dags);
    auto str = dagNarsi.ConvertTasksetToCsv();
    // rt_num_opt::RTA_Narsi19 r(dagNarsi);
    // rt_num_opt::VectorDynamic rta = r.ResponseTimeOfTaskSet();

    // std::cout << rta << std::endl;

    // rt_num_opt::VectorDynamic rtaExpect = rt_num_opt::GenerateVectorDynamic(57);
    // rtaExpect;
}
int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
