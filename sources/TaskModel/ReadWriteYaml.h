#pragma once

#include <yaml-cpp/yaml.h>
#include "sources/TaskModel/DAG_Task.h"

namespace rt_num_opt
{
    std::vector<rt_num_opt::DAG_Model> ReadDAG_NasriFromYaml(std::string path)
    {
        YAML::Node config = YAML::LoadFile(path);
        YAML::Node tasksNode;
        if (config["tasks"])
        {
            tasksNode = config["tasks"];
        }
        else
        {
            CoutError("Input file doesn't follow Nasri format: " + path);
        }

        // read frequency ratio, if any
        if (config["frequencyRatio"])
        {
            frequencyRatio = config["frequencyRatio"].as<double>();
        }
        else
        {
            frequencyRatio = 0.5;
        }

        std::vector<rt_num_opt::DAG_Model> dags;
        dags.reserve(tasksNode.size());

        for (size_t i = 0; i < tasksNode.size(); i++)
        {
            YAML::Node vertices = tasksNode[i]["vertices"];
            rt_num_opt::TaskSet tasks;
            tasks.reserve(vertices.size());
            for (size_t j = 0; j < vertices.size(); j++)
            {
                rt_num_opt::Task taskCurr;
                taskCurr.deadline = tasksNode[i]["deadline"].as<int>();
                taskCurr.executionTime = vertices[j]["executionTime"].as<int>();
                taskCurr.executionTimeOrg = taskCurr.executionTime;
                taskCurr.id = vertices[j]["id"].as<int>();
                if (vertices[j]["offset"])
                {
                    taskCurr.offset = vertices[j]["offset"].as<int>();
                }
                if (vertices[j]["overhead"])
                {
                    taskCurr.overhead = vertices[j]["overhead"].as<int>();
                }
                taskCurr.period = tasksNode[i]["period"].as<int>();
                taskCurr.periodOrg = taskCurr.period;
                if (vertices[j]["processorId"])
                {
                    taskCurr.processorId = vertices[j]["processorId"].as<int>();
                }
                tasks.push_back(taskCurr);
            }

            rt_num_opt::DAG_Model dag(tasks);

            YAML::Node edgesNode = tasksNode[i]["edges"];
            for (size_t j = 0; j < edgesNode.size(); j++)
            {
                // std::cout << "Add edge: " << edgesNode[j]["from"].as<int>() << ", " << edgesNode[j]["to"].as<int>() << std::endl;
                dag.AddEdge(edgesNode[j]["from"].as<int>(), edgesNode[j]["to"].as<int>());
            }

            dags.push_back(dag);
        }
        sort(dags.begin(), dags.end(), [](const rt_num_opt::DAG_Model &a, const rt_num_opt::DAG_Model &b) -> bool
             { return a.tasks_[0].deadline < b.tasks_[0].deadline; });
        return dags;
    }

    void WriteDAG_NasriToYaml(std::vector<rt_num_opt::DAG_Model> &dags, std::string path)
    {

        YAML::Node nodeTasks;
        for (size_t i = 0; i < dags.size(); i++)
        {

            rt_num_opt::DAG_Model dag = dags[i];

            YAML::Node verticesNode;
            for (size_t j = 0; j < dag.tasks_.size(); j++)
            {
                YAML::Node taskNode;
                taskNode["id"] = dag.tasks_[j].id;
                taskNode["executionTime"] = std::to_string(int(dag.tasks_[j].executionTime));
                verticesNode.push_back(taskNode);
            }

            YAML::Node edgesNode;
            rt_num_opt::edge_iter ei, ei_end;
            auto vertex2index_ = boost::get(boost::vertex_name, dag.graph_);
            for (tie(ei, ei_end) = boost::edges(dag.graph_); ei != ei_end; ++ei)
            {
                int fromId = vertex2index_[boost::source(*ei, dag.graph_)];
                int toId = vertex2index_[boost::target(*ei, dag.graph_)];
                YAML::Node edgeNode;
                edgeNode["from"] = fromId;
                edgeNode["to"] = toId;
                // std::cout << fromId << ", " << toId << std::endl;
                edgesNode.push_back(edgeNode);
            }

            YAML::Node nodeDAG;
            nodeDAG["deadline"] = std::to_string(dag.tasks_[0].deadline);
            nodeDAG["period"] = std::to_string(rt_num_opt::HyperPeriod(dag.tasks_));
            nodeDAG["vertices"] = verticesNode;
            nodeDAG["edges"] = edgesNode;
            nodeTasks.push_back(nodeDAG);
        }
        YAML::Node nodeRoot;
        nodeRoot["tasks"] = nodeTasks;

        std::ofstream fout(path);
        fout << nodeRoot;
        fout.close();
    }
} // namespace rt_num_opt