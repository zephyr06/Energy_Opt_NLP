#pragma once
// The analysis is from the paper "Schedulability analysis of DAG tasks with
// arbitrary deadlines under global fixed-priority scheduling"
#include "sources/TaskModel/DAG_Task.h"

namespace rt_num_opt {
dagSched::DAGTask TransformSingleTaskNumOpt2dagSched(
    rt_num_opt::DAG_Model dagTasks) {
    dagTasks.AddDummyNode();

    long long hyperPeriod = rt_num_opt::HyperPeriod(dagTasks.tasks_);

    std::vector<dagSched::SubTask *> given_V;
    dagSched::DAGTask taskDAGCurr;
    // read and set data
    taskDAGCurr.setPeriod(static_cast<float>(hyperPeriod));
    taskDAGCurr.setDeadline(static_cast<float>(hyperPeriod));
    rt_num_opt::TaskSet tasksMy = dagTasks.GetNormalTaskSet();

    // add tasks
    for (size_t j = 0; j < tasksMy.size(); j++) {
        dagSched::SubTask *v = new dagSched::SubTask;
        // v->id = tasksMy[i].id;
        v->id = j;
        v->c = tasksMy[j].executionTime;
        given_V.push_back(v);
    }

    // add edges

    auto vertex2index_ = boost::get(boost::vertex_name, dagTasks.graph_);
    std::pair<rt_num_opt::edge_iter, rt_num_opt::edge_iter> ep;
    rt_num_opt::edge_iter ei, ei_end;
    for (tie(ei, ei_end) = boost::edges(dagTasks.graph_); ei != ei_end; ++ei) {
        rt_num_opt::Vertex vv = boost::source(*ei, dagTasks.graph_);
        int from_id = vertex2index_[vv];
        rt_num_opt::Vertex vt = boost::target(*ei, dagTasks.graph_);
        int to_id = vertex2index_[vt];
        // std::cout << from_id << ", " << to_id << std::endl;

        given_V[from_id]->succ.push_back(given_V[to_id]);
        given_V[to_id]->pred.push_back(given_V[from_id]);
    }

    taskDAGCurr.setVertices(given_V);

    // processing by Verucchi
    taskDAGCurr.transitiveReduction();
    taskDAGCurr.computeWorstCaseWorkload();
    taskDAGCurr.computeVolume();
    taskDAGCurr.computeLength();
    taskDAGCurr.computeUtilization();
    taskDAGCurr.computeDensity();

    return taskDAGCurr;
}

std::vector<rt_num_opt::DAG_Model> TransformTaskSetNumOpt2dagSched(
    std::vector<std::string> paths) {
    std::vector<rt_num_opt::DAG_Model> taskset;
    size_t taskSetSize = paths.size();
    for (size_t i = 0; i < taskSetSize; i++) {
        auto dagTasks = rt_num_opt::ReadDAG_Task(paths[i], "RM");
        // dagTasks.AddDummyNode();
        taskset.push_back(dagTasks);
    }
    return taskset;
}

class DAG_Fonseca : public TaskSetNormal {
    Graph graph_;
    IndexVertexMap index2Vertex_;
    Vertex_name_map_t vertex2index_;

   public:
    DAG_Fonseca() {}
    DAG_Fonseca(DAG_Model dagTasks) {
        TaskSet taskSet = dagTasks.GetNormalTaskSet();
        UpdateTaskSet(taskSet);
        graph_ = dagTasks.GetGraph();
        index2Vertex_ = dagTasks.GetIndexVertexMap();
        vertex2index_ = dagTasks.GetVertex_name_map_t();
    }

    inline DAG_Model GetDAG_Model() {
        return DAG_Model(tasks_, graph_, index2Vertex_, vertex2index_);
    }

    inline static std::string Type() { return "Fonseca"; }
};
DAG_Fonseca ReadDAGFonseca_Tasks(std::string path,
                                 std::string priorityType = "orig") {
    DAG_Model dagTasks = ReadDAG_Task(path, priorityType);
    return DAG_Fonseca(dagTasks);
}
}  // namespace rt_num_opt