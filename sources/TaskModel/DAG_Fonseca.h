#pragma once

#include "sources/TaskModel/DAG_Task.h"

namespace rt_num_opt
{
    class DAG_Fonseca : public TaskSetNormal
    {
        Graph graph_;
        IndexVertexMap index2Vertex_;
        Vertex_name_map_t vertex2index_;

    public:
        DAG_Fonseca(DAG_Model dagTasks)
        {
            TaskSetNormal(dagTasks.GetNormalTaskSet());
            graph_ = dagTasks.GetGraph();
            index2Vertex_ = dagTasks.GetIndexVertexMap();
            vertex2index_ = dagTasks.GetVertex_name_map_t();
        }

        inline DAG_Model GetDAG_Model()
        {
            return DAG_Model(tasks_, graph_, index2Vertex_, vertex2index_);
        }
    };

}