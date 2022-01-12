#pragma once
#include <unordered_map>
#include <boost/config.hpp>
#include <boost/utility.hpp> // for boost::tie
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/topological_sort.hpp>

#include "Tasks.h"

typedef std::map<int, TaskSet> MAP_Prev;
using namespace boost;
typedef adjacency_list<vecS, vecS, bidirectionalS,
                       property<vertex_name_t, LLint>,
                       property<edge_name_t, LLint>>
    Graph;
// map to access properties of vertex from the graph
typedef property_map<Graph, vertex_name_t>::type Vertex_name_map_t;
typedef graph_traits<Graph>::vertex_descriptor Vertex;
typedef property_map<Graph, edge_name_t>::type edge_name_map_t;
// map from task index to Vertex
typedef std::unordered_map<int, Vertex> IndexVertexMap;

pair<Graph, IndexVertexMap> EstablishGraphOnlyNodes(TaskSet &tasks)
{

    Graph g;
    // map to access properties of vertex from the graph
    Vertex_name_map_t vertex2index = get(vertex_name, g);

    // map to access vertex from its global index

    IndexVertexMap indexesBGL;
    for (uint i = 0; i < tasks.size(); i++)
    {
        IndexVertexMap::iterator pos;
        bool inserted;
        Vertex u;
        boost::tie(pos, inserted) = indexesBGL.insert(std::make_pair(i, Vertex()));
        if (inserted)
        {
            u = add_vertex(g);
            vertex2index[u] = i;
            // make sure the inserted vertex in indexesBGL
            //  is the same as the one inserted in the graph
            pos->second = u;
        }
        else
        {
            CoutError("Error building indexVertexMap!");
        }
    }
    return std::make_pair(g, indexesBGL);
}

class DAG_Model : public TaskSetNormal
{
public:
    Graph graph_;
    IndexVertexMap index2Vertex_;
    Vertex_name_map_t vertex2index_;
    int N;

    DAG_Model()
    {
        ;
    }
    DAG_Model(TaskSet &tasks) : TaskSetNormal(tasks)
    {
        N = tasks.size();
        auto ss = EstablishGraphOnlyNodes(tasks);
        graph_ = ss.first;
        index2Vertex_ = ss.second;
        vertex2index_ = get(vertex_name, graph_);
    }
    // constructors for DAG specifically
    DAG_Model(TaskSet &tasks, Graph &graph,
              IndexVertexMap &index2Vertex,
              Vertex_name_map_t &vertex2index) : TaskSetNormal(tasks),
                                                 graph_(graph),
                                                 index2Vertex_(index2Vertex),
                                                 vertex2index_(vertex2index)
    {
        N = tasks.size();
    }

    static string Type() { return "dag"; }

    void addEdge(int prevIndex, int nextIndex)
    {
        // mapPrev[nextIndex].push_back(tasks_[prevIndex]);
        boost::add_edge(prevIndex, nextIndex, graph_);
        vertex2index_ = get(vertex_name, graph_);
    }
    TaskSet GetTasks() const
    {
        return tasks_;
    }

    double volume()
    {
        double vol = 0;
        for (uint i = 0; i < tasks_.size(); i++)
        {
            vol += tasks_[i].executionTime;
        }
        return vol;
    }
    double criticalPath()
    {
        return 0;
    }
};

DAG_Model ReadDAG_Tasks(string path, string priorityType = "orig")
{
    TaskSet tasks = ReadTaskSet(path, priorityType);
    // some default parameters in this function
    string delimiter = ",";
    string token;
    string line;
    size_t pos = 0;

    fstream file;
    file.open(path, ios::in);
    if (file.is_open())
    {
        DAG_Model dagModel(tasks);
        string line;
        while (getline(file, line))
        {
            if (line[0] != '*')
                continue;
            line = line.substr(1, int(line.size()) - 1);
            vector<int> dataInLine;
            while ((pos = line.find(delimiter)) != string::npos)
            {
                token = line.substr(0, pos);
                int temp = atoi(token.c_str());
                dataInLine.push_back(temp);
                line.erase(0, pos + delimiter.length());
            }
            dataInLine.push_back(atoi(line.c_str()));
            // mapPrev[dataInLine[1]].push_back(tasks[dataInLine[0]]);
            dagModel.addEdge(dataInLine[1], dataInLine[0]);
        }

        if (debugMode == 1)
            cout << "Finish reading the data file" << path << " succesfully!\n";
        return dagModel;
    }
    else
    {
        cout << Color::red << "The path does not exist in ReadTaskSet!" << endl
             << path
             << Color::def << endl;
        throw;
    }
}

vector<int> GetDependentTasks(DAG_Model &dagTasks, int index)
{
    vector<int> dependentIndexes;
    Vertex v = dagTasks.index2Vertex_[index];
    boost::graph_traits<Graph>::out_edge_iterator eo, edge_end_o;
    for (boost::tie(eo, edge_end_o) = out_edges(v, dagTasks.graph_); eo != edge_end_o; ++eo)
    {
        Vertex vvv = target(*eo, dagTasks.graph_);
        dependentIndexes.push_back(dagTasks.vertex2index_[vvv]);
    }
    return dependentIndexes;
}