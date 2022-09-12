#pragma once
#include <unordered_map>
#include <iomanip>
#include <iostream>

#include <boost/config.hpp>
#include <boost/utility.hpp> // for boost::tie
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/graph/topological_sort.hpp>

#include "sources/TaskModel/Tasks.h"

namespace rt_num_opt
{
    typedef std::map<int, TaskSet> MAP_Prev;
    // using namespace boost;
    typedef boost::property<boost::edge_weight_t, float> edge_weight_property;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
                                  boost::property<boost::vertex_name_t, int>,
                                  edge_weight_property>
        Graph;
    // map to access properties of vertex from the graph
    typedef boost::property_map<Graph, boost::vertex_name_t>::type Vertex_name_map_t;
    typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
    typedef boost::property_map<Graph, boost::edge_weight_t>::type edge_name_map_t;
    typedef Graph::edge_descriptor Edge;
    // map from task index to Vertex
    typedef std::unordered_map<int, Vertex> IndexVertexMap;
    typedef boost::graph_traits<rt_num_opt::Graph>::edge_iterator edge_iter;

    void AddVertexMy(Graph &g, IndexVertexMap &indexesBGL, int taskIndex)
    {
        // map to access vertex from its global index
        Vertex_name_map_t vertex2index = get(boost::vertex_name, g);
        IndexVertexMap::iterator pos;
        bool inserted;
        Vertex u;
        boost::tie(pos, inserted) = indexesBGL.insert(std::make_pair(taskIndex, Vertex()));
        if (inserted)
        {
            u = add_vertex(g);
            vertex2index[u] = taskIndex;
            // make sure the inserted vertex in indexesBGL
            //  is the same as the one inserted in the graph
            pos->second = u;
        }
        else
        {
            CoutError("Error building indexVertexMap!");
        }
    }

    std::pair<Graph, IndexVertexMap> EstablishGraphOnlyNodes(TaskSet &tasks)
    {

        Graph g;

        IndexVertexMap indexesBGL;
        for (uint i = 0; i < tasks.size(); i++)
        {
            AddVertexMy(g, indexesBGL, i);
        }
        return std::make_pair(g, indexesBGL);
    }

    class DAG_Model
    {
    public:
        TaskSet tasks_;
        Graph graph_;
        IndexVertexMap index2Vertex_;
        int N;
        double longestPath;
        double volume;

        DAG_Model()
        {
            longestPath = 0;
            volume = 0;
        }
        DAG_Model(TaskSet &tasks)
        {
            tasks_ = tasks;
            N = tasks.size();
            auto ss = EstablishGraphOnlyNodes(tasks);
            graph_ = ss.first;
            index2Vertex_ = ss.second;
            longestPath = 0;
            volume = 0;
        }
        // constructors for DAG specifically
        DAG_Model(TaskSet &tasks, Graph &graph,
                  IndexVertexMap &index2Vertex,
                  Vertex_name_map_t &vertex2index) : tasks_(tasks),
                                                     graph_(graph),
                                                     index2Vertex_(index2Vertex)
        {
            N = tasks.size();
            longestPath = 0;
            volume = 0;
        }
        inline TaskSet GetNormalTaskSet() { return tasks_; }
        inline Graph GetGraph() { return graph_; }
        inline IndexVertexMap GetIndexVertexMap() { return index2Vertex_; }
        inline Vertex_name_map_t GetVertex_name_map_t()
        {
            auto vertex2index_ = boost::get(boost::vertex_name, graph_);
            return vertex2index_;
        }
        static std::string Type() { return "dag"; }

        /**
         * @brief weight of edge is the same as child task's execution time
         *
         * @param prevIndex
         * @param nextIndex
         */
        void AddEdge(int prevIndex, int nextIndex)
        {
            // mapPrev[nextIndex].push_back(tasks_[prevIndex]);
            auto sth = boost::add_edge(prevIndex, nextIndex, graph_);
            Edge edge = sth.first;
            boost::property_map<Graph,
                                boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight, graph_);

            if (nextIndex < N)
            {
                EdgeWeightMap[edge] = tasks_[nextIndex].executionTime * -1;
            }
            else
            {
                EdgeWeightMap[edge] = 0;
            }
        }
        TaskSet GetTasks() const
        {
            return tasks_;
        }

        double Volume()
        {
            if (volume)
                return volume;
            for (uint i = 0; i < tasks_.size(); i++)
            {
                volume += tasks_[i].executionTime;
            }
            return volume;
        }
        struct EdgeProperties
        {
            int weight;
        };
        double CriticalPath()
        {
            if (longestPath)
                return longestPath;
            std::vector<Vertex> sources, sinks;

            boost::graph_traits<Graph>::in_edge_iterator ei, edge_end_i;
            boost::graph_traits<Graph>::out_edge_iterator eo, edge_end_o;
            boost::graph_traits<Graph>::vertex_iterator vi, viEnd;
            for (boost::tie(vi, viEnd) = vertices(graph_); vi != viEnd; ++vi)
            {
                boost::tie(ei, edge_end_i) = in_edges(*vi, graph_);
                if (ei == edge_end_i)
                    sources.push_back(*vi);
                boost::tie(eo, edge_end_o) = out_edges(*vi, graph_);
                if (eo == edge_end_o)
                    sinks.push_back(*vi);
            }
            AddVertexMy(graph_, index2Vertex_, N);
            AddVertexMy(graph_, index2Vertex_, N + 1);
            for (Vertex sourceVertex : sources)
            {
                AddEdge(N, sourceVertex);
            }
            for (Vertex sinkVertex : sinks)
            {
                AddEdge(sinkVertex, N + 1);
            }

            // Using bellman_ford_shortest_paths
            edge_name_map_t weight_pmap = get(boost::edge_weight, graph_);
            std::vector<int> distance(N + 2, (std::numeric_limits<short>::max)());
            distance[N] = 0;
            std::vector<std::size_t> parent(N + 2);
            for (int i = 0; i < N + 2; ++i)
                parent[i] = i;

            bellman_ford_shortest_paths(graph_, int(N + 2),
                                        weight_map(weight_pmap)
                                            .distance_map(&distance[0])
                                            .predecessor_map(&parent[0]));

            return distance.back() * -1;
        }

        /**
         * @brief add a dummy node to graph for the convenience of analysis by Nasri19;
         * this should only be applied during RTA analysis
         *
         */
        void AddDummyNode()
        {
            std::unordered_map<int, bool> umap;
            auto vertex2index_ = boost::get(boost::vertex_name, graph_);
            std::pair<rt_num_opt::edge_iter, rt_num_opt::edge_iter> ep;
            rt_num_opt::edge_iter ei, ei_end;
            for (tie(ei, ei_end) = boost::edges(graph_); ei != ei_end; ++ei)
            {
                rt_num_opt::Vertex vt = boost::target(*ei, graph_);
                int to_id = vertex2index_[vt];
                umap[to_id] = true;
            }
            if (static_cast<int>(umap.size()) >= N - 1)
            {
                return;
            }
            // for (auto it : umap)
            // {
            //     std::cout << it.first << ", " << it.second << std::endl;
            // }
            // std::cout << std::endl;
            tasks_.push_back(Task(0, 1000, 0, 1, 1000, N, 0));
            AddVertexMy(graph_, index2Vertex_, N);
            for (int i = 0; i < N; i++)
            {
                // if (!umap[i])
                {
                    AddEdge(N, i);
                }
            }
            N++;
        }
    };

    DAG_Model ReadDAG_Task(std::string path, std::string priorityType = "orig")
    {
        using namespace std;
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
                dagModel.AddEdge(dataInLine[0], dataInLine[1]);
            }

            if (debugMode == 1)
                std::cout << "Finish reading the data file" << path << " succesfully!\n";
            return dagModel;
        }
        else
        {
            std::cout << Color::red << "The path does not exist in ReadTaskSet!" << std::endl
                      << path
                      << Color::def << endl;
            throw;
        }
    }

    std::vector<int> GetDependentTasks(DAG_Model &dagTasks, int index)
    {
        std::vector<int> dependentIndexes;
        Vertex v = dagTasks.index2Vertex_[index];
        boost::graph_traits<Graph>::in_edge_iterator ei, edge_end_i;
        auto vertex2index_ = boost::get(boost::vertex_name, dagTasks.graph_);
        for (boost::tie(ei, edge_end_i) = in_edges(v, dagTasks.graph_); ei != edge_end_i; ++ei)
        {
            Vertex vvv = source(*ei, dagTasks.graph_);
            dependentIndexes.push_back(vertex2index_[vvv]);
        }
        return dependentIndexes;
    }

} // namespace rt_num_opt