#pragma once
#include <unordered_map>
#include "Tasks.h"

typedef std::map<int, TaskSet> MAP_Prev;
class DAG_Model : public TaskSetNormal
{
public:
    MAP_Prev mapPrev;
    int N;

    DAG_Model()
    {
        ;
    }
    // constructors for DAG specifically
    DAG_Model(TaskSet &tasks, MAP_Prev &mapPrev) : TaskSetNormal(tasks),
                                                   mapPrev(mapPrev)
    {
        N = tasks.size();
    }
    void UpdateTaskSet(TaskSet &tasks, MAP_Prev &mapPrevIn)
    {
        tasks_ = tasks;
        mapPrev = mapPrevIn;
        N = tasks.size();
    }
    static string Type() { return "dag"; }

    void addEdge(int prevIndex, int nextIndex)
    {
        mapPrev[nextIndex].push_back(tasks_[prevIndex]);
    }
    TaskSet GetTasks() const
    {
        return tasks_;
    }
    int edgeNumber()
    {
        int count = 0;
        for (auto itr = mapPrev.begin(); itr != mapPrev.end(); itr++)
        {
            count += (itr->second).size();
        }
        return count;
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

    MAP_Prev mapPrev;

    fstream file;
    file.open(path, ios::in);
    if (file.is_open())
    {
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
            mapPrev[dataInLine[1]].push_back(tasks[dataInLine[0]]);
        }

        DAG_Model ttt(tasks, mapPrev);

        if (debugMode == 1)
            cout << "Finish reading the data file" << path << " succesfully!\n";
        return ttt;
    }
    else
    {
        cout << Color::red << "The path does not exist in ReadTaskSet!" << endl
             << path
             << Color::def << endl;
        throw;
    }
}