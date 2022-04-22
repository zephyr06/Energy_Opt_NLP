#pragma once

#include "sources/TaskModel/Tasks.h"
namespace rt_num_opt
{
    std::vector<double> String2IntVector(std::vector<std::string> vecS)
    {
        std::vector<double> vecI;
        vecI.reserve(vecS.size());

        for (int i = 1; i < stoi(vecS[0]) + 1; i++)
        {
            vecI.push_back(stod(vecS[i]));
        }
        return vecI;
    }
    std::pair<TaskSet, VectorDynamic> ReadControlCase(std::string path)
    {
        using namespace std;
        fstream newfile;
        VectorDynamic coeffVec;
        vector<double>
            executionTimeVector;
        newfile.open(path, ios::in); //open a file to perform read operation using file object
        if (newfile.is_open())
        { //checking whether the file is open
            std::string tp;
            // get number of lines in the file
            getline(newfile, tp);

            // get computation time vector
            getline(newfile, tp);
            executionTimeVector = String2IntVector(SplitStringMy(tp, " "));

            // get coeffVec
            getline(newfile, tp);
            coeffVec = Vector2Eigen<double>(String2IntVector(SplitStringMy(tp, " ")));

            // max period doesn't have to be read

            newfile.close(); //close the file object.
        }
        else
        {
            CoutError("Path doesn't exist in ReadControlCase: " + path);
        }
        double period = std::accumulate(executionTimeVector.begin(), executionTimeVector.end(), 0) * 5;
        TaskSet tasks;
        for (uint i = 0; i < executionTimeVector.size(); i++)
        {
            Task t(0, period, 0, executionTimeVector[i], period, i, 0);
            tasks.push_back(t);
        }
        return std::make_pair(tasks, coeffVec);
    }
} // namespace rt_num_opt