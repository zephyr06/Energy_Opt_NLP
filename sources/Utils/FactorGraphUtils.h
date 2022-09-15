#pragma once
#include <gtsam/inference/Symbol.h>

#include "sources/Utils/GlobalVariables.h"
namespace rt_num_opt
{
    bool ContainFalse(EliminationRecord &eliminationRecord)
    {
        for (uint i = 0; i < eliminationRecord.record.size(); i++)
        {
            if (eliminationRecord[i].type == EliminationType::Not)
            {
                return true;
            }
        }
        return false;
    }
    /**
     * @brief generate keys for factor graph modeling the control problem
     * keys' attributes can be extracted via key.chr(). key.index()
     *
     * @param idtask
     * @param type
     * @return gtsam::Symbol
     */
    gtsam::Symbol GenerateKey(int idtask, std::string type)
    {
        if (type == "period")
        {
            return gtsam::Symbol('t', idtask);
        }
        else if (type == "response")
        {
            return gtsam::Symbol('r', idtask);
        }
        else if (type == "executionTime")
        {
            return gtsam::Symbol('c', idtask);
        }
        else
        {
            CoutError("Unrecognized type in GenerateControlKey");
            gtsam::Symbol key('a', idtask);
            return key;
        }
    }

    int AnalyzeKey(gtsam::Symbol key)
    {
        return key.index();
    }

    /**
     * @brief perform customized quotient operation for two double type numbers
     * return result is min_k |a - k b|, where k is an integer, a & b >0;
     *
     * Brute force solution, find minimum remainder by finding minimum rem=(a-=b)
     * @param a
     * @param b
     * @return double
     */
    double QuotientDouble(double a, double b)
    {
        double remainder = a - int(a / b) * b;
        return min(remainder, abs(remainder - b));
    }

    template <typename T>
    bool Equals(std::vector<T> &v1, std::vector<T> &v2)
    {
        if (v1.size() != v2.size())
        {
            return false;
        }
        for (uint i = 0; i < v1.size(); i++)
        {
            if (v1[i] != v2[i])
            {
                return false;
            }
        }
        return true;
    }

    /* only used in Reorder function */
    struct TaskAugment
    {
        Task task_;
        double coeff_T;
        double coeff_R;
        TaskAugment(Task &task, double coeffT, double coeffR) : task_(task), coeff_T(coeffT), coeff_R(coeffR) {}
        static bool Compare(const TaskAugment &t1, const TaskAugment &t2)
        {
            if (enableReorder <= 1)
            {
                return t1.task_.period < t2.task_.period;
            }
            else
            {
                if (t1.task_.period != t2.task_.period)
                    return t1.task_.period < t2.task_.period;
                else
                {
                    return t1.task_.executionTime < t2.task_.executionTime;
                }
            }
        }
    };
    /* in-place adjustment of tasks' order; highest order being the first in tasks */
    void Reorder(TaskSet &tasks, VectorDynamic &coeff)
    {
        std::vector<TaskAugment> tasksAug;
        tasksAug.reserve(tasks.size());
        for (uint i = 0; i < tasks.size(); i++)
        {
            tasksAug.push_back({tasks[i], coeff(i * 2), coeff(i * 2 + 1)});
        }
        stable_sort(tasksAug.begin(), tasksAug.end(), TaskAugment::Compare);
        for (uint i = 0; i < tasks.size(); i++)
        {
            tasks[i] = tasksAug[i].task_;
            coeff(2 * i) = tasksAug[i].coeff_T;
            coeff(2 * i + 1) = tasksAug[i].coeff_R;
        }
    }

    /* @brief return a std::string with expected precision by adding leading 0 */
    std::string to_string_precision(int a, int precision)
    {
        return std::string(4 - min(4, std::to_string(a).length()), '0') + std::to_string(a);
    }

    template <typename T>
    void print(const std::vector<T> &vec)
    {
        for (auto x : vec)
        {
            std::cout << x << ", ";
        }
    }

} // namespace rt_num_opt