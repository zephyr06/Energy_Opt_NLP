
#pragma once
#include <gtsam/nonlinear/Values.h>

#include "sources/MatrixConvenient.h"
#include "sources/Utils/Parameters.h"
#include "sources/TaskModel/Tasks.h"
#include "sources/Tools/testMy.h"
namespace rt_num_opt
{
    // these two variables record the best solution ever known yet
    double valueGlobalOpt = INT64_MAX;
    VectorDynamic vectorGlobalOpt;

    void InitializeGlobalVector(int N)
    {
        vectorGlobalOpt.resize(N, 1);
        vectorGlobalOpt.setZero();
        valueGlobalOpt = INT64_MAX;
        TASK_NUMBER = N;
    }

    enum class EliminationType
    {
        Not,
        RTA,
        Bound
    };
    struct RecordInfo
    {
        uint taskIndex;

        EliminationType type;

        RecordInfo(uint index, EliminationType type) : taskIndex(index), type(type) {}

        void print()
        {
            std::cout << "TaskIndex: " << taskIndex << " ";
            if (type == EliminationType::Not)
            {
                std::cout << "Not eliminated!" << std::endl;
            }
            else if (type == EliminationType::RTA)
            {
                std::cout << "eliminated by RTA" << std::endl;
            }
            else if (type == EliminationType::Bound)
            {
                std::cout << "eliminated by bound constraint" << std::endl;
            }
            else
            {
                CoutError("Unrecognized EliminationType");
            }
        }
    };
    class EliminationRecord
    {
    public:
        std::vector<RecordInfo> record;
        EliminationRecord()
        {
        }

        RecordInfo operator[](int index)
        {
            return record[index];
        }

        bool whetherAllEliminated()
        {
            for (size_t i = 0; i < record.size(); i++)
            {
                if (record[i].type == EliminationType::Not)
                    return false;
            }
            return true;
        }
        /**
         * @brief
         *
         * @param n  task set's size
         */
        void Initialize(uint n)
        {
            record.clear();
            if (record.size() >= n)
            {
                return;
            }
            record.reserve(n);
            for (uint i = 0; i < n; i++)
            {
                record.push_back({i, EliminationType::Not});
            }
        }
        std::vector<RecordInfo> SaveRecord()
        {
            return record;
        }
        void RangeCheck(uint i)
        {
            if (i < 0 || i >= record.size())
            {
                CoutError("Out of Scope in EliminationRecord");
            }
        }
        void SetEliminated(uint i, EliminationType type)
        {
            record[i] = {i, type};
        }
        void SetUnEliminated(uint i)
        {
            record[i] = {i, EliminationType::Not};
        }

        void AdjustEliminationError(double err, uint index, EliminationType type)
        {
            RangeCheck(index);

            if (err > 0)
            {
                if (type <= record[index].type)
                    return;
                SetEliminated(index, type);
            }
            else if (err == 0)
            {
                // this statement prevents variables going back from elimination;
                // reason: we only adjust elimination after LM stops; during LM's inner loops, even though elimination stats could possibly change, the graph doesn't change because it is build before inner loops, and so these changes won't influence inner loop; as for the outer loop, it will make a difference;
                // changes made during inner loop must be discarded by recovering elimination stats based on previous' result
                if (record[index].type != EliminationType::Not)
                    return;
                SetUnEliminated(index);
            }
            else
            {
                CoutError("Unrecognized input type, negative err in AdjustEliminationError!");
            }
        }
        void Print()
        {
            std::cout << "The elimination record is " << std::endl;
            for (auto x : record)
            {
                // std::cout << x << ", ";
                x.print();
            }
            std::cout << std::endl;
        }
        void PrintViolatedFactor()
        {

            for (uint i = 0; i < record.size(); i++)
            {
                if (record[i].type != EliminationType::Not)
                {
                    record[i].print();
                }
            }
            std::cout << std::endl;
        }
    };
} // namespace rt_num_opt