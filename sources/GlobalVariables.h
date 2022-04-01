
#pragma once
#include "testMy.h"

enum class EliminationType
{
    Not,
    RTA,
    Bound
};
struct RecordInfo
{
    int taskIndex;

    EliminationType type;

    RecordInfo(int index, EliminationType type) : taskIndex(index), type(type) {}

    void print()
    {
        cout << "TaskIndex: " << taskIndex << " ";
        if (type == EliminationType::Not)
        {
            cout << "Not eliminated!" << endl;
        }
        else if (type == EliminationType::RTA)
        {
            cout << "eliminated by RTA" << endl;
        }
        else if (type == EliminationType::Bound)
        {
            cout << "eliminated by bound constraint" << endl;
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
    /**
     * @brief 
     * 
     * @param n  task set's size
     */
    void Initialize(int n)
    {
        record.clear();
        if (record.size() >= n)
        {
            return;
        }
        record.reserve(n);
        for (int i = 0; i < n; i++)
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
        RangeCheck(i);
        if (type > record[i].type)
        {
            record[i] = {i, type};
        }
    }
    void SetUnEliminated(uint i)
    {
        record[i] = {i, EliminationType::Not};
    }

    void AdjustEliminationError(double err, uint index, EliminationType type)
    {
        if (err > 0)
        {
            SetEliminated(index, type);
        }
        else if (err == 0)
        {
            SetUnEliminated(index);
        }
        else
        {
            CoutError("Unrecognized input type, negative err in AdjustEliminationError!");
        }
    }
    void Print()
    {
        cout << "The elimination record is " << endl;
        for (auto x : record)
        {
            // cout << x << ", ";
            x.print();
        }
        cout << endl;
    }
    void PrintViolatedFactor(NonlinearFactorGraph &graph)
    {

        for (uint i = 0; i < record.size(); i++)
        {
            if (record[i].type != EliminationType::Not)
            {
                record[i].print();
            }
        }
        cout << endl;
    }
};
EliminationRecord eliminationRecordGlobal;