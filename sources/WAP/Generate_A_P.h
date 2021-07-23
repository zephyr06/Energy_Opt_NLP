#pragma once
#include "RTA_WAP.h"

/**
 * @brief The logic to decide how to assign A[i][j], P[i][j], given current task i, and lp task j;
 * Assume A, P matrix are initialized as zero matrix
 * 
 * This function performs in-place updating
 * 
 * @param indexCurr 
 * @param indexLp 
 * @param tasks 
 * @param A 
 * @param P 
 * @return vector<int> flag 
 */
bool AssignAP_LogicBasic(int indexCurr, int indexLp, TaskSet &tasks,
                         SquareMatrix &A, SquareMatrix &P)
{

    if (ResponseTimeWAP(tasks, A, P, indexCurr) <= tasks[indexCurr].deadline)
    {
        return true;
    }
    else
    {
        P(indexCurr, indexLp) = 1;
        if (ResponseTimeWAP(tasks, A, P, indexCurr) <= tasks[indexCurr].deadline)
        {
            return true;
        }
        else
        {
            A(indexCurr, indexLp) = 1;
            P(indexCurr, indexLp) = 0;
            if (ResponseTimeWAP(tasks, A, P, indexCurr) <= tasks[indexCurr].deadline)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    cout << "Unexpected situation happen in AssignAP_LogicBasic" << endl;
    throw;
    return false;
}

class DataMetaWAP
{
public:
    double value;
    int index;

    // "block" means block cost, "preempt" means preemption cost
    string pattern;

    DataMetaWAP() : value(0), index(-1), pattern("block") {}
    DataMetaWAP(double v, int i, string p) : value(v), index(i), pattern(p) {}

    void print()
    {
        cout << "In one DataMetaWAP the value, index, pattern is " << value << ", "
             << index << ", " << pattern << endl;
    }
};

bool compareInWAP(DataMetaWAP &p1, DataMetaWAP &p2)
{
    return p1.value > p2.value;
}

/**
 * @brief 
 * 
 *     We analyze AB from hp to lp, for the current considered task, we only need to worry about lp part because hp part
    is already fixed and optimal; The influence of lp part is by blocking, which could be caused by 'block' type or
    'preempt' type, and only one type of one lp task could block the current hp task. This inspires us to create a sort
    array which contains all the possible block items, and then we find the longest entry that makes current task
    schedulable. By sustainability w.r.t. blocking time, all the following entry will make current task schedulable, and
    we try to assign A/B values to make the lp tasks most likely to be schedulable.

    # Central assumption: preemption overhead is always smaller than or equal to execution_time - 1;
    otherwise, the system may not work

    #The main logic has two steps
    #first step, find the first entry in list_for_sort that makes task_curr_id schedulable

    #second step, decide A[i,:] B[i:, :] based on the entry found

    if the found entry is type a
        all the entries following it including itself:
            if this entry is type A, A[task_curr_id, meta.id]=0, B[task_curr_id, meta.id]=0
            if this entry is type B and not settled, A[task_curr_id, meta.id]=0, B[task_curr_id, meta.id]=1
        for all the entries that are not settled, their type a and b must have a position earlier than the found entry:
            A[task_curr_id, meta.id] = 0, B[task_curr_id, meta.id] = 0
    if the found entry is type b
        A[task_curr_id, found_entry.id] = 0, B[task_curr_id, found_entry.id] = 0
        all the entries following it:
            if this entry is type A, A[task_curr_id, meta.id]=0, B[task_curr_id, meta.id]=0
            if this entry is type B and not settled, A[task_curr_id, meta.id]=0, B[task_curr_id, meta.id]=1
        all the entries not settled, their type a and b must have a position earlier than the found entry:
            A[task_curr_id, meta.id] = 0, B[task_curr_id, meta.id] = 0
 * 
 * This function performs in-place update!
 * @param indexCurr 
 * @param tasks 
 * @param A 
 * @param B 
 * @return true 
 * @return false 
 */
bool AssignAP_LogicForOneTask(int indexCurr, TaskSet &tasks, SquareMatrix &A, SquareMatrix &P)
{
    int N = tasks.size();

    // stores meta data of lp tasks
    vector<DataMetaWAP> vecForLP;

    for (int i = indexCurr + 1; i < N; i++)
    {
        vecForLP.push_back(DataMetaWAP(tasks[i].executionTime - 1, i, "block"));
        vecForLP.push_back(DataMetaWAP(tasks[i].overhead, i, "preempt"));
    }

    sort(vecForLP.begin(), vecForLP.end(), compareInWAP);

    bool success = false;
    int indexFirstAvailable = -1;

    // first step, find the first entry in list_for_sort that makes task_curr_id schedulable
    for (int i = 0; i < int(vecForLP.size()); i++)
    {
        if (ResponseTimeWapGivenBlock(tasks, A, P, indexCurr, vecForLP[i].value) <= tasks[indexCurr].deadline)
        {
            success = true;
            indexFirstAvailable = i;
            break;
        }
    }

    if (indexFirstAvailable == -1)
    {
        //The current task cannot tolerate any block from lp tasks
        //Let's try if all block items are 0, will it be schedulable
        if (ResponseTimeWapGivenBlock(tasks, A, P, indexCurr, 0) <= tasks[indexCurr].deadline)
        {
            success = true;
            // In this case, all block items must be 0
            for (int l = indexCurr + 1; l < N; l++)
            {
                A(indexCurr, l) = 0;
                P(indexCurr, l) = 0;
            }
            return true;
        }
        else
        {
            if (debugMode == 1)
                cout << "The given system is not schedulable by any method in this paper!" << endl;
            return false;
        }
    }

    // decide A[i,:] P[i:, :] based on the entry found

    vector<bool> decidedIndex(N, false);
    // first iteration, go through entries that can tolerate block or preempt
    for (int l = indexFirstAvailable; l < int(vecForLP.size()); l++)
    {
        int realTaskIndex = vecForLP[l].index;
        if (vecForLP[l].pattern == "block" && decidedIndex[realTaskIndex] == false)
        {
            A(indexCurr, realTaskIndex) = 0;
            P(indexCurr, realTaskIndex) = 0;
            decidedIndex[realTaskIndex] = true;
        }
        else if (vecForLP[l].pattern == "preempt" && decidedIndex[realTaskIndex] == false)
        {
            A(indexCurr, realTaskIndex) = 0;
            P(indexCurr, realTaskIndex) = 1;
            decidedIndex[realTaskIndex] = true;
        }
    }

    // second iteration, go through entries that cannot tolerate block or preempt
    for (int i = 0; i < indexFirstAvailable; i++)
    {
        if (decidedIndex[vecForLP[i].index] == false)
        {
            A(indexCurr, i) = 1;
            P(indexCurr, i) = 0;
            decidedIndex[i] = true;
        }
    }
    return true;
}

/**
 * @brief performs in-place update, so requires create A, P at first
 * 
 * @param tasks 
 * @param A 
 * @param B 
 * @return true 
 * @return false 
 */
bool GenerateAP_InWAP(TaskSet &tasks, SquareMatrix &A, SquareMatrix &P)
{
    int N = tasks.size();

    for (int i = 0; i < N - 1; i++)
    {
        bool success = AssignAP_LogicForOneTask(i, tasks, A, P);
        if (debugMode == 1)
        {
            cout << "A " << endl
                 << A << endl
                 << "P" << endl
                 << P << endl;
        }
        if (not success)
            return false;
    }

    if (ResponseTimeWAP(tasks, A, P, N - 1) <= tasks[N - 1].deadline)
        return true;
    else
        return false;
}