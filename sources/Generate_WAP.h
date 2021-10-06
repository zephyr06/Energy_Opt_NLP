#pragma once

#include <tuple>
#include <string.h>
#include "RTA_WAP.h"

class DataMetaWAP
{
public:
    double value;
    int index;
    char pattern;
    /**
     * @brief Construct a new Data Meta WAP object, this function is mainly used for GenerateWAP
     * 
     * @param value 
     * @param index: this is not task id!!! 
     * @param pattern 
     */
    DataMetaWAP(double value, int index, char pattern) : value(value), index(index), pattern(pattern) {}

    void print()
    {
        cout << "The value of the entry is " << value << "\nThe index (Not id): " << index << "\nThe pattern: " << pattern << endl;
    }
};
bool compareDataMetaWAP(DataMetaWAP &a, DataMetaWAP &b)
{
    return a.value > b.value;
}

/**
 * @brief 
 * 
 * ***************************************************Explanation**********************************************************
    We analyze AB from hp to lp, for the current considered task, we only need to worry about lp part because hp part
    is already fixed and optimal; The influence of lp part is by blocking, which could be caused by 'abort' type or
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
************************************************************************************************************************
 * 
 * @param index 
 * @param tasks 
 * @param A in-place update
 * @param P in-place update
 * @return true 
 * @return false 
 */
bool AssignLogicWAP(int index, const TaskSet &tasks, MatrixDynamic &A, MatrixDynamic &P)
{
    int N = tasks.size();
    vector<DataMetaWAP> list4Sort;
    for (int i = index + 1; i < N; i++)
    {
        list4Sort.push_back(DataMetaWAP(tasks[i].executionTime - 1, i, 'a'));
        list4Sort.push_back(DataMetaWAP(tasks[i].overhead, i, 'p'));
    }
    sort(list4Sort.begin(), list4Sort.end(), compareDataMetaWAP);

    bool success = false;
    int index_first_available = -1;
    for (size_t i = 0; i < list4Sort.size(); i++)
    {
        if (RTA_WAP::RTA_block(tasks, A, P, index, list4Sort[i].value) <= tasks.at(index).deadline)
        {
            success = true;
            index_first_available = i;
            break;
        }
    }

    if (!success)
    {
        // The current task cannot tolerate any block from lp tasks
        // Let's try if all block items are 0, will it be schedulable
        if (RTA_WAP::RTA_block(tasks, A, P, index, 0) <= tasks.at(index).deadline)
        {
            success = true;
            for (int l = index + 1; l < N; l++)
            {
                A(index, l) = 1;
                A(index, l) = 0;
            }
            return success;
        }
        else
            // not schedulable!
            return false;
    }
    else
    {
        // Update A P based on index_first_available
        vector<bool> decidedEntry(N, 0);
        // first iteration, go through entries after the first entry found
        for (size_t i = index_first_available; i < list4Sort.size(); i++)
        {
            DataMetaWAP entry = list4Sort[i];
            if (entry.pattern == 'a')
            {
                A(index, entry.index) = 0;
                P(index, entry.index) = 0;
                decidedEntry[entry.index] = 1;
            }
            else if (entry.pattern == 'p' && decidedEntry[entry.index] == 0)
            {
                A(index, entry.index) = 0;
                P(index, entry.index) = 1;
                decidedEntry[entry.index] = 1;
            }
        }
        // second iteration, go through entries before the first entry found
        for (size_t i = 0; i < index_first_available; i++)
        {
            DataMetaWAP entry = list4Sort[i];
            if (decidedEntry[entry.index] == 0)
            {
                A(index, entry.index) = 1;
                P(index, entry.index) = 0;
                decidedEntry[entry.index] = 1;
            }
        }
        return true;
    }
}

tuple<bool, MatrixDynamic, MatrixDynamic> Generate_WAP(const TaskSet &tasks)
{
    int N = tasks.size();
    MatrixDynamic A = GenerateZeroMatrix(N, N);
    MatrixDynamic P = GenerateZeroMatrix(N, N);

    for (int i = 0; i < N - 1; i++)
    {
        if (not AssignLogicWAP(i, tasks, A, P))
        {
            // Not schedulable
            return make_tuple(0, A, P);
        }
    }

    // check the last task

    if (RTA_WAP::RTA_Common(tasks, N - 1) <= tasks.at(N - 1).deadline)
    {
        return make_tuple(1, A, P);
    }
    else
    {
        return make_tuple(0, A, P);
    }
    return make_tuple(0, A, P);
}