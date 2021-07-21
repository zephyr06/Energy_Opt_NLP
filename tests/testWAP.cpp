#include <CppUnitLite/TestHarness.h>
#include "../sources/WAP/RTA_WAP.h"

TEST(ResponseTimeWAP, BlockingTime)
{
    // test_data_n3_v2 in python version
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v23.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    int N = tasks.size();
    SquareMatrix A = GenerateZeroMatrix(N);
    SquareMatrix P = GenerateZeroMatrix(N);
    vector<int> blockExpect = {252, 16, 0};
    vector<int> responseExpect = {264, 281, 282};

    for (int i = 0; i < N; i++)
    {
        if (BlockingTime(tasks, A, P, i) != blockExpect[i])
        {
            cout << "Error in ResponseTimeWAP-BlockingTime 1" << i << endl;
        }
        if (ResponseTimeWAP(tasks, A, P, i) != responseExpect[i])
        {
            cout << "Error in ResponseTimeWAP-BlockingTime 2" << i << endl;
        }
    }
}
TEST(ResponseTimeWAP, a2)
{
    // test_data_N3_v2 in python
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v25.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    int N = tasks.size();
    SquareMatrix A = GenerateZeroMatrix(N);
    SquareMatrix P = GenerateOneMatrix(N);

    vector<int> blockExpect = {10, 10, 0};
    vector<int> responseExpect = {155, 197, 298};

    for (int i = 0; i < N; i++)
    {
        if (BlockingTime(tasks, A, P, i) != blockExpect[i])
        {
            cout << "Error in ResponseTimeWAP-BlockingTime 3" << i << endl;
        }
        if (ResponseTimeWAP(tasks, A, P, i) != responseExpect[i])
        {
            cout << "Error in ResponseTimeWAP-BlockingTime 4" << i << endl;
        }
    }

    A = GenerateOneMatrix(N);
    P = GenerateZeroMatrix(N);
    blockExpect = {0, 0, 0};
    responseExpect = {145, 214, 511};
    for (int i = 0; i < N; i++)
    {
        if (BlockingTime(tasks, A, P, i) != blockExpect[i])
        {
            cout << "Error in ResponseTimeWAP-BlockingTime 5" << i << endl;
        }
        if (ResponseTimeWAP(tasks, A, P, i) != responseExpect[i])
        {
            cout << "Error in ResponseTimeWAP-BlockingTime 6" << i << endl;
        }
    }
}

TEST(busyPeriod, a1)
{
    // test_data_N3 in python version
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v24.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    int N = tasks.size();
    SquareMatrix A = GenerateZeroMatrix(N);
    SquareMatrix P = GenerateOneMatrix(N);
    vector<int> busyPeriodExpect = {155, 232, 298};
    for (int i = 0; i < N; i++)
    {
        if (GetBusyPeriod(tasks, A, P, i) != busyPeriodExpect[i])
            cout << "Error in busyPeriod-a1" << i << endl;
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
