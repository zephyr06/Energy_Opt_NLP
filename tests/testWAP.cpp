#include <CppUnitLite/TestHarness.h>
#include "../sources/WAP/RTA_WAP.h"
#include "../sources/WAP/Generate_A_P.h"
#include "../sources/Optimize.h"

#include <chrono>

using namespace std::chrono;
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

TEST(GenerateAP, a1)
{
    // test_data_N3 in python version
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v26.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    int N = tasks.size();
    SquareMatrix A_actual = GenerateZeroMatrix(N);
    SquareMatrix P_actual = GenerateZeroMatrix(N);

    SquareMatrix A_expect = GenerateZeroMatrix(N);
    SquareMatrix P_expect = GenerateZeroMatrix(N);

    bool success = GenerateAP_InWAP(tasks, A_actual, P_actual);
    if (success)
    {
        if (A_actual.isApprox(A_expect) && P_actual.isApprox(P_expect))
        {
            ;
        }
        else
        {
            cout << "Error in GenerateAP-a1-success" << endl;
        }
    }
    else
        cout << "Error in GenerateAP-a1-fail" << endl;
}

TEST(GenerateAP, a2)
{
    // test_data_N3 in python version
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v27.csv";
    TaskSet tasks = ReadTaskSet(path, "orig");
    int N = tasks.size();
    SquareMatrix A_actual = GenerateZeroMatrix(N);
    SquareMatrix P_actual = GenerateZeroMatrix(N);

    SquareMatrix A_expect;
    A_expect.resize(N, N);
    A_expect << 0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    SquareMatrix P_expect;
    P_expect.resize(N, N);
    P_expect << 0, 1, 1,
        0, 0, 0,
        0, 0, 0;

    bool success = GenerateAP_InWAP(tasks, A_actual, P_actual);
    if (success)
    {
        if (A_actual.isApprox(A_expect) && P_actual.isApprox(P_expect))
        {
            ;
        }
        else
        {
            cout << "Error in GenerateAP-a2" << endl;
        }
    }
    else
        cout << "Error in GenerateAP-a2-flag" << endl;
}

// TEST(GenerateAP, a3)
// {
//     // test_data_N3 in python version
//     string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v28.csv";
//     TaskSet tasks = ReadTaskSet(path, "orig");
//     int N = tasks.size();
//     SquareMatrix A_actual = GenerateZeroMatrix(N);
//     SquareMatrix P_actual = GenerateZeroMatrix(N);

//     SquareMatrix A_expect;
//     A_expect.resize(N, N);
//     A_expect << 0, 0, 0,
//         0, 0, 0,
//         0, 0, 0;
//     SquareMatrix P_expect;
//     P_expect.resize(N, N);
//     P_expect << 0, 0, 0,
//         0, 0, 1,
//         0, 0, 0;

//     bool success = GenerateAP_InWAP(tasks, A_actual, P_actual);
//     if (success)
//     {
//         cout << "Error in GenerateAP-a3" << endl;
//         cout << A_actual << endl
//              << P_actual << endl;
//     }
//     else
//         ;
// }

TEST(OPT, WAP)
{
    string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/" + testDataSetName + ".csv";
    // string path = "/home/zephyr/Programming/Energy_Opt_NLP/TaskData/test_n3_v27.csv";
    TaskSet tasks = ReadTaskSet(path, "RM");
    int N = tasks.size();
    bool success;
    if (not fully_preemptive)
    {
        A_Global = GenerateZeroMatrix(N);
        P_Global = GenerateZeroMatrix(N);
        cout << "A" << endl
             << A_Global << endl
             << "P" << endl
             << P_Global << endl;
        success = GenerateAP_InWAP(tasks, A_Global, P_Global);
        // success = 1;
    }
    else
    {
        success = 1;
        A_Global = GenerateZeroMatrix(N);
        P_Global = GenerateOneMatrix(N);
    }

    if (success)
    {
        cout << "A" << endl
             << A_Global << endl
             << "P" << endl
             << P_Global << endl;
        auto start = chrono::high_resolution_clock::now();
        double res = OptimizeTaskSet(tasks);
        cout << blue << "The energy saving ratio is " << res << def << endl;
        auto stop = chrono::high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stop - start);
        cout << "The time taken is: " << double(duration.count()) / 1e6 << " seconds" << endl;
    }
    else
    {
        cout << "A" << endl
             << A_Global << endl
             << "P" << endl
             << P_Global << endl;
        cout << "Test failed in OPT-WAP because the task set is not schedulable" << endl;
    }
}

int main()
{
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
