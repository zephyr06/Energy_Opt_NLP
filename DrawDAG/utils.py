import os

import numpy as np


class Task_Periodic:
    id = 0
    offset = 0
    period = 0
    overhead = 0
    execution_time = 0.0
    deadline = 0

    def __init__(self, id, offset, period, overhead=0, execution_time=0, deadline=0):
        self.id = id
        self.offset = offset
        self.period = period
        self.overhead = overhead
        self.execution_time = np.float(execution_time)
        self.deadline = deadline

    def print(self):
        print("The tasks parameters are ", "id", self.id, "Period: ", self.period,
              "execution_time: ", self.execution_time,
              "overhead", self.overhead, "deadline: ", self.deadline)

    @staticmethod
    def Execution_time(self):
        return self.execution_time

    @staticmethod
    def Period(self):
        return self.period

    @staticmethod
    def Utilization(self):
        return self.execution_time / np.float(self.period)

    @staticmethod
    def Slack(self):
        return self.deadline - self.execution_time


class Data_meta_task:
    value = 0
    id = 0
    pattern = '0'  # 'a' means block cost incurred by NO abortion, 'b' means block cost incurred by preemption

    def __init__(self, value, id, pattern):
        self.value = value
        self.id = id
        self.pattern = pattern

    def print(self):
        print("The value of the entry is ", self.value, "\nThe id: ", self.id, "\nThe pattern: ", self.pattern, "\n")

    @staticmethod
    def Value(self):
        return self.value


def Reassign_id(task_set):
    N = len(task_set)
    for i in range(N):
        task_set[i].id = i
    return task_set


def Change_Priority(task_set, method):
    """
    Given a task set, change priorities based on fixed priority method
    --method == "original": don't change order
    --method == "RM", jobs with small period have highest priority
    --method == "util", jobs with high utilization have highest priority

    :param task_set:
    :param method:
    :return:
    """
    if method == "orig":
        return task_set
    elif method == "RM":
        task_set.sort(key=Task_Periodic.Period)
    elif method == "util":
        task_set.sort(key=Task_Periodic.Utilization, reverse=0)
    elif method == "exec":
        task_set.sort(key=Task_Periodic.Execution_time, reverse=0)
    elif method == "slack":
        task_set.sort(key=Task_Periodic.Slack, reverse=0)
    else:
        raise Exception("The only supported read options include: orig, RM, util")

    task_set = Reassign_id(task_set)
    return task_set


# !!! Modify the default back to "orig" after finishing experiments, otherwise, some unit tests will fail
def Read_task_set(path='../TaskData/periodic-set-1-syntheticJobs.csv', priority_option="orig"):
    if not os.path.exists(path):
        raise Exception("Error: The given path doesn't exist in Read_task_set function!")

    file = open(path, 'r')
    lines_task = file.readlines()[1:]

    task_set = []
    for line in lines_task:
        parameter = line.split(',')
        task_set.append(Task_Periodic(int(parameter[0]), int(parameter[1]),
                                      int(parameter[2]), int(parameter[3]),
                                      int(parameter[4]), int(parameter[5])))

    # print("Task sets from the given file is: *********************************")
    # for task in task_set:
    #     task.print()
    # print('*********************************')

    return Change_Priority(task_set, priority_option)


def Get_hyper_period(task_set: [Task_Periodic]):
    N = len(task_set)
    hyper = task_set[0].period
    if N < 2:
        return hyper
    else:
        for i in range(1, N):
            hyper = np.lcm(hyper, task_set[i].period)
        return hyper


def response_time_stan(execution_time_curr, hp_period, hp_execution):
    """
    This function analyzes response time using standard iteration method
    :param execution_time_curr: current task's execution time
    :param hp_period: higher priority tasks' period list
    :param hp_execution: higher priority tasks' execution time list
    :return:
        response time for current task, calculated by solving the following iteration:
            r = c + Sum_i( ceil(r/T_i)c_i )
    """
    N = len(hp_period)
    hp_period = np.array(hp_period)
    hp_execution = np.array(hp_execution)
    utilization = np.sum(hp_execution / hp_period)
    if utilization >= 1 - 1e-4:
        # print("The given system is unschedulable")
        return np.inf

    if len(hp_period) != len(hp_execution):
        print("Please check input sequence length")
        return np.inf
    if execution_time_curr != 0:
        response_before = execution_time_curr
    else:
        for tt in hp_execution:
            response_before = np.sum(hp_execution)

    stop_flag = False
    while not stop_flag:
        response = execution_time_curr
        for i in range(N):
            response += np.ceil(response_before / np.float(hp_period[i])) * hp_execution[i]
        if response == response_before:
            stop_flag = True
            return response
        else:
            response_before = response

    return np.inf


def NumericalDerivative(fun, x0, step, *args):
    """

    :param fun:
    :param x0: one dimension list or np.array
    :param step:
    :param args:
    :return:
    """
    x0=np.array(x0)
    N = len(x0)

    value_curr = np.array(fun(x0, *args))
    M = len(value_curr)



    jac = np.zeros((M,N))
    for i in range(M):
        for j in range(N):
            x0[j]=x0[j]-step
            y_prev=fun(x0, *args)
            x0[j]=x0[j]+step*2
            y_next=fun(x0, *args)
            jac[i][j] = (y_next[i]-y_prev[i])/2/step
            x0[j] = x0[j] - step
    return jac


if __name__ == '__main__':




    # path = '../TaskData/test_data_N3.csv'
    # taskset = Read_task_set(path, "RM")
    # np.testing.assert_almost_equal(taskset[0].period, 450)
    #
    # path = '../TaskData/test_data_N3.csv'
    # tasksets = Read_task_set(path)
    # assert len(tasksets) == 3
    a=1
