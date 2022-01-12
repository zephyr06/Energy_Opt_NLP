import os

import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from utils import Task_Periodic


# def Compute_IJK()

def DBF_all(task0, task1, start0, start1):
    res = 0
    # 0-0
    begin = min(start0, start1)
    end = max(start0 + task0.execution_time, start1 + task1.execution_time)
    value = end - begin - task0.execution_time - task1.execution_time
    if (value > 0):
        return 0
    else:
        return -1 * value


def draw_DBF_2_tasks():
    task0 = Task_Periodic(id=0, offset=0, period=10, overhead=0, execution_time=2, deadline=10)
    task1 = Task_Periodic(id=1, offset=0, period=15, overhead=0, execution_time=3, deadline=15)


    x_data = np.linspace(0,60,300)
    y_data = np.linspace(0,60,300)
    dbf_all = np.zeros(shape=(len(x_data), len(y_data)))
    for start0 in range(len(x_data)):
        for start1 in range(len(y_data)):
            dbf_all[start0, start1] = DBF_all(task0, task1, x_data[start0], y_data[start1])

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    X, Y = np.meshgrid(x_data, y_data)
    ax.plot_surface(X, Y, dbf_all, rstride=1, cstride=1,
                   cmap='viridis', edgecolor='none')
    plt.show()

    one_col=dbf_all[11:42,23]
    plt.plot(range(len(one_col)),one_col)
    plt.show()

    one_row=dbf_all[23,0:]
    plt.plot(range(len(one_row)), one_row)
    plt.show()


def ComputeTimeIJK(start_i, task_i, start_j, task_j, start_k, task_k):
    if(start_i<=start_k and start_k+task_k.execution_time<= start_j+task_j.execution_time):
        return task_k.execution_time
    else:
        return 0

def DBF_all(tasks, start_i):


def draw_DBF_1d_3_tasks():
    task0 = Task_Periodic(id=0, offset=0, period=10, overhead=0, execution_time=2, deadline=10)
    task1 = Task_Periodic(id=1, offset=0, period=15, overhead=0, execution_time=3, deadline=15)
    task2 = Task_Periodic(id=2, offset=0, period=20, overhead=0, execution_time=4, deadline=20)
    tasks={task0, task1, task2}

    start0_array=range(0,100)
    dbf=np.zeros(len(start0_array))
    for start0 in start0_array:
        dbf[start0]=1

    plt.plot(start0_array, dbf)
    plt.show()

if __name__ == '__main__':
    print(os.path.isdir('/home/zephyr/Programming/Uppal_simulation_files/Generate_A_B'))
    draw_DBF_2_tasks()