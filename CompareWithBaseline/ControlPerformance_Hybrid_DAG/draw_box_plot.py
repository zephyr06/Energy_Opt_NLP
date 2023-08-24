import sys

sys.path.append("/")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


def Read_txt_file_1d_box(path, func, data_readin_size):
    """ read txt files, and return a list, each element contains one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = []
    count = 0
    for line in lines:
        number = float(line)
        res.append(func(number))
        count = count + 1
        if (count > data_readin_size):
            print("The input file provides too much data!!!!!")
            break

    return np.array(res)


def ChangeDataBaseline(data_1d, func, task_set_size, baseline_data_index, main_data_index):
    """the input is 2000*1=[a;b], the output is 1000*1=[a/b], for the visualization for ControlPerformance_Hybrid"""
    data_res = [0] * task_set_size
    for i in range(task_set_size):
        data_res[i] = func(data_1d[i + task_set_size * main_data_index] / data_1d[i + task_set_size * baseline_data_index])
        if (data_res[i] > 0.1):
            print(i, data_1d[i + task_set_size * main_data_index] , data_1d[i + task_set_size * baseline_data_index], data_res[i])
            aa = 1
    return data_res


def read_data_2d_box(minTaskNumber, maxTaskNumber, task_set_size,  data_source, baseline_data_index, main_data_index):
    data_2d = {}
    for i in range(minTaskNumber, maxTaskNumber + 1):
        file = data_source + "/" + "N" + str(i) + ".txt"
        data_1d = Read_txt_file_1d_box(file, lambda x: x, task_set_size * (main_data_index + 1))
        data_1d = ChangeDataBaseline(data_1d, lambda x: (x - 1) * 100.0, task_set_size, baseline_data_index, main_data_index)
        data_2d[i] = data_1d
    return data_2d


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=3,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=3,
                    help='Nmax')
parser.add_argument('--task_set_size', type=int, default=100,
                    help='task_set_size')
parser.add_argument('--data_source', type=str, default="EnergySaveRatio",
                    help='data source folder')
parser.add_argument('--baseline_data_index', type=str, default="0",
                    help='the baseline data for drawing the box plot; by default, it would be the first 1000 data')
parser.add_argument('--main_data_index', type=str, default="1",
                    help='the method for drawing the box plot; by default, it would be the second 1000 data')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
task_set_size = args.task_set_size
data_source = args.data_source
baseline_data_index = int(args.baseline_data_index)
main_data_index = int(args.main_data_index)

if __name__ == "__main__":
    data_box_plot = read_data_2d_box(minTaskNumber, maxTaskNumber, task_set_size,  data_source, baseline_data_index,
                                     main_data_index)
    dataset_pd = pd.DataFrame(dict([(k, pd.Series(v)) for k, v in data_box_plot.items()]))
    ax = sns.boxplot(data=dataset_pd, orient="v", fliersize=1, saturation=0.75, whis=1.5)
    x = np.linspace(-0.5, 15.5, 16)
    y = np.zeros((16))
    plt.ylim(-80, 40)
    ax = sns.lineplot(x=x, y=y, linestyle="dashed", color='darkgray')

    ax.set(xlabel="Task Number", ylabel="Relative Gap (%)")
    plt.savefig( "Compare_control_box_" +str(main_data_index)+ ".pdf", format='pdf')
    # plt.savefig("Compare_control_box" +  "Zhao20" + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
