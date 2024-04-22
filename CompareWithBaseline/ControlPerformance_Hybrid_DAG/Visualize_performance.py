import sys

sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt



def read_data_2d(minTaskNumber, maxTaskNumber, type="Time"):
    data2d = []
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = type + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NORTH
        data.append(float(lines[0]))

        # NORTH+Gradient
        data.append(float(lines[1]))

        # NORTH+RM
        data.append(float(lines[2]))

        # NORTH+Coeff
        data.append(float(lines[3]))

        data2d.append(data)
        file.close()

    return data2d
def relative_gap(val, base):
    return (val-base)/base*100

def read_data_2d_normalize(minTaskNumber, maxTaskNumber, type="Time"):
    data2d = []
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = type + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        base = float(lines[0])
        # NORTH
        data.append(0.0)

        # NORTH+Gradient
        data.append(relative_gap(float(lines[1]), base))

        # NORTH+RM
        data.append(relative_gap(float(lines[2]), base))

        # NORTH+Coeff
        data.append(relative_gap(float(lines[3]), base))

        data2d.append(data)
        file.close()

    return data2d


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=3,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=20,
                    help='Nmax')
parser.add_argument('--methodsNum', type=int, default=4,
                    help='number of optimizers to compare')
parser.add_argument('--data_source', type=str, default="Time",
                    help='data source folder, EnergySaveRatio_Average/RTA/Time')
parser.add_argument('--title', type=str, default="ControlPerformance_Hybrid_DAG",
                    help='tilte in produced figure')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
methodsNum = args.methodsNum
title = args.title
data_source = args.data_source

if __name__ == "__main__":
    data_2d = read_data_2d(minTaskNumber, maxTaskNumber, data_source)
    if (data_source == "EnergySaveRatio_Average"):
        data_2d = read_data_2d_normalize(minTaskNumber, maxTaskNumber, data_source)

    data_2d = np.array(data_2d).transpose()
    dataset_pd = pd.DataFrame()
    optimizer_name = ["NORTH", "NORTH+PAOpt", "NORTH+RM", "NORTH+OnlyGrad"] # , "NORTH+_Sort"
    marker_list = ["o", "*", "v", "s", "D"]  #
    marker_size = [8,13,10,7]
    color_list = ["#0084DB", "#00cd6c", "#a6761d", "#ffc61e", "purple"]  #
    dataset_pd.insert(0, "index", np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber - minTaskNumber + 1))
    for i in range(min(data_2d.shape[0], 4)):
        dataset_pd.insert(0, optimizer_name[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i],
                             color=color_list[i], markersize=marker_size[i], label = optimizer_name[i])


    font_size = 15
    plt.rcParams.update({'font.size': font_size / 1.2})
    # splot.set(xlabel="Task Number", fontsize=font_size)
    splot.set_xlabel("Task Number", fontsize=font_size)
    splot.set_xticks([i for i in range(3,22,2)])
    # splot.set_ylim([0.95, 2.0])
    if(data_source=="Time"):
        splot.set(yscale="log")
        splot.set_ylim(5e-2, 1e3)
        # splot.set( ylabel="Running time (seconds)", fontsize=font_size)
        splot.set_ylabel("Running time (seconds)", fontsize=font_size)
    elif (data_source=="EnergySaveRatio_Average"):
        splot.set_ylim(-50, 5)
        # splot.set(ylabel="Relative gap (%)", fontsize=font_size)
        splot.set_ylabel("Relative gap with NORTH (%)", fontsize=font_size)
    plt.legend()
    splot.set_xlim([2, 21])
    plt.grid(linestyle="--")
    plt.savefig("Compare_" + title + "_" + data_source + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
