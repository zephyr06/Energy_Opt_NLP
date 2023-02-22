import sys

sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


def read_data_2d_time(minTaskNumber, maxTaskNumber):
    data2d = []

    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = "Time" + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NORTH
        data.append(float(lines[0]))

        # NMBO
        data.append(float(lines[1]))

        # IPM
        data.append(float(lines[2]))

        # MUA
        data.append(float(lines[3]))

        # MILP, maximum number is 15
        if (task_number < 12):
            data.append(float(lines[4]))
        else:
            data.append(-1)

        data2d.append(data)
        file.close()

    # N = 40 ~ 80
    for task_number in range(40, 90, 10):
        file_path = "Time" + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NORTH
        data.append(float(lines[0]))

        # NMBO
        data.append(float(lines[1]))

        # IPM
        data.append(float(lines[2]))

        # MUA
        data.append(-1)

        # MILP, maximum number is 15
        data.append(-1)

        data2d.append(data)
        file.close()
    return data2d


def read_data_2d_rta(minTaskNumber, maxTaskNumber):
    def extract_ave(method_index):
        ave = 0
        for i in range(method_index * 1000, method_index * 1000 + 1000):
            ave += float(lines[i])
        return ave / 1000.0

    data2d = []
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = "RTACalling" + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NORTH
        data.append(extract_ave(0))

        # NMBO
        data.append(extract_ave(1))

        # IPM
        data.append(extract_ave(2))

        data2d.append(data)
        file.close()

    # N = 40 ~ 80
    for task_number in range(40, 90, 10):
        file_path = "RTACalling" + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NORTH
        data.append(float(lines[0]))

        # NMBO
        data.append(float(lines[1]))

        # IPM
        data.append(float(lines[2]))

        data2d.append(data)
        file.close()
    return data2d


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=30,
                    help='Nmax')
parser.add_argument('--methodsNum', type=int, default=4,
                    help='number of optimizers to compare')
parser.add_argument('--data_source', type=str, default="Time",
                    help='data source folder')
parser.add_argument('--title', type=str, default="EnergyPerformance",
                    help='tilte in produced figure')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
methodsNum = args.methodsNum
title = args.title
data_source = args.data_source

if __name__ == "__main__":
    if (data_source == "Time"):
        data_2d = read_data_2d_time(minTaskNumber, maxTaskNumber)
    elif (data_source == "RTA"):
        data_2d = read_data_2d_rta(minTaskNumber, maxTaskNumber)

    data_2d = np.array(data_2d).transpose()
    dataset_pd = pd.DataFrame()
    optimizer_name = ["NORTH", "NMBO", "IPM", "Zhao20", "MIGP"]
    marker_list = ["o", "v", "^", "s", "D"]  #
    color_list = ["#0084DB", "r", "y", "limegreen", "purple"]
    # dataset_pd.insert(0, "index", np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber-minTaskNumber+1))

    long_index = []
    for i in range(5, 31):
        long_index.append(i)
    for i in [40, 50, 60, 70, 80]:
        long_index.append(i)
    dataset_pd.insert(0, "index", long_index)
    for i in range(3):
        dataset_pd.insert(0, optimizer_name[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i],
                             color=color_list[i], markersize=8, label = optimizer_name[i])
        # plt.plot(long_index, data_2d[i], marker=marker_list[i], color=color_list[i], markersize=8)

    # Zhao20
    if (data_source == "Time"):
        i = 3
        plt.plot(np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber - minTaskNumber + 1),
                 data_2d[i][:maxTaskNumber - minTaskNumber + 1], marker=marker_list[i], color=color_list[i],
                 markersize=8, label = optimizer_name[i])

        # MILP
        if (data_source == "EnergySaveRatio" or data_source == "Time"):
            plt.plot(np.linspace(minTaskNumber, min(11, maxTaskNumber), min(11, maxTaskNumber) - minTaskNumber + 1),
                     data_2d[-1][:min(11, maxTaskNumber) - minTaskNumber + 1], marker=marker_list[-1],
                     color=color_list[-1], markersize=8, label = optimizer_name[-1])

    if (data_source == "EnergySaveRatio"):
        plt.set(xlabel="Task Number", ylabel="Relative gap with Zhao20 (%)")
        splot.set_ylim([0.95, 2.1])
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_" + title +"_"+ data_source + ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)
    elif (data_source == "Time"):
        splot.set(xlabel="Task Number", ylabel="Running time (seconds)")
        splot.set_xlim([0, 85])
        splot.set_ylim([1e-4, 1e3])
        splot.set(yscale="log")
        # splot.set_ylim(1e-4, 1e3)
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_" + title + "_"+data_source + "Long" + ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)
    elif (data_source == "RTA"):
        splot.set(xlabel="Task Number", ylabel="Schedulability analysis call times")
        # splot.set_ylim([0.95, 2.0])
        # splot.set(yscale="log")
        # splot.set_ylim(1e-4, 1e3)
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_" + title + "_"+data_source + "Long" + ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)
