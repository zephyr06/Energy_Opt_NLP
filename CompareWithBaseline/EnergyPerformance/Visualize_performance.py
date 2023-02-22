import sys

sys.path.append("../")
import argparse
import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


def read_data_2d_energy(minTaskNumber, maxTaskNumber):
    def extract_ave(method_index):
        ave = 0
        for i in range(method_index * 1000, method_index * 1000 + 1000):
            ave += float(lines[i])
        return ave / 1000.0

    data2d = []
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = "EnergySaveRatio" + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NLP, with elimination
        data.append(extract_ave(0))

        # NLP, with elimination, exact Jacobian
        data.append(extract_ave(1))

        # NLP, without elimination
        data.append(extract_ave(2))

        # MUA
        data.append(1.0)

        # MILP, maximum number is 15
        if (task_number < 12):
            ave = 0
            for i in range(1000):
                ave += float(lines[4000 + i]) / float(lines[3000 + i])
            data.append(ave / 1000)
        else:
            data.append(-1)

        data2d.append(data)
        file.close()
    return data2d


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=30,
                    help='Nmax')
parser.add_argument('--title', type=str, default="EnergyPerformance",
                    help='tilte in produced figure')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
title = args.title

if __name__ == "__main__":
    data_2d = read_data_2d_energy(minTaskNumber, maxTaskNumber)
    data_2d = np.array(data_2d).transpose()
    data_2d = (data_2d - 1) * 100

    dataset_pd = pd.DataFrame()
    optimizer_name = ["NORTH", "NMBO", "IPM", "Zhao20", "MIGP"]
    marker_list = ["o", "v", "^", "s", "D"]  #
    color_list = ["#0084DB", "r", "y", "limegreen", "purple"]
    dataset_pd.insert(0, "index", np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber - minTaskNumber + 1))
    for i in range(min(data_2d.shape[0], 3)):
        dataset_pd.insert(0, optimizer_name[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i],
                             color=color_list[i], markersize=8, label = optimizer_name[i])

    # Zhao20
    i = 3
    dataset_pd.insert(0, optimizer_name[i], data_2d[i])
    splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i], color=color_list[i],
                         markersize=8, label = optimizer_name[i])

    font_size = 15
    plt.rcParams.update({'font.size': font_size / 1.2})
    # MILP
    plt.plot(np.linspace(minTaskNumber, min(11, maxTaskNumber), min(11, maxTaskNumber) - minTaskNumber + 1),
             data_2d[-1][:min(11, maxTaskNumber) - minTaskNumber + 1], marker=marker_list[-1], color=color_list[-1],
             markersize=8, label = optimizer_name[-1])

    plt.xlabel("Task Number", fontsize=font_size)
    plt.ylabel("Relative gap with Zhao20 (%)", fontsize=font_size)
    splot.set_ylim([-5, 130])
    plt.legend()
    plt.grid(linestyle="--")
    plt.savefig("Compare_" + title + "_" + "EnergySaveRatio" + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
