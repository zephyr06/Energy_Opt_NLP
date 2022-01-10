import sys
import argparse

import numpy as np

sys.path.insert(1, "../")
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def read_data_2d():
    data2d = []
    for optimizer_index in range(optimizerNum):
        data = []
        for i in range(minTaskNumber, maxTaskNumber + 1):
            file_path = "EnergySaveRatio/N" + str(i) + ".txt"
            file = open(file_path, "r")
            lines = file.readlines()
            data.append(float(lines[optimizer_index - 1]))
            file.close()
        data2d.append(data)
    return data2d


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=7,
                    help='Nmax')
parser.add_argument('--optimizerNum', type=int, default=2,
                    help='number of optimizers to compare')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
optimizerNum = args.optimizerNum

if __name__ == "__main__":
    data_2d = read_data_2d()
    dataset_pd = pd.DataFrame()
    optimizer_name=["Dogleg", "LM", "GN", "cSD"]
    marker_list = ["o", "v", "s", "D"]
    color_list = ["#0084DB", "limegreen", "r", "gold"]
    dataset_pd.insert(0, "index", np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber-minTaskNumber+1))
    for i in range(optimizerNum):
        dataset_pd.insert(0, optimizer_name[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i], color=color_list[i], markersize=8)

    splot.set(xlabel="Task Number", ylabel="Energy Saving ratio (%)")
    plt.legend(labels=optimizer_name)
    plt.grid(linestyle="--")
    plt.show(block=False)
    plt.pause(3)
