import sys
sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from utils_visualize import read_data_2d


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=7,
                    help='Nmax')
parser.add_argument('--methodsNum', type=int, default=4,
                    help='number of optimizers to compare')
parser.add_argument('--title', type=str, default="elimination",
            help='tilte in produced figure')


args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
methodsNum = args.methodsNum
title=args.title

if __name__ == "__main__":
    data_2d = read_data_2d(minTaskNumber, maxTaskNumber, methodsNum)
    dataset_pd = pd.DataFrame()
    optimizer_name=["Dogleg", "LM", "GN", "cSD"]
    marker_list = ["o", "v", "s", "D"]
    color_list = ["#0084DB", "limegreen", "r", "gold"]
    dataset_pd.insert(0, "index", np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber-minTaskNumber+1))
    for i in range(methodsNum):
        dataset_pd.insert(0, optimizer_name[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i], color=color_list[i], markersize=8)

    splot.set(xlabel="Task Number", ylabel="Energy Saving ratio (%)")
    plt.legend(labels=optimizer_name)
    plt.grid(linestyle="--")
    plt.savefig("Compare_" +title+ ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
