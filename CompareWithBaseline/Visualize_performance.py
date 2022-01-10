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
parser.add_argument('--title', type=str, default="elimination",
            help='tilte in produced figure')
parser.add_argument('--method_names', nargs="+", default=["a","b"],
            help='method_names')


args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
title=args.title
method_names=args.method_names
methodsNum=len(method_names)

if __name__ == "__main__":
    data_2d = read_data_2d(minTaskNumber, maxTaskNumber, methodsNum)
    dataset_pd = pd.DataFrame()
    # method_names=["Dogleg", "LM", "GN", "cSD"]
    marker_list = ["o", "v", "s", "D"]
    color_list = ["#0084DB", "limegreen", "r", "gold"]
    dataset_pd.insert(0, "index", np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber-minTaskNumber+1))
    for i in range(methodsNum):
        dataset_pd.insert(0, method_names[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=method_names[i], marker=marker_list[i], color=color_list[i], markersize=8)

    splot.set(xlabel="Task Number", ylabel="Energy Saving ratio (%)")
    plt.legend(labels=method_names)
    plt.grid(linestyle="--")
    plt.savefig("Compare_" +title+ ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
