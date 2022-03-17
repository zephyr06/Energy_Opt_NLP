import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
# from utils_visualize import *


def Read_txt_file_2d(path, func, delimiter=','):
    """ read txt files, and return a 2D list, each element contains MORE THAN one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = []
    for line in lines:
        list1 = line.split(delimiter)
        n = len(list1)
        for i in range(n):
            list1[i] = func(float(list1[i]))
        res.append(list1)
    return np.array(res)

parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=6,
                    help='Nmax')
parser.add_argument('--taskSetNumber', type=int, default=100,
                    help='taskSetNumber')
parser.add_argument('--baseline', type=str, default="MUA",
                    help='baseline')
parser.add_argument('--ylim', type=float, default=1e2,
                    help='ylim')
args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
taskSetNumber = args.taskSetNumber
baseline = args.baseline
ylim=args.ylim



path = "time_task_number.txt"
data_2d = Read_txt_file_2d(path, lambda x: x).T
task_number_seq = range(minTaskNumber, maxTaskNumber + 1)
df=pd.DataFrame({"index":task_number_seq,"NLP":data_2d[0,:], baseline: data_2d[1,:]})
# df=pd.DataFrame({"NLP":data_2d[0,:], baseline: data_2d[1,:]})
# df = pd.DataFrame(data=data_2d.T, columns={"NLP", baseline}, index=range(minTaskNumber, maxTaskNumber+1))

# line, ax=plt.subplots()
# ax=sns.lineplot(x="index", y="NLP", data=df)

splot = sns.lineplot(data=df, x="index", y="NLP",  marker="*", markersize=12)
splot = sns.lineplot(data=df, x="index", y=baseline, marker="o", markersize=8)
plt.legend(labels=["NLP",baseline])
splot.set(yscale="log")
plt.grid(linestyle="--")
splot.set(xlabel="Task Number", ylabel="Runt-Time (seconds)")
splot.set_xlim(4, None)
splot.set_ylim(1e-4, ylim)
plt.savefig("Compare_run_time" +baseline+ ".pdf", format='pdf')
plt.savefig("Compare_run_time" +baseline+ ".png", format='png')
plt.show(block=False)
plt.pause(3)
