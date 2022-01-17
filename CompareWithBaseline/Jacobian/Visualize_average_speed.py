import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
# from utils_visualize import *


def Read_txt_file_1d(path, func, delimiter=','):
    """ read txt files, and return a 2D list, each element contains MORE THAN one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = []
    for line in lines:
        res.append(func(float(line)))
    return np.array(res)

def Extract2d(data1d, minN, maxN):
    res=[]
    res.append(data1d[:(maxN-minN+1)])
    res.append(data1d[(maxN - minN + 1):])
    return res
parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=20,
                    help='Nmax')
parser.add_argument('--taskSetNumber', type=int, default=100,
                    help='taskSetNumber')
parser.add_argument('--baseline', type=str, default="ExactJacobian",
                    help='baseline')
parser.add_argument('--ylim', type=float, default=1e2,
                    help='ylim')
args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
taskSetNumber = args.taskSetNumber
baseline = args.baseline
ylim=args.ylim

baseline="ExactJacobian"
proposedMethod="ApproximateJacobian"

path = "time_task_number.txt"
data_1d = Read_txt_file_1d(path, lambda x: x).T
data_2d = Extract2d(data_1d, minTaskNumber, maxTaskNumber)
task_number_seq = range(minTaskNumber, maxTaskNumber + 1)
df=pd.DataFrame({"index":task_number_seq,proposedMethod:data_2d[0], baseline: data_2d[1]})
# df=pd.DataFrame({"NLP":data_2d[0,:], baseline: data_2d[1,:]})
# df = pd.DataFrame(data=data_2d.T, columns={"NLP", baseline}, index=range(minTaskNumber, maxTaskNumber+1))


splot = sns.lineplot(data=df, x="index", y=proposedMethod,  marker="*", markersize=12)
splot = sns.lineplot(data=df, x="index", y=baseline, marker="o", markersize=8)
plt.legend(labels=[proposedMethod,baseline])
splot.set(yscale="log")
plt.grid(linestyle="--")
splot.set(xlabel="Task Number", ylabel="Runt-Time (seconds)")
splot.set_xlim(4, None)
splot.set_ylim(1e-4, ylim)
plt.savefig("Compare_run_time" +baseline+ ".pdf", format='pdf')
plt.savefig("Compare_run_time" +baseline+ ".png", format='png')
plt.show(block=False)
plt.pause(3)
