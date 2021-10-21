import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import argparse
import cv2


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=20,
                    help='Nmax')
parser.add_argument('--taskSetNumber', type=int, default=100,
                    help='taskSetNumber')
parser.add_argument('--baseline', type=str, default="MUA",
                    help='baseline')
parser.add_argument('--ylim', type=float, default=1e0,
                    help='ylim')
args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
taskSetNumber = args.taskSetNumber
baseline = args.baseline
ylim=args.ylim

def Read_txt_file_1d(path, func):
    """ read txt files, and return a list, each element contains one number"""
    file=open(path, 'r')
    lines=file.readlines()
    res=[]
    for line in lines:
        number=float(line)
        res.append(func(number))
    return np.array(res)


path = "weight_task_number.txt"
fs = cv2.FileStorage("../sources/parameters.yaml", cv2.FILE_STORAGE_READ)
weightDrawBegin = fs.getNode("weightDrawBegin").real()
weightDrawEnd = fs.getNode("weightDrawEnd").real()

data_number=int(np.log(weightDrawEnd/weightDrawBegin)/np.log(10))+1
data_1d = Read_txt_file_1d(path, lambda x: x) * 100.0
data_x=[]
for i in range(data_number):
    data_x.append(weightDrawBegin * np.power(10, i))

df=pd.DataFrame({"index":data_x,"weight_trajectory":data_1d})

splot = sns.lineplot(data=df, x="index", y="weight_trajectory",  marker="*", markersize=12)

# plt.legend(labels=["NLP",baseline])
# splot.set(yscale="log")
splot.set(xscale="log")
plt.grid(linestyle="--")
splot.set(xlabel="weight w", ylabel="Energy saving ratio (%)")
# splot.set_xlim(4, None)
# splot.set_ylim(1e-3, ylim)
plt.savefig("weight_task_number" + ".pdf", format='pdf')
plt.savefig("weight_task_number" +".png", format='png')
plt.show(block=False)
plt.pause(3)
