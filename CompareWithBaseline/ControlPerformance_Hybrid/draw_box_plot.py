import sys

sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


def Read_txt_file_1d_box(path, func):
    """ read txt files, and return a list, each element contains one number"""
    file=open(path, 'r')
    lines=file.readlines()
    res=[]
    count=0
    for line in lines:
        number=float(line)
        res.append(func(number))
        count = count + 1
        if(count > 1000):
            break

    return np.array(res)

def read_data_2d_box(minTaskNumber, maxTaskNumber):
    data_2d={}
    for i in range(minTaskNumber, maxTaskNumber+1):
        file="EnergySaveRatio/N"+str(i)+".txt"
        data_1d=Read_txt_file_1d_box(file, lambda x: (x-1)*100.0)
        data_2d[i]=data_1d
    return data_2d

parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=20,
                    help='Nmax')
parser.add_argument('--methodsNum', type=int, default=4,
                    help='number of optimizers to compare')
parser.add_argument('--data_source', type=str, default="EnergySaveRatio",
                    help='data source folder')
parser.add_argument('--title', type=str, default="elimination",
                    help='tilte in produced figure')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
methodsNum = args.methodsNum
title = args.title
data_source = args.data_source

if __name__ == "__main__":

    data_box_plot=read_data_2d_box(minTaskNumber, maxTaskNumber)
    dataset_pd = pd.DataFrame(dict([(k, pd.Series(v)) for k, v in data_box_plot.items()]))
    ax = sns.boxplot(data=dataset_pd, orient="v", fliersize=1, saturation=0.75, whis=1.5)
    x = np.linspace(-0.5, 15.5, 16)
    y = np.zeros((16))
    ax = sns.lineplot(x=x, y=y, linestyle="dashed", color='darkgray')

    ax.set(xlabel="Task Number", ylabel="Relative Gap (%)")
    plt.savefig("Compare_control_box" + "Zhao20" + ".pdf", format='pdf')
    # plt.savefig("Compare_control_box" +  "Zhao20" + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
