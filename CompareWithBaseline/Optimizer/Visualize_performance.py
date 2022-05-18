import sys
sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def read_data_2d_energy(minTaskNumber, maxTaskNumber):
    def extract_ave(method_index):
        ave = 0
        for i in range(method_index*1000, method_index*1000+1000):
            ave += float(lines[i])
        return ave/1000.0
    data2d = []
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = "EnergySaveRatio"+"/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data=[]

        # LM
        data.append(extract_ave(0))

        # Dogleg
        data.append(extract_ave(1))

        # GN
        data.append(extract_ave(2))

        # cGD
        data.append(extract_ave(3))

        # Zhao20
        data.append(1.0)

        data2d.append(data)
        file.close()
    return data2d

def read_data_2d_rta(minTaskNumber, maxTaskNumber):
    def extract_ave(method_index):
        ave = 0
        for i in range(method_index*1000, method_index*1000+1000):
            ave += float(lines[i])
        return ave/1000.0
    data2d = []
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = "RTACalling"+"/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data=[]

        # LM
        data.append(extract_ave(0))

        # Dogleg
        data.append(extract_ave(1))

        # GN
        data.append(extract_ave(2))

        # cGD
        data.append(extract_ave(3))

        data2d.append(data)
        file.close()
    return data2d
def read_data_2d_time(minTaskNumber, maxTaskNumber):
    data2d = []
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = "Time"+"/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data=[]

        # LM
        data.append(float(lines[0]))

        # DL
        data.append(float(lines[1]))

        # GN
        data.append(float(lines[2]))

        # cGD
        data.append(float(lines[3]))

        data2d.append(data)
        file.close()

        file.close()
    return data2d

parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=30,
                    help='Nmax')
parser.add_argument('--methodsNum', type=int, default=4,
                    help='number of optimizers to compare')
parser.add_argument('--data_source',type=str, default="EnergySaveRatio",
                    help='data source folder')
parser.add_argument('--title', type=str, default="Optimizer",
            help='tilte in produced figure')


args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
methodsNum = args.methodsNum
title=args.title
data_source=args.data_source

if __name__ == "__main__":
    if(data_source=="EnergySaveRatio"):
        data_2d = read_data_2d_energy(minTaskNumber, maxTaskNumber)
    elif (data_source == "Time"):
        data_2d = read_data_2d_time(minTaskNumber, maxTaskNumber)
    elif (data_source == "RTA"):
        data_2d = read_data_2d_rta(minTaskNumber, maxTaskNumber)

    data_2d=np.array(data_2d).transpose()
    if (data_source == "EnergySaveRatio"):
        data_2d = data_2d * 100
    dataset_pd = pd.DataFrame()
    optimizer_name=["LM","Dogleg", "Gauss-Newton",  "cGD", "Zhao20"]
    marker_list = ["o", "v", "x", "*", "D"] #
    color_list = ["#0084DB","cyan", "limegreen", "r", "gold"] #
    dataset_pd.insert(0, "index", np.linspace(minTaskNumber, maxTaskNumber, maxTaskNumber-minTaskNumber+1))
    for i in range(data_2d.shape[0]):
        dataset_pd.insert(0, optimizer_name[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i], color=color_list[i], markersize=8)

    if(data_source=="EnergySaveRatio"):
        splot.set(xlabel="Task Number", ylabel="Energy Saving ratio (%)")
        # splot.set_ylim([0.95, 2.0])
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_" +title+ ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)
    elif(data_source=="Time"):
        splot.set(xlabel="Task Number", ylabel="Running time (seconds)")
        # splot.set_ylim([0.95, 2.0])
        splot.set(yscale="log")
        splot.set_ylim(1e-4, 1e3)
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_Time" + title + ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)
    elif(data_source=="RTA"):
        splot.set(xlabel="Task Number", ylabel="RTA calling times")
        # splot.set_ylim([0.95, 2.0])
        # splot.set(yscale="log")
        # splot.set_ylim(1e-4, 1e3)
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_Time" + title +"_"+data_source+ ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)
