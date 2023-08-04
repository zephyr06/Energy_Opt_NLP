import sys

sys.path.append("/")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


def Read_txt_file_1d_box(path, func, data_readin_size=1000):
    """ read txt files, and return a list, each element contains one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = []
    count = 0
    for line in lines:
        number = float(line)
        res.append(func(number))
        count = count + 1
        if (count > data_readin_size):
            print("The input file provides too much data!!!!!")
            break

    return np.array(res)


def ChangeDataBaseline(data_1d, func):
    """the input is 2000*1=[a;b], the output is 1000*1=[a/b], for the visualization for ControlPerformance_Hybrid"""
    data_res=data_1d[:1000].copy()
    for i in range(1000):
        data_res[i]=func(data_1d[i]/data_1d[i+1000])
        if(data_res[i]>0.1):
            print(i, data_1d[i],data_1d[i+1000],data_1d[i]/data_1d[i+1000])
            aa=1
    return data_res

def read_data_2d_box(minTaskNumber, maxTaskNumber, exp_folder, data_source):
    data_2d = {}
    for i in range(minTaskNumber, maxTaskNumber + 1):
        file = exp_folder + "/" + data_source + "/" + "N" + str(i) + ".txt"
        if(exp_folder=="ControlPerformance_Hybrid"):
            data_1d = Read_txt_file_1d_box(file, lambda x: x,2000)
            data_1d = ChangeDataBaseline(data_1d,lambda x: (x - 1) * 100.0)
        else:
            data_1d = Read_txt_file_1d_box(file, lambda x: (x - 1) * 100.0, 1000)
        data_2d[i] = data_1d
    return data_2d


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=5,
                    help='Nmax')
parser.add_argument('--data_source', type=str, default="EnergySaveRatio",
                    help='data source folder')
parser.add_argument('--exp_folder', type=str, default="ControlPerformance_Hybrid",
                    help='exp folder with data')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
data_source = args.data_source
exp_folder = args.exp_folder

if __name__ == "__main__":
    data_box_plot = read_data_2d_box(minTaskNumber, maxTaskNumber, exp_folder, data_source)
    dataset_pd = pd.DataFrame(dict([(k, pd.Series(v)) for k, v in data_box_plot.items()]))
    ax = sns.boxplot(data=dataset_pd, orient="v", fliersize=1, saturation=0.75, whis=1.5)
    x = np.linspace(-0.5, 15.5, 16)
    y = np.zeros((16))
    ax = sns.lineplot(x=x, y=y, linestyle="dashed", color='darkgray')

    ax.set(xlabel="Task Number", ylabel="Relative Gap (%)")
    plt.savefig(exp_folder + "/Compare_control_box_" + exp_folder+".pdf", format='pdf')
    # plt.savefig("Compare_control_box" +  "Zhao20" + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
