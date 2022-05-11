import sys
import os
sys.path.append("../")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def to_string_my(x):
    if(x<10):
        return "00"+ str(x)
    elif (x<100):
        return "0"+str(x)
    elif (x<1000):
        return str(x)
    else:
        raise Exception("Index is too big to support")

def read_data_2d_energy(minUtil, maxUtil, root, source_task_index, type):
    data2d = []
    folder = root + "/TaskData" + "/N" + str(source_task_index) + "/"
    file_num_per_index = int(len(os.listdir(folder))/3/(maxUtil-minUtil+1))
    for util in range(minUtil, maxUtil + 1):
        data = []
        ave_elim = 0
        ave_not_elim = 0
        for i in range(file_num_per_index):
            path = folder+"periodic-dag-Narsi-set-"+str(source_task_index)+"-"+to_string_my((util-1) * file_num_per_index + i) +"-syntheticJobs.yaml_elim_Res.txt"
            assert os.path.exists(path)
            file = open(path, "r")
            lines = file.readlines()
            if(type=="EnergySaveRatio"):
                ave_elim += (float(lines[0]))
            elif(type=="Time"):
                ave_elim+= (float(lines[1]))
            else:
                raise Exception("type is not recognized")
            file.close()

            path = folder+"periodic-dag-Narsi-set-"+str(source_task_index)+"-"+to_string_my((util-1) * file_num_per_index + i) +"-syntheticJobs.yaml_not_elim_Res.txt"
            file = open(path, "r")
            lines = file.readlines()
            if(type=="EnergySaveRatio"):
                ave_not_elim += (float(lines[0]))
            elif(type=="Time"):
                ave_not_elim += (float(lines[1]))
            else:
                raise Exception("type is not recognized")
            file.close()
        data.append(ave_elim / file_num_per_index)
        data.append(ave_not_elim / file_num_per_index)
        data2d.append(data)

    return data2d


parser = argparse.ArgumentParser()
parser.add_argument('--minUtil', type=int, default=1,
                    help='Umin')
parser.add_argument('--maxUtil', type=int, default=7,
                    help='Umax')
parser.add_argument('--title', type=str, default="DAGPerformanceUtil",
            help='tilte in produced figure')
parser.add_argument('--source_task_index', type=int, default="3",
            help='source_task_index')
parser.add_argument('--plot_type', type=str, default="EnergySaveRatio",
            help='EnergySaveRatio or Time')
parser.add_argument('--root', type=str, default="/home/zephyr/Programming/Energy_Opt_NLP",
            help='root path')


args = parser.parse_args()
minUtil = args.minUtil
maxUtil = args.maxUtil
title=args.title
source_task_index = args.source_task_index
plot_type = args.plot_type
root=args.root

if __name__ == "__main__":
    data_2d = read_data_2d_energy(minUtil, maxUtil, root, source_task_index, plot_type)
    data_2d=np.array(data_2d).transpose()
    dataset_pd = pd.DataFrame()
    optimizer_name=["NLP_Elimination", "NLP_Raw",  "Zhao20", "MILP"]
    marker_list = ["o", "v", "s", "D"] #
    color_list = ["#0084DB", "limegreen", "r", "gold"] #
    dataset_pd.insert(0, "index", np.linspace(minUtil*0.1, maxUtil*0.1, (maxUtil-minUtil+1)))
    for i in {0,1}:
        dataset_pd.insert(0, optimizer_name[i], data_2d[i])
        splot = sns.lineplot(data=dataset_pd, x="index", y=optimizer_name[i], marker=marker_list[i], color=color_list[i], markersize=8)

    # MILP
    # plt.plot(np.linspace(minTaskNumber, min(15, maxTaskNumber), min(15, maxTaskNumber)-minTaskNumber+1), data_2d[3][:min(15, maxTaskNumber)-minTaskNumber+1], marker=marker_list[3], color=color_list[3], markersize=8)

    if(plot_type=="EnergySaveRatio"):
        splot.set(xlabel="Task Number", ylabel="Energy Saving ratio (%)")
        splot.set_ylim([0.25, 1])
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_" +title+ ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)
    elif(plot_type=="Time"):
        splot.set(xlabel="Task Number", ylabel="Running time (seconds)")
        # splot.set_ylim([0.95, 2.0])
        splot.set(yscale="log")
        splot.set_ylim(1e-4, 1e3)
        plt.legend(labels=optimizer_name)
        plt.grid(linestyle="--")
        plt.savefig("Compare_Time" + title + ".pdf", format='pdf')
        plt.show(block=False)
        plt.pause(3)