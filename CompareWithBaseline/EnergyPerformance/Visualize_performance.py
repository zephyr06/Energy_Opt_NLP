import sys

sys.path.append("../")
import argparse
import os
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt


MIGP_MAX_TASK_NUMBER = 20

def read_data_2d_energy(minTaskNumber, maxTaskNumber):
    """ read data from a continuous, inclusive range minTaskNumber to maxTaskNumber"""
    def extract_ave(method_index):
        ave = 0
        for i in range(method_index * 1000, method_index * 1000 + 1000):
            ave += float(lines[i])
        return ave / 1000.0

    data2d = {}
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        file_path = "EnergySaveRatio" + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NLP, with elimination
        data.append(extract_ave(0))

        # NLP, with elimination, exact Jacobian
        data.append(extract_ave(1))

        # NLP, without elimination
        data.append(extract_ave(2))

        # MUA
        data.append(1.0)

        # MILP, maximum number is 15
        if (task_number < MIGP_MAX_TASK_NUMBER+1):
            ave = 0
            for i in range(1000):
                ave += float(lines[4000 + i]) / float(lines[3000 + i])
            data.append(ave / 1000)
        else:
            data.append(np.nan)

        data2d[task_number]=data
        file.close()
    return data2d

def read_data_2d_energy_from_set(task_number_set=[30, 40, 50, 60, 70, 80]):
    data2d = {}
    for task_number in task_number_set:
        file_path = "EnergySaveRatio" + "/N" + str(task_number) + ".txt"
        file = open(file_path, "r")
        lines = file.readlines()
        data = []

        # NLP, with elimination
        data.append(float(lines[0]))

        # NLP, with elimination, exact Jacobian
        data.append(float(lines[1]))

        # NLP, without elimination
        data.append(float(lines[2]))

        file.close()
        data2d[task_number]=data
    return data2d

def appendNan(seq, len_exp):
    while len(seq) < len_exp:
        seq.append(np.nan)
    return seq


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=5,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=30,
                    help='Nmax')
parser.add_argument('--title', type=str, default="EnergyPerformance",
                    help='tilte in produced figure')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
title = args.title

if __name__ == "__main__":
    data_2d_first_part = read_data_2d_energy(minTaskNumber, maxTaskNumber)
    # data_2d_first_part = np.array(data_2d_first_part).transpose()
    # data_2d_first_part = (data_2d_first_part - 0) * 100

    large_task_number_set = [ 40, 50, 60, 70, 80]
    data_2d_second_part = read_data_2d_energy_from_set(large_task_number_set)


    optimizer_name = ["NORTH", "NMBO", "IPM", "Zhao20", "MIGP"]
    marker_list = ["o", "v", "^", "s", "D"]  #
    color_list = ["#0084DB", "r", "y", "limegreen", "purple"]

    df = pd.DataFrame()
    data_dict={}
    for task_number in range(minTaskNumber, maxTaskNumber + 1):
        data_dict[task_number] = {}
        for i in range(len(optimizer_name)):
            if len(data_2d_first_part[task_number]) > i:
                data_dict[task_number][optimizer_name[i]] = data_2d_first_part[task_number][i]
                df[task_number] = appendNan(data_2d_first_part[task_number], len(optimizer_name))
        
    
    for task_number in large_task_number_set:
        data_dict[task_number] = {}
        for i in range(len(optimizer_name)):
            if len(data_2d_second_part[task_number]) > i:
                data_dict[task_number][optimizer_name[i]] = data_2d_second_part[task_number][i]
                # df[task_number] = data_2d_second_part[task_number]
                df[task_number] = appendNan(data_2d_second_part[task_number], len(optimizer_name))
    df.index = optimizer_name
    # df = pd.DataFrame(data_dict)
    print(df)
    # Normalize against NORTH
    shape_data = df.shape
    for i in range(shape_data[0]-1, -1, -1):
        print(i)
        for j in range(0, shape_data[1]):
            df.iloc[i,j] = (df.iloc[i,j]/df.iloc[0,j]-1)*100.0
        # df.iloc[0, j]=1.0
    print(df)


    df_transposed = df.transpose()

    # Plot the transposed DataFrame
    # df_transposed.plot(kind='line', marker='o', figsize=(10, 6), colormap = color_list)
    for i in range(len(optimizer_name)):
        x_values = df_transposed.index
        x_values = [str(x) for x in x_values]
        plt.plot(x_values, df_transposed[optimizer_name[i]], marker=marker_list[i], color=color_list[i],
                 markersize=8, label=optimizer_name[i])
    # plt.xlabel('Columns')
    # plt.ylabel('Values')
    # "40","60",
    plt.xticks([0, 5, 10, 15, 20, 25,  26,  28, 30], ["5", "10", "15", "20", "25", "30 ",  " 40",  "60", "80"])
    font_size = 15
    plt.xlabel("Task Number", fontsize=font_size)
    plt.ylabel("Relative gap with Zhao20 (%)", fontsize=font_size)
    plt.legend()
    plt.grid(linestyle="--")
    plt.ylim(-5, 180)
    plt.savefig("Compare_" + title + "_" + "EnergySaveRatio" + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
