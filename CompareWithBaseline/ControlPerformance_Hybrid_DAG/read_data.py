import sys

sys.path.append("/")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def Read_txt_file_1d_box(path, func, data_readin_size):
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


def ChangeDataBaseline(data_1d, func, task_set_size, baseline_data_index, main_data_index):
    """the input is 2000*1=[a;b], the output is 1000*1=[a/b], for the visualization for ControlPerformance_Hybrid"""
    data_res = [0] * task_set_size
    for i in range(task_set_size):
        data_res[i] = func(data_1d[i + task_set_size * main_data_index] / data_1d[i + task_set_size * baseline_data_index])
        if (data_res[i] > 0.1):
            print(i, data_1d[i + task_set_size * main_data_index] , data_1d[i + task_set_size * baseline_data_index], data_res[i])
            aa = 1
    return data_res


def read_data_2d_box(minTaskNumber, maxTaskNumber, task_set_size,  data_source, baseline_data_index, main_data_index):
    data_2d = {}
    for i in range(minTaskNumber, maxTaskNumber + 1):
        file = data_source + "/" + "N" + str(i) + ".txt"
        data_1d = Read_txt_file_1d_box(file, lambda x: x, task_set_size * (main_data_index + 1))
        data_1d = ChangeDataBaseline(data_1d, lambda x: (x - 1) * 100.0, task_set_size, baseline_data_index, main_data_index)
        data_2d[i] = data_1d
    return data_2d