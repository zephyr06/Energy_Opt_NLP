import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
import argparse


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

def Read_all_result_files():
    # data_2d=np.empty((maxTaskNumber+1-minTaskNumber, taskSetNumber))
    data_2d={}
    for i in range(minTaskNumber, maxTaskNumber+1):
        file="EnergySaveRatio/N"+str(i)+".txt"
        data_1d=Read_txt_file_1d(file, lambda x: (x-1)*100)
        data_2d[i]=data_1d
    return data_2d

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
