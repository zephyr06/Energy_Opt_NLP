import argparse
import random
import numpy as np
import os

def clear_dataset(path_dataset):
    files_periodic = []
    for f in os.listdir(path_dataset):
        if (f.split('-')[0] == "periodic"):
            task_path = os.path.join(path_dataset, f)
            os.remove(task_path)

def txt2data(line):
    data=line.split()
    for i in range(len(data)):
        data[i]=int(data[i])
    return data

if __name__ == "__main__":
    # import arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--convertionNumber', type=int, default=5,
                        help='N')
    parser.add_argument('--directory', type=str, default='/home/zephyr/Programming/Energy_Opt_NLP/TaskData',
                        help='O')
    parser.add_argument('--directoryWriteTo', type=str,
                        default='/home/zephyr/Programming/Energy_Opt_NLP/TaskData/',
                        help='O')

    args = parser.parse_args()
    directory = args.directory
    targetDataSet=args.directoryWriteTo

    files = os.listdir(directory)
    files.sort()

    index = 0
    for file in files:
        if(file[:4]=="test" and file[-4:]==".csv" ):
            ff=open(directory+"/"+file,"r")
            lines=ff.readlines()
            lines[0]=lines[0][:-1]+",processorId\n"
            for i in range(1, len(lines)):
                lines[i]=lines[i][:-1]+",0\n"
            ff = open(directory + "/" + file, "w")
            ff.writelines(lines)
            ff.close()
            a=1




