import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt



def read_data_2d(minTaskNumber, maxTaskNumber, methodsNum):
    data2d = []
    for optimizer_index in range(methodsNum):
        data = []
        for i in range(minTaskNumber, maxTaskNumber + 1):
            file_path = "EnergySaveRatio/N" + str(i) + ".txt"
            file = open(file_path, "r")
            lines = file.readlines()
            data.append(float(lines[optimizer_index ]))
            file.close()
        data2d.append(data)
    return data2d
