import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

def Read_txt_file_1d(path, func):
    """ read txt files, and return a list, each element contains one number"""
    file=open(path, 'r')
    lines=file.readlines()
    res=[]
    for line in lines:
        number=float(line)
        res.append(func(number))
    return np.array(res)

