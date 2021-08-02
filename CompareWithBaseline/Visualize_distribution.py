import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from util_visua import Read_txt_file_1d

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
    data_2d=np.empty((16, 1000))
    for i in range(5,20+1):
        file="ResultFiles/N"+str(i)+".txt"
        data_1d=Read_txt_file_1d(file, lambda x: (x-1)*100)
        data_2d[i-5]=data_1d
    return data_2d

if __name__ == "__main__":
    data_2d=Read_all_result_files()
    # fig1, ax1 =plt.subplots()
    # ax1.boxplot(data_2d)
    # plt.show()
    dataset_pd = pd.DataFrame(data=data_2d,
                              index=range(5,21)).T
    ax = sns.boxplot( data=dataset_pd, orient="v")
    ax.set(xlabel="Task Number", ylabel="Relative Gap (%)")
    plt.savefig("Compare_energy" + ".pdf", format='pdf')
    plt.savefig("Compare_energy" + ".png", format='png')
    plt.show()
    a=1