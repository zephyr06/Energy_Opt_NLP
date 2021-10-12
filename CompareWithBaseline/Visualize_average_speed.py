import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from utils_visualize import *

if __name__ == "__main__":

    path = "time_task_number.txt"
    data_2d = Read_txt_file_2d(path, lambda x: x).T
    task_number_seq = range(minTaskNumber, maxTaskNumber + 1)
    df = pd.DataFrame(data=data_2d, index={"NLP", baseline}, columns=range(minTaskNumber, maxTaskNumber+1))

    splot = sns.lineplot(data=df.T, markers=True)
    splot.set(yscale="log")
    plt.grid(linestyle="--")
    splot.set(xlabel="Task Number", ylabel="Runt-Time (seconds)")
    splot.set_xlim(4, None)
    splot.set_ylim(1e-3, ylim)
    plt.savefig("Compare_run_time" + ".pdf", format='pdf')
    plt.savefig("Compare_run_time" + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
