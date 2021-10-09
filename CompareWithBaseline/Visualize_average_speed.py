import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from utils_visualize import *

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--minTaskNumber', type=int, default=5,
                        help='Nmin')
    parser.add_argument('--maxTaskNumber', type=int, default=6,
                        help='Nmax')
    args = parser.parse_args()
    minTaskNumber = args.minTaskNumber
    maxTaskNumber = args.maxTaskNumber

    path = "time_task_number.txt"
    data_2d = Read_txt_file_2d(path, lambda x: x).T
    task_number_seq = range(minTaskNumber, maxTaskNumber + 1)
    df = pd.DataFrame(data=data_2d, index={"NLP", "SA"}, columns=range(minTaskNumber, maxTaskNumber+1))

    splot = sns.lineplot(data=df.T, markers=True)
    splot.set(yscale="log")
    plt.grid(linestyle="--")
    splot.set(xlabel="Task Number", ylabel="Runt-Time (seconds)")
    splot.set_xlim(4, None)
    splot.set_ylim(1e-3, None)
    plt.savefig("Compare_run_time" + ".pdf", format='pdf')
    plt.savefig("Compare_run_time" + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
