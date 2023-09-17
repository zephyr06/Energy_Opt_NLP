import sys

sys.path.append("/")
import argparse
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

from read_data import *


parser = argparse.ArgumentParser()
parser.add_argument('--minTaskNumber', type=int, default=3,
                    help='Nmin')
parser.add_argument('--maxTaskNumber', type=int, default=3,
                    help='Nmax')
parser.add_argument('--task_set_size', type=int, default=100,
                    help='task_set_size')
parser.add_argument('--data_source', type=str, default="EnergySaveRatio",
                    help='data source folder')
parser.add_argument('--baseline_data_index', type=str, default="0",
                    help='the baseline data for drawing the box plot; by default, it would be the first 1000 data')
parser.add_argument('--main_data_index', type=str, default="1",
                    help='the method for drawing the box plot; by default, it would be the second 1000 data')

args = parser.parse_args()
minTaskNumber = args.minTaskNumber
maxTaskNumber = args.maxTaskNumber
task_set_size = args.task_set_size
data_source = args.data_source
baseline_data_index = int(args.baseline_data_index)
main_data_index = int(args.main_data_index)

if __name__ == "__main__":
    data_box_plot = read_data_2d_box(minTaskNumber, maxTaskNumber, task_set_size,  data_source, baseline_data_index,
                                     main_data_index)
    dataset_pd = pd.DataFrame(dict([(k, pd.Series(v)) for k, v in data_box_plot.items()]))
    ax = sns.boxplot(data=dataset_pd, orient="v", fliersize=1, saturation=0.75, whis=1.5)
    line_len=18
    x = np.linspace(-0.5, line_len-0.5, line_len)
    y = np.zeros((line_len))
    plt.ylim(-70, 60)
    ax = sns.lineplot(x=x, y=y, linestyle="dashed", color='darkgray')

    ax.set(xlabel="Task Number", ylabel="Relative Gap (%)")
    plt.savefig( "Compare_control_box_" +str(main_data_index)+ ".pdf", format='pdf')
    # plt.savefig("Compare_control_box" +  "Zhao20" + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
