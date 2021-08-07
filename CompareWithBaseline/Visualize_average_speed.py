import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns


def Read_txt_file_2d(path, func, delimiter=','):
    """ read txt files, and return a 2D list, each element contains MORE THAN one number"""
    file = open(path, 'r')
    lines = file.readlines()
    res = []
    for line in lines:
        list1 = line.split(delimiter)
        n=len(list1)
        for i in range(n):
            list1[i]=func(float(list1[i]))
        res.append(list1)
    return np.array(res)

if __name__ == "__main__":
    path="time_task_number.txt"
    data_2d = Read_txt_file_2d(path, lambda x:x).T
    task_number_seq=range(5,20+1)
    df=pd.DataFrame(data=data_2d, index={"NLP", "MUA"}, columns=range(5,21))

    # splot=sns.lineplot(data=df0, x="task_number", y="run-time", legend="MUA")
    # splot=sns.lineplot(data=df1, x="task_number", y="run-time", legend="NLP")
    # f, ax=plt.subplots(1,1)
    # ax.plot_date(df0.task_number, df0["run-time"], color="blue", label="NLP", linestyle="-")
    # ax.plot_date(df1.task_number, df1["run-time"], color="blue", label="MUA-incremental", linestyle="-")
    # sns.pointplot(ax=ax, data=df1, color='blue')
    # ax.legend()
    # plt.gcf().autofmt_xdate()
    splot=sns.lineplot(data=df.T,markers=True)
    splot.set(yscale="log")
    plt.grid(linestyle="--")
    splot.set(xlabel="Task Number", ylabel="Runt-Time (seconds)")
    splot.set_xlim(4, None)
    splot.set_ylim(1e-3, 1e2)
    plt.savefig("Compare_run_time" + ".pdf", format='pdf')
    plt.savefig("Compare_run_time" + ".png", format='png')
    plt.show()