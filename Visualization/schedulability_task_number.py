import argparse
import matplotlib.pyplot as plt
import numpy as np


def Read_batch_analyze(path):
    data = []
    file = open(path, 'r')
    lines = file.readlines()
    for line in lines:
        data.append(float(line[:-1]))
    return data

def plot_energy(data, parameter_list, title, x_label, y_label):
    fig, ax = plt.subplots()
    # the shape of data is (1, N)
    plt.plot(parameter_list, data, color='red', linestyle='dashed', marker='_',
             markerfacecolor='g', markersize=10)

    plt.xlabel(x_label)
    plt.ylabel(y_label)

    ax.grid(linestyle='--')
    plt.savefig(title + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
    plt.close()
    np.savetxt(title+".txt", data)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--x_label', type=str, default="x",
                        help='x label in your graph')
    parser.add_argument('--y_label', type=str, default="y",
                        help='y label in your graph')
    parser.add_argument('--type', type=str, default="energy",
                        help='schedulability or energy')
    parser.add_argument('--title', type=str, default="task_number",  # task_number  utilization  constrained
                        help='title of the saved figure')
    parser.add_argument('--task_number', type=int, default="x",
                        help='task_number')                        
    args = parser.parse_args()
    x_label = args.x_label
    y_label = args.y_label
    type_runtime = args.type
    title = args.title
    time_file = "time_" + title + ".txt"
    task_number = args.task_number

    data_path = "data_buffer_energy_" + title + ".txt"
    energy_saving_ratio = Read_batch_analyze(data_path)

    time_path = "time_task_number.txt"
    time_list = Read_batch_analyze(time_path)

    task_number_list = range(3, task_number + 1)

    plot_energy(energy_saving_ratio, task_number_list, "energy_" + title, x_label, y_label)

    plot_energy(time_list, task_number_list, "Running_time_" + title, x_label, "Running time (seconds)")
