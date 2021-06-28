import argparse

import matplotlib.pyplot as plt
import numpy as np


def Read_batch_analyze(path="data_buffer.txt"):
    data = np.empty((0, 6))
    file = open(path, 'r')
    lines = file.readlines()
    for line in lines:
        parameter = line[1:-2].split(',')
        # total number of task sets, total schedulable task sets
        data = np.vstack([data, np.array([int(parameter[0]), int(parameter[1]),
                                          # schedulable by AB, abort
                                          int(parameter[2]), int(parameter[3]),
                                          # preempt, wait
                                          int(parameter[4]), int(parameter[5])])])

    return data


def Read_batch_analyze_energy(path):
    data = np.empty((0, 1))
    file = open(path, 'r')
    lines = file.readlines()
    for line in lines:
        if line[0] == 'n':
            data = np.vstack([data, 1.0])
            continue
        parameter = line[1:-2].split(',')
        # total number of task sets, total schedulable task sets
        data = np.vstack([data, np.array([double(parameter[0])])])

    return data


def Read_time_file(path):
    file = open(path, 'r')
    lines = file.readlines()

    data = np.empty((0, 1))
    for line in lines:
        if line[0:4] == "user":
            time_str = line[1:-2].split('\t')[1]
            minute = np.int(time_str.split('m')[0])
            seconds = np.double(time_str.split('m')[1][:-1])
            data = np.vstack([data, np.array([minute * 60 + seconds])])

    return data


def make_plot(data, parameter_list, title, x_label, y_label):
    fig, ax = plt.subplots()

    # entry list
    Wait = data[:, 5]
    plt.plot(parameter_list, Wait, label="Wait", color='purple', linestyle='dashed', marker='*',
             markerfacecolor='purple', markersize=10)

    schedulable = data[:, 1]
    plt.plot(parameter_list, schedulable, label="Schedulable", color='green', linestyle='dashed', marker='x',
             markerfacecolor='green', markersize=10)

    AB = data[:, 2]
    plt.plot(parameter_list, AB, label="WAP", color='red', linestyle='dashed', marker='|',
             markerfacecolor='red', markersize=10)

    Abort = data[:, 3]
    plt.plot(parameter_list, Abort, label="Abort", color='pink', linestyle='dashed', marker='_',
             markerfacecolor='pink', markersize=7)

    Preempt = data[:, 4]
    plt.plot(parameter_list, Preempt, label="Preempt", color='blue', linestyle='dashed', marker='.',
             markerfacecolor='blue', markersize=6)

    plt.xlabel(x_label)
    plt.ylabel(y_label)

    plt.legend()
    ax.grid(linestyle='--')
    plt.savefig(title + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
    plt.close()
    np.savetxt(title + ".txt", data)


def plot_energy(data, parameter_list, title, x_label, y_label):
    fig, ax = plt.subplots()
    # the shape of data is (1, N)
    plt.plot(parameter_list, data[:, 0], color='red', linestyle='dashed', marker='|',
             markerfacecolor='red', markersize=10)

    plt.xlabel(x_label)
    plt.ylabel(y_label)

    ax.grid(linestyle='--')
    plt.savefig(title + ".pdf", format='pdf')
    plt.show(block=False)
    plt.pause(3)
    plt.close()
    np.savetxt(title+".txt", data)


def preprocess():
    parser = argparse.ArgumentParser()
    parser.add_argument('--x_label', type=str, default="x",
                        help='x label in your graph')
    parser.add_argument('--y_label', type=str, default="y",
                        help='y label in your graph')
    parser.add_argument('--type', type=str, default="energy",
                        help='schedulability or energy')
    parser.add_argument('--title', type=str, default="task_number",  # task_number  utilization  constrained
                        help='title of the saved figure')
    args = parser.parse_args()
    x_label = args.x_label
    y_label = args.y_label
    type_runtime = args.type
    title = args.title
    time_file = "time_" + title + ".txt"

    time_data = Read_time_file(time_file)

    data_path = ""
    if type_runtime == "schedulability":
        data_path = "data_buffer_schedulability_" + title + ".txt"
        data = Read_batch_analyze(data_path)
    elif type_runtime == "energy":
        data_path = "data_buffer_energy_" + title + ".txt"
        data = Read_batch_analyze_energy(data_path)
    else:
        raise Exception("Type error in visualization file")

    return x_label, y_label, type_runtime, title, data, time_data
