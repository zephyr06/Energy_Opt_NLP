import argparse
import os
import numpy as np
import sys


def clear_dataset(path_dataset):
    files_periodic = []
    for f in os.listdir(path_dataset):
        if (f.split('-')[0] == "periodic"):
            task_path = os.path.join(path_dataset, f)
            os.remove(task_path)


def txt2data(line):
    data = line.split()
    for i in range(len(data)):
        data[i] = int(data[i])
    return data


if __name__ == "__main__":
    # import arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--taskSize', type=int, default=10,
                        help='N')
    parser.add_argument('--directory', type=str,
                        default='/home/zephyr/Programming/others/YechengRepo/',
                        help='O')

    args = parser.parse_args()

    taskSize = args.taskSize
    YechengDirectory = args.directory


    root_path = "/home/zephyr/Programming/Energy_Opt_NLP/"
    directory_path = root_path + 'TaskData/N' + str(taskSize)
    files = os.listdir(directory_path)
    files.sort()

    # order: nlp_elim_approx, nlp_elim_exact, nlp_raw, sa
    # index: 0, 1, 2, 3
    run_time = np.zeros((4,105))
    energy = np.zeros((4,105))

    index = 0
    for file in files:
        split_arr=file.split("_")
        if(len(split_arr)<3):
            continue

        def get_index(file):
            # print(file)
            return int(file.split('-')[5])
        def read_result(file):
            ff = open(directory_path + "/" + file, "r")
            lines = ff.readlines()
            energy_read = float(lines[0].split(" ")[0])
            run_time_read = float(lines[1].split(" ")[0])
            ff.close()
            return energy_read, run_time_read
        if ("elim" in split_arr and "approx" in split_arr and "Res.txt" in split_arr):
            energy_read, run_time_read = read_result(file)
            index = get_index(file)
            run_time[0][index] = run_time_read
            energy[0][index] = energy_read
        elif ("elim" in split_arr and "exact" in split_arr and "Res.txt" in split_arr):
            energy_read, run_time_read = read_result(file)
            index = get_index(file)
            run_time[1][index] = run_time_read
            energy[1][index] = energy_read
        elif ("elim" in split_arr and "not" in split_arr and "Res.txt" in split_arr):
            energy_read, run_time_read = read_result(file)
            index = get_index(file)
            run_time[2][index] = run_time_read
            energy[2][index] = energy_read
        elif ("SA" in split_arr and "Res.txt" in split_arr):
            energy_read, run_time_read = read_result(file)
            index = get_index(file)
            run_time[3][index] = run_time_read
            energy[3][index] = energy_read



    target_file_name_time = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" + "DAGPerformance" + "/Time/N" + str(
        taskSize) + ".txt"
    with open(target_file_name_time, "w") as f:
        for i in range(4):
            f.write(str(np.average(run_time[i]))+"\n")

    target_file_name_energy = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" + "DAGPerformance" + "/EnergySaveRatio/N" + str(
        taskSize) + ".txt"
    with open(target_file_name_energy, "w") as f:
        for i in range(4):
            f.write(str(np.average(energy[i])) + "\n")
