import argparse
import os
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
    # parser.add_argument('--baseline', type=str, default="Zhao20",
    #                     help='0')
    parser.add_argument('--application', type=str, default="energy",
                        help='0')
    parser.add_argument('--taskSize', type=int, default=5,
                        help='N')
    parser.add_argument('--directory', type=str,
                        default='/home/zephyr/Programming/others/YechengRepo/',
                        help='O')
    parser.add_argument('--directoryWriteTo', type=str,
                        default='EnergySpeed',
                        help='O')

    args = parser.parse_args()

    # baseline = args.baseline
    application = args.application
    taskSize = args.taskSize
    YechengDirectory = args.directory

    targetFolder = args.directoryWriteTo

    if (application == "energy"):
        YechengDirectory = YechengDirectory + "Experiment/WCETEnergyOpt/TestCases/NSweep"
    elif (application == "control"):
        YechengDirectory = YechengDirectory + "Experiment/ControlPerformance/TestCases/NSweep"
    else:
        print("Unrecognized application type!")
        sys.exit()

    target_file_name = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/EnergySpeed/Time/N" + str(
        taskSize) + ".txt"

    directory_path = YechengDirectory + '/N' + str(taskSize)
    files = os.listdir(directory_path)
    files.sort()

    run_time_bfs = []
    run_time_gp = []

    index = 0
    for file in files:
        split_arr=file.split("_")
        if(len(split_arr)<3):
            continue
        if (split_arr[2][:3] == "BFS"):
            ff = open(directory_path + "/" + file, "r")
            lines = ff.readlines()
            run_time = float(lines[0].split(" ")[0])
            run_time_bfs.append(run_time)
            ff.close()
        elif (file.split("_")[2][:2] == "GP"):
            ff = open(directory_path + "/" + file, "r")
            lines = ff.readlines()
            run_time = float(lines[0].split(" ")[0])
            run_time_gp.append(run_time)
            ff.close()

    with open(target_file_name, "a") as f:
        f.write(str(sum(run_time_bfs) / len(run_time_bfs))+"\n")
        f.write(str(sum(run_time_gp) / len(run_time_gp)))
