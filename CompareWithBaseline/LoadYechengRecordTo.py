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
    parser.add_argument('--application', type=str, default="Energy",
                        help='0')
    parser.add_argument('--taskSize', type=int, default=11,
                        help='N')
    parser.add_argument('--directory', type=str,
                        default='/home/zephyr/Programming/others/YechengRepo/',
                        help='O')
    # parser.add_argument('--directoryWriteTo', type=str,
    #                     default='EnergySpeed',
    #                     help='O')

    args = parser.parse_args()

    # baseline = args.baseline
    application = args.application
    taskSize = args.taskSize
    YechengDirectory = args.directory

    # targetFolder = args.directoryWriteTo

    if (application == "Energy"):
        YechengDirectory = YechengDirectory + "Experiment/WCETEnergyOpt/TestCases/NSweep"
    elif (application == "Control"):
        YechengDirectory = YechengDirectory + "Experiment/ControlPerformance/TestCases/NSweep"
    else:
        print("Unrecognized application type!")
        sys.exit()



    directory_path = YechengDirectory + '/N' + str(taskSize)
    files = os.listdir(directory_path)
    files.sort()

    run_time_bfs = []
    run_time_gp = []
    energy_bfs = []
    energy_gp = []

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
            energy =  float(lines[0].split(" ")[1])
            energy_bfs.append(energy)
            ff.close()
        elif (file.split("_")[2][:2] == "GP"):
            ff = open(directory_path + "/" + file, "r")
            lines = ff.readlines()
            run_time = float(lines[0].split(" ")[0])
            run_time_gp.append(run_time)
            energy = float(lines[0].split(" ")[1])
            energy_gp.append(energy)
            ff.close()

    target_file_name_time = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" + application+"Performance" + "/Time/N" + str(
        taskSize) + ".txt"
    with open(target_file_name_time, "a") as f:
        f.write(str(sum(run_time_bfs) / len(run_time_bfs))+"\n")
        if(len(run_time_gp)!=0):
            f.write(str(sum(run_time_gp) / len(run_time_gp))+"\n")

    target_file_name_energy = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +  application+"Performance" + "/EnergySaveRatio/N" + str(
        taskSize) + ".txt"
    with open(target_file_name_energy, "a") as f:
        for i in range(len(energy_bfs)):
            f.write(str(energy_bfs[i])+"\n")
        # if (application == "Energy"):
        for i in range(len(energy_gp)):
            f.write(str(energy_gp[i])+"\n")
