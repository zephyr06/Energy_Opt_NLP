import argparse
import os
import sys


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
    parser.add_argument('--application', type=str, default="Control",
                        help='0')
    parser.add_argument('--taskSize', type=int, default=5,
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


    target_file_name_time = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" + application+"Performance" + "/Time/N" + str(
        taskSize) + ".txt"
    with open(target_file_name_time, "r") as f:
        lines=f.readlines()
        f.close()
    with open(target_file_name_time, "w") as f:
        for line in lines[:3]:
            f.write(line)
        f.close()

    target_file_name_energy = "/home/zephyr/Programming/Energy_Opt_NLP/CompareWithBaseline/" +  application+"Performance" + "/EnergySaveRatio/N" + str(
        taskSize) + ".txt"
    with open(target_file_name_energy, "r") as f:
        lines=f.readlines()
        f.close()
    with open(target_file_name_energy, "w") as f:
        for line in lines[:1000*3]:
            f.write(line)
        f.close()
