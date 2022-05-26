import argparse
import copy
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
    parser.add_argument('--application', type=str, default="Energy",
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



    directory_path = YechengDirectory + '/N' + str(taskSize)
    files = os.listdir(directory_path)
    files.sort()

    run_time_bfs = []
    run_time_gp = []
    energy_bfs = []
    energy_gp = []

    index = 0
    for file in files:
        split_arr=file.split(".")
        if(split_arr[1]=="m" and split_arr[0][:4]=="Case"):
            ff = open(directory_path + "/" + file, "r")
            lines = ff.readlines()
            # line_T=lines[1]
            # # line_D=lines[2]
            # line_D_implicit = copy.deepcopy(line_T)
            # lines[2]=line_D_implicit.replace("T", "D")
            # lines[2]=line_D_implicit
            lines.insert(1, "tic;\n")
            for index, line in enumerate(lines):
                if(line=="runTime = sol.solvertime + sol.yalmiptime;\n"):
                    lines[index]="runTime = toc;\n"
                if(line=="vals = [sol.solvertime; value(obj); size(periods, 1); periods];\n"):
                    lines[index]="vals = [runTime; value(obj); size(periods, 1); periods];\n"
                if(line=="vals = [sol.solvertime; value(obj); size(wcets, 1); wcets];\n"):
                    lines[index]="vals = [toc; value(obj); size(wcets, 1); wcets];\n"

            ff.close()
            ff = open(directory_path + "/" + file, "w")
            for line in lines:
                ff.write(line)
            ff.close()
