import argparse
import os


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
    parser.add_argument('--convertionNumber', type=int, default=10,
                        help='N')
    parser.add_argument('--directory', type=str,
                        default='/home/zephyr/Programming/others/YechengRepo/Experiment/WCETEnergyOpt/TestCases/NSweep',
                        help='O')
    parser.add_argument('--directoryWriteTo', type=str,
                        default='/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number',
                        help='O')

    args = parser.parse_args()
    YechengDirectory = args.directory
    targetTaskNumber = args.convertionNumber
    targetDataSet = args.directoryWriteTo

    clear_dataset(targetDataSet)

    directory_path = YechengDirectory + '/N' + str(targetTaskNumber)
    files = os.listdir(directory_path)
    files.sort()

    index = 0
    for file in files:
        if (file[:4] == "Case" and file[-4:] == ".txt" and file[-5] != 't'):
            ff = open(directory_path + "/" + file, "r")
            lines = ff.readlines()
            period_txt = lines[1]
            deadline_txt = lines[2]
            computation_time_txt = lines[3]

            period = txt2data(period_txt)[1:]
            deadline = txt2data(deadline_txt)[1:]
            computation_time = txt2data(computation_time_txt)[1:]

            target_file_name = targetDataSet + "/" + "periodic-set-" + file[4:-4] + "-syntheticJobs.csv"
            # print(file[4:-4])
            # print(file)
            with open(target_file_name, "w+") as f:
                f.write('JobID,Offset,Period,Overhead,ExecutionTime,DeadLine,processorId (From Yecheng)\n')

                for i in range(len(period)):
                    f.write(
                        str(i) + ',' + str(0) + ',' + str(period[i]) + ',' + str(0) + ',' + str(
                            computation_time[i]) + ',' + str(deadline[i]) + ',' + str(0) + '\n')
            f.close()
            index = index + 1
