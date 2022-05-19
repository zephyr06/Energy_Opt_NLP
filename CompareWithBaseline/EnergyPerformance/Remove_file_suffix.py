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
    parser.add_argument('--N', type=int, default=40,
                        help='N')
    parser.add_argument('--directory', type=str,
                        default='/home/zephyr/Programming/others/YechengRepo/Experiment/WCETEnergyOpt/TestCases/NSweep',
                        help='O')
    parser.add_argument('--directoryWriteTo', type=str,
                        default='/home/zephyr/Programming/Energy_Opt_NLP/TaskData/task_number',
                        help='O')

    args = parser.parse_args()
    YechengDirectory = args.directory
    targetTaskNumber = args.N
    targetDataSet = args.directoryWriteTo

    clear_dataset(targetDataSet)

    directory_path = YechengDirectory + '/N' + str(targetTaskNumber)
    files = os.listdir(directory_path)
    files.sort()

    index = 0
    for file in files:
        if (file[:4] == "Case" and file[-4:] == ".txt"):
            if(len(file.split("."))==3):
                file_new = file.split(("."))[0]+"."+file.split(("."))[1]
                os.rename(os.path.join(directory_path, file), os.path.join(directory_path, file_new));

