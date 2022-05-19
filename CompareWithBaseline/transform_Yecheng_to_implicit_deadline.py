import argparse
import os

def txt2data(line):
    data = line.split()
    for i in range(len(data)):
        data[i] = int(data[i])
    return data


if __name__ == "__main__":
    # import arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--convertionNumber', type=int, default=20,
                        help='N')
    parser.add_argument('--directory', type=str,
                        default='/home/zephyr/Programming/others/YechengRepo/Experiment/WCETEnergyOpt/TestCases/NSweep',
                        help='O')
    parser.add_argument('--directoryWriteTo', type=str,
                        default='/home/zephyr/Programming/others/YechengRepo/Experiment/WCETEnergyOpt/TestCases/NSweep',
                        help='O')

    args = parser.parse_args()
    YechengDirectory = args.directory
    targetTaskNumber = args.convertionNumber
    targetDataSet = args.directoryWriteTo

    directory_path = YechengDirectory + '/N' + str(targetTaskNumber)
    files = os.listdir(directory_path)
    files.sort()


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

            target_file_name = targetDataSet  + '/N' + str(targetTaskNumber)+'/'+file
            # print(file[4:-4])
            # print(file)
            with open(target_file_name, "w+") as f:
                f.write(lines[0])
                f.write(lines[1])
                f.write(lines[1])
                f.write(lines[3])
                f.write(lines[4])

            f.close()
