import os
import argparse


def clear_dir_result(directory, Nmin, Nmax):
    for filename in os.listdir(directory):
        if filename[0] == "N" and filename[-4:] == ".txt" and Nmin <= int(filename[1:-4]) and int(
                filename[1:-4]) <= Nmax:
            file = open(directory + "/" + filename, "w")
            file.truncate()
            file.close()


parser = argparse.ArgumentParser()
parser.add_argument('--folder', type=str, default="Optimizer",
                    help='folder that stores results')
parser.add_argument('--Nmin', type=int, default="11",
                    help='folder that stores results to clear')
parser.add_argument('--Nmax', type=int, default="10000",
                    help='folder that stores results to clear')

args = parser.parse_args()
folder = args.folder
Nmin = args.Nmin
Nmax = args.Nmax

clear_dir_result(folder + "/EnergySaveRatio", Nmin, Nmax)
clear_dir_result(folder + "/Time", Nmin, Nmax)
clear_dir_result(folder + "/RTACalling", Nmin, Nmax)
