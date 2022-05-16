import os
import argparse

def clear_dir_result(directory):
    for filename in os.listdir(directory):
        if filename[0] == "N" and filename[-4:] == ".txt":
            file = open(directory + "/" + filename, "w")
            file.truncate()
            file.close()

parser = argparse.ArgumentParser()
parser.add_argument('--folder', type=str, default="optimizer",
                    help='folder that stores results')
args = parser.parse_args()
folder = args.folder
clear_dir_result(folder+"/EnergySaveRatio")
clear_dir_result(folder+"/Time")
clear_dir_result(folder+"/RTACalling")
