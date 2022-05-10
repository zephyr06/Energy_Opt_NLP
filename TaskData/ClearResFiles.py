import argparse
import os
import sys

def clear_dataset(path_dataset):
    files_periodic = []
    for f in os.listdir(path_dataset):
        if ("_Res.txt" in f):
            task_path = os.path.join(path_dataset, f)
            os.remove(task_path)


if __name__ == "__main__":
    # import arguments
    parser = argparse.ArgumentParser()
    # parser.add_argument('--baseline', type=str, default="Zhao20",
    #                     help='0')
    parser.add_argument('--pathDataset', type=str, default="TaskData/",
                        help='0')
    parser.add_argument('--taskSize', type=int, default=-1,
                        help='-1 means clearing all the task sizes')
    parser.add_argument('--root', type=str,
                        default='/home/zephyr/Programming/Energy_Opt_NLP/',
                        help='O')
    # parser.add_argument('--directoryWriteTo', type=str,
    #                     default='EnergySpeed',
    #                     help='O')

    args = parser.parse_args()
    root = args.root
    pathDataset=args.pathDataset
    taskSize=args.taskSize

    if(taskSize>0):
        target_dir = root +pathDataset+ "N/"+str(taskSize)
        clear_dataset(target_dir)
    else:
        taskData=root+pathDataset
        for f in os.listdir(taskData):
            if(f[0]=='N' and (not ("." in f))):
                clear_dataset(root +pathDataset+f+"/")

