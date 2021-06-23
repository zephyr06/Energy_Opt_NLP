import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_number', type=int, default="5",
                        help='task number')
    args = parser.parse_args()
    task_number = args.task_number
    path = "/home/zephyr/Programming/Energy_Opt_NLP/sources/Parameters.h"
    file = open(path,'r')
    lines = file.readlines()
    line_task_number=lines[12]
    lines[12] = line_task_number.split("=")[0]+"= "+str(task_number)+";\n"
    file.close()

    file = open(path, 'w')
    file.writelines(lines)
    file.close()
    # print(lines)
    a=1
