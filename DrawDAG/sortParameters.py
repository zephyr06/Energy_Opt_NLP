import os
import shutil

def sortCustomize(line1, line2):
    elements1 = line1.split(" ")
    elements2 = line2.split(" ")
    # if(len(elements1)>2):
    #     return elements1[2] < elements2[2]
    # else:
    #     return elements1[0] < elements2[0]
    for i in range(min(len(elements1), len(elements2))):
        if(elements1[i]!=elements2[i]):
            return elements1[i] < elements2[i]
    return True


# this file sorts the parameters.yaml and Parameters.h file in source folder
if __name__=="__main__":
    sorted_fn = 'sorted_filename.h'
    file_list = ["../sources/parameters.yaml", "../sources/Parameters.h"]
    # sort Parameters.h
    # file = "../sources/Parameters.h"
    for file in file_list:
        with open(file, 'r') as ff:
            lines = ff.readlines()
            lines_no_sort=[]
            lines_sort=[]
            for line in lines:
                if(line[0]=="#" or line[0:2]=="cv" or line[:5]=="using" or line[0]=="%"):
                    lines_no_sort.append(line)
                elif(line=="\n"):
                    continue
                else:
                    lines_sort.append(line)
            # lines_sort=sorted(lines_sort, key=cmp_to_key(sortCustomize) , reverse=False)
            lines_sort = sorted(lines_sort, key=str.lower, reverse=False)


            with open(sorted_fn,'w') as destination:

                for line in lines_no_sort:
                    destination.write(line)
                destination.write("\n")
                destination.write("\n")
                for line in lines_sort:
                    destination.write(line)

            # os.rename(sorted_fn, "../sources/"+file.split('/')[-1])
            # os.replace(sorted_fn, "../sources/" + file.split('/')[-1])
            shutil.move(sorted_fn, "../sources/" + file.split('/')[-1])



