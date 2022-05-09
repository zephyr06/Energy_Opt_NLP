import argparse
import graphviz

def write_csv_to_graphviz(path, destination):
    file=open(path)
    lines=file.readlines()
    file_d=open(destination,'w')
    file_d.write('digraph G{')

    for line in lines:
        if(line[0]=='*'):
            prev=line[1:].split(',')[0]
            next = line[1:].split(',')[1]
            file_d.write("Node"+str(prev)+" -> "+"Node"+str(next))
    file_d.write('}')

def show_graphviz(path):
    file = open(path)
    lines = file.readlines()
    dot = graphviz.Digraph(comment='The Round Table')


    for line in lines:
        if(line[0]>='0' and line[0]<='9'):
            dot.node("Node"+line.split(',')[0])
        if (line[0] == '*'):
            prev = line[1:].split(',')[0]
            next = line[1:].split(',')[1]
            dot.edge("Node"+str(prev), "Node"+str(next))
    print(dot.source)
    dot.render('currGraph.gv', view=True)


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--n', type=int, default="14",
                        help='the number of test file to visualize, default 14 means test_n5_v14.csv')
    args = parser.parse_args()
    n = args.n
    path1="../TaskData/test_n5_v"+str(n)+".csv"
    path2="../TaskData/task_number/dag-set-000-syntheticJobs.csv"
    # path_destination="currGraph.gv"
    # write_csv_to_graphviz(path1,path_destination)
    show_graphviz(path2)

