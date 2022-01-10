import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--entry', type=str, default="batchTestMethod",
                    help='entry to modify')
parser.add_argument('--value', type=str, default="1",
                    help='value for the new entry')
parser.add_argument('--target', type=str, default="../sources/parameters.yaml",
                    help='target yaml file location')
args = parser.parse_args()
entry = args.entry
value = args.value
target = args.target

my_file=open(target,"r+")
lines=my_file.readlines()
for i, line in enumerate(lines):
    entry_curr = line.split(":")[0]
    if(entry_curr==entry):
        line_new=entry_curr+": "+value+"\n"
        lines[i]=line_new
        break

my_file.close()
my_file=open(target,"w")
my_file.writelines(lines)

