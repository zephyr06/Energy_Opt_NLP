
from utils_visualize import *




if __name__ == "__main__":


    data_2d=Read_all_result_files(taskSetNumber)
    # fig1, ax1 =plt.subplots()
    # ax1.boxplot(data_2d)
    # plt.show()

    # index = range(minTaskNumber, maxTaskNumber + 1)
    dataset_pd = pd.DataFrame(data=data_2d)
    ax = sns.boxplot( data=dataset_pd, orient="v", fliersize=1, saturation=0.75, whis=1.5)
    ax.set(xlabel="Task Number", ylabel="Relative Gap (%)")
    plt.savefig("Compare_energy" + ".pdf", format='pdf')
    plt.savefig("Compare_energy" + ".png", format='png')
    plt.show(block=False)
    plt.pause(3)
    a=1
