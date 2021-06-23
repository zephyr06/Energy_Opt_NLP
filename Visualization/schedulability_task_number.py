from util_visua import *

if __name__ == "__main__":
    x_label, y_label, type_runtime, title, data, time_data = preprocess()

    task_number_list = range(3, 10 + 1)

    if type_runtime == "schedulability":
        make_plot(data, task_number_list, "schedulability_" + title, x_label, y_label)
    elif type_runtime == "energy":
        plot_energy(data, task_number_list, "energy_" + title, x_label, y_label)
        # divide the time_data by the number of schedulable task sets
        data_path = "data_buffer_schedulability_" + title + ".txt"
        schedulability_data = Read_batch_analyze(data_path)
        time_data = time_data[:, 0] / schedulability_data[:, 1]
        time_data = np.expand_dims(time_data, 1)
        plot_energy(time_data, task_number_list, "Running_time_" + title, x_label, "Running time (seconds)")
