import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

current_file_dir = os.path.dirname(os.path.realpath(__file__))
data_file_dir = current_file_dir +"/../../data/static_system_identification/current_experiment/"
data_file_name = "/data_with_angles_60_60_60_60.csv"

# folders_dir = [x[0] for x in os.walk(data_file_dir)  if x[0]!= data_file_dir and not "Figures" in x[0] and not "test" in x[0]]
folder_names = ["no_comb_1", "comb_small_1", "comb_mid_1", "comb_large_1"]
folders_dir = [data_file_dir+"/"+folder for folder in folder_names]


all_data = np.array([[0,0,0,0,0,0]])
for folder_dir in folders_dir:

    df = pd.read_csv(folder_dir + data_file_name, header=None)
    np_arr = df.to_numpy()
    all_data = np.concatenate((all_data,np_arr))

window_size = 1
shift = 1
full_length = int((all_data.shape[0]-window_size)/shift) + 1
x_axis = [i*shift for i in range(full_length)]

smoothed_res = np.zeros([6,full_length])
for i in range(full_length):
    for j in range(6):
        res = np.average(all_data[i*shift:i*shift+window_size,j])
        smoothed_res[j,i] = res


if full_length>1:
    plt.plot(x_axis, smoothed_res[0,:])
    plt.plot(x_axis, smoothed_res[1,:])
    plt.plot(x_axis, smoothed_res[2,:])
    plt.plot(x_axis, smoothed_res[3,:])
    plt.plot(x_axis, smoothed_res[4,:])
    plt.plot(x_axis, smoothed_res[5,:])
else:
    plt.scatter(x_axis, smoothed_res[0,0])
    plt.scatter(x_axis, smoothed_res[1,0])
    plt.scatter(x_axis, smoothed_res[2,0])
    plt.scatter(x_axis, smoothed_res[3,0])
    plt.scatter(x_axis, smoothed_res[4,0])
    plt.scatter(x_axis, smoothed_res[5,0])

plt.legend(["x force", "y force", "z force", "x torque", "y torque", "z torque"])
print(np.average(all_data[:,2]))

# plt.plot(x_axis, smoothed_res[2,:])
# plt.legend(["z force"])


plt.show()

print("Done!")
