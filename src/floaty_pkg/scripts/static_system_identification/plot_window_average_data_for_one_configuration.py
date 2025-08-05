import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

current_file_dir = os.path.dirname(os.path.realpath(__file__))
data_file_dire = current_file_dir +"/../../data/static_system_identification/current_experiment/"
data_file_name = "data_with_angles_60_60_60_60.csv"

df = pd.read_csv(data_file_dire + data_file_name, header=None)
np_arr = df.to_numpy()

window_size = 100
shift = 1
full_length = int((np_arr.shape[0]-window_size)/shift) + 1
x_axis = [i*shift for i in range(full_length)]

smoothed_res = np.zeros([6,full_length])
for i in range(full_length):
    for j in range(6):
        res = np.average(np_arr[i*shift:i*shift+window_size,j])
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
print(np.average(np_arr[:,2]))

# plt.plot(x_axis, smoothed_res[2,:])
# plt.legend(["z force"])


plt.show()

print("Done!")
