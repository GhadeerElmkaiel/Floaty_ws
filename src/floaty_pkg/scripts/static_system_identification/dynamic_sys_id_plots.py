import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

current_file_dir = os.path.dirname(os.path.realpath(__file__))
data_file_dir = current_file_dir +"/../../data/dynamic_system_identification/current_experiment/"
data_file_name = "/dynamic_sys_id_1.csv"


# all_data = np.array([[0,0,0,0,0,0]])
# for folder_dir in folders_dir:

df = pd.read_csv(data_file_dir + data_file_name, header=None)
np_arr = df.to_numpy()

window_size = 1000
num_of_windows = int(np_arr.shape[0]/window_size)
x_axis = [j/10 for j in range(1,window_size//2)]

fig, axis = plt.subplots(8, 1, figsize=(20, 10))
data_id = 5
for i in range(num_of_windows):
    data = np_arr[i*window_size:(i+1)*window_size, :]

    signal = data[:,data_id]
    fft_res = np.fft.fft(signal)/len(signal)
    # fft_res = fft[range(int(len(signal)/2))]


    amps = np.abs(fft_res)
    angs = [np.angle(f) if (abs(amps[i])>0.0001) else 0 for (i,f) in enumerate(fft_res)]
    axis[i].plot(x_axis, amps[1:window_size//2])

    min_amp = min(amps[1:window_size//2])
    max_amp = max(amps[1:window_size//2])
    axis[i].plot([10,10], [min_amp,max_amp])
plt.show()

# smoothed_res = np.zeros([6,full_length])
# for i in range(full_length):
#     for j in range(6):
#         res = np.average(all_data[i*shift:i*shift+window_size,j])
#         smoothed_res[j,i] = res


# if full_length>1:
#     plt.plot(x_axis, smoothed_res[0,:])
#     plt.plot(x_axis, smoothed_res[1,:])
#     plt.plot(x_axis, smoothed_res[2,:])
#     plt.plot(x_axis, smoothed_res[3,:])
#     plt.plot(x_axis, smoothed_res[4,:])
#     plt.plot(x_axis, smoothed_res[5,:])
# else:
#     plt.scatter(x_axis, smoothed_res[0,0])
#     plt.scatter(x_axis, smoothed_res[1,0])
#     plt.scatter(x_axis, smoothed_res[2,0])
#     plt.scatter(x_axis, smoothed_res[3,0])
#     plt.scatter(x_axis, smoothed_res[4,0])
#     plt.scatter(x_axis, smoothed_res[5,0])

# plt.legend(["x force", "y force", "z force", "x torque", "y torque", "z torque"])
# print(np.average(all_data[:,2]))

# # plt.plot(x_axis, smoothed_res[2,:])
# # plt.legend(["z force"])


# plt.show()

print("Done!")
