import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("/home/gelmkaiel/Floaty/ws/src/ros_test_pkg/src/motors_accuracy.csv")
np_arr = df.to_numpy()
np_arr[:,0] = (np_arr[:,0]-4000)*15/500

# max_val = max(np_arr[:,1])
# min_val = min(np_arr[:,1])

max_val = np.average(np_arr[115:165,1])
min_val = np.average(np_arr[200:265,1])

np_arr[:,1] = ((np_arr[:,1]-min_val)/(max_val-min_val))*45

# np_arr = np_arr[200:400,:]

length = np_arr.shape[0]
freq = 100
x = [i/freq for i in range(length)]
plt.plot(x, np_arr[:,0], label = "Motor command")
plt.scatter(x, np_arr[:,1], s= 2, c="r", label = "Optitrack")
plt.legend(loc='lower right')
plt.show()
print("done!")