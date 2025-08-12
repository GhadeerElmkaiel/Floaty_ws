import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from itertools import product
import os

# Get the home directory of the current user
home_directory = os.path.expanduser('~')

# Get the username from the home directory path
username = os.path.basename(home_directory)

# Get the home directory of the current user
home_directory = os.path.expanduser('~')

# Get the username from the home directory path
username = os.path.basename(home_directory)

# -------------------------------------------------------------------
# -------------------------------------------------------------------


# Get the home directory of the current user
home_directory = os.path.expanduser('~')

# Get the username from the home directory path
username = os.path.basename(home_directory)


data = pd.read_csv(f"/home/{username}/Floaty/ws/src/ros_test_pkg/src/data/lower_motor_thrust.csv", header=None)

# X = np.array(data.iloc[:,[0,1]])
data = -1*np.array(data.iloc[:,2])
number_of_points_per_speed = 20
speed_increment = 5
start_speed = 0
end_speed = 80

x = np.array(range(start_speed,end_speed+1,speed_increment))
y = np.array([])
j=0
bias = 0
for _ in x:
    temp_data = []
    for i in range(number_of_points_per_speed):
        temp_data.append(data[j])
        j = j+1
    if j==number_of_points_per_speed:
        bias = np.mean(np.array(temp_data))
    y =  np.append(y, np.mean(np.array(temp_data))-bias)

plt.rcParams.update({'font.size': 15})

fig, axs  = plt.subplots(1,2, figsize=(14, 10))

x_pol = np.array(range(start_speed,end_speed))
low_pol = np.polyfit(x,y,2)
y_pol = np.polyval(low_pol,x_pol)

labels = ["measurements", "fit"]

axs[0].plot(x, y, 'bo--')
# axs[0].scatter(x, y)
axs[0].plot(x_pol, y_pol, 'r')
axs[0].set_xlabel("motor speed [%]", fontsize=17)
axs[0].set_ylabel("measured force [N]", fontsize=17)
axs[0].set_title("clock-wise motor", fontsize=17)
axs[0].legend(labels)

# -------------------------------------------------------------------
# -------------------------------------------------------------------


data = pd.read_csv(f"/home/{username}/Floaty/ws/src/ros_test_pkg/src/data/higher_motor_thrust.csv", header=None)

# X = np.array(data.iloc[:,[0,1]])
data = -1*np.array(data.iloc[:,2])
number_of_points_per_speed = 20
speed_increment = 5
start_speed = 0
end_speed = 80

y = np.array([])
bias = 0
j=0
for _ in x:
    temp_data = []
    for i in range(number_of_points_per_speed):
        temp_data.append(data[j])
        j = j+1
    if j==number_of_points_per_speed:
        bias = np.mean(np.array(temp_data))
    y =  np.append(y, np.mean(np.array(temp_data))-bias)



high_pol = np.polyfit(x,y,2)
y_pol = np.polyval(high_pol,x_pol)

axs[1].plot(x, y, 'bo--')
# axs[1].scatter(x, y)
axs[1].plot(x_pol, y_pol, 'r')
axs[1].set_xlabel("motor speed [%]", fontsize=17)
axs[1].set_ylabel("measured force [N]", fontsize=17)
axs[1].set_title("counter clock-wise motor", fontsize=17)
axs[1].legend(labels)


import tikzplotlib
tikzplotlib.save(f"/home/{username}/Floaty/ws/src/ros_test_pkg/src/test.tex")

plt.show()
print("done")

