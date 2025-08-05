import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


# -------------------------------------------------------------------
# -------------------------------------------------------------------


pth_to_data = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/lower"


# X = np.array(data.iloc[:,[0,1]])
number_of_points_per_speed = 20
speed_increment = 5
start_speed = 0
end_speed = 75

forces = []
force_std = []
speeds = np.array(range(start_speed,end_speed+1,speed_increment))
for speed in speeds:
    print(speed)
    data = pd.read_csv(pth_to_data+"/"+str(speed)+".csv", header=None)
    data = -1*np.array(data.iloc[:,2])
    force = np.mean(data)
    forces.append(force)
    force_std.append(np.std(data))

plt.rcParams.update({'font.size': 15})

fig, axs  = plt.subplots(1,3, figsize=(14, 10))

x_pol = np.array(range(start_speed,end_speed))
low_pol = np.polyfit(speeds,forces,2)
y_pol = np.polyval(low_pol,x_pol)

# labels = ["measurements", "fit"]
labels = ["fit", "measurements"]    # in the case of using errorbars

axs[0].errorbar(speeds, forces,  yerr=force_std, fmt='bo', capsize=5)
# axs[0].plot(speeds, forces, 'bo--')
# axs[0].scatter(x, y)
axs[0].plot(x_pol, y_pol, 'r')
axs[0].set_xlabel("motor speed [%]", fontsize=17)
axs[0].set_ylabel("measured force [N]", fontsize=17)
axs[0].set_title("clock-wise motor", fontsize=17)
axs[0].legend(labels)

# -------------------------------------------------------------------
# -------------------------------------------------------------------

pth_to_data = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/upper"



forces = []
force_std = []
speeds = np.array(range(start_speed,end_speed+1,speed_increment))
for speed in speeds:
    print(speed)
    data = pd.read_csv(pth_to_data+"/"+str(speed)+".csv", header=None)
    data = -1*np.array(data.iloc[:,2])
    force = np.mean(data)
    forces.append(force)
    force_std.append(np.std(data))
    


high_pol = np.polyfit(speeds,forces,2)
y_pol = np.polyval(high_pol,x_pol)

axs[1].errorbar(speeds, forces,  yerr=force_std, fmt='bo', capsize=5)
# axs[1].plot(x, y, 'bo--')
# axs[1].scatter(x, y)
axs[1].plot(x_pol, y_pol, 'r')
axs[1].set_xlabel("motor speed [%]", fontsize=17)
axs[1].set_ylabel("measured force [N]", fontsize=17)
axs[1].set_title("counter clock-wise motor", fontsize=17)
axs[1].legend(labels)


# -------------------------------------------------------------------
# -------------------------------------------------------------------

pth_to_data = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/center"



forces = []
force_std = []
speeds = np.array(range(start_speed,end_speed+1,speed_increment))
for speed in speeds:
    print(speed)
    data = pd.read_csv(pth_to_data+"/"+str(speed)+".csv", header=None)
    data = -1*np.array(data.iloc[:,2])
    force = np.mean(data)
    forces.append(force)
    force_std.append(np.std(data))
    


center_pol = np.polyfit(speeds,forces,2)
y_pol = np.polyval(center_pol,x_pol)

axs[2].errorbar(speeds, forces,  yerr=force_std, fmt='bo', capsize=5)
# axs[1].plot(x, y, 'bo--')
# axs[1].scatter(x, y)
axs[2].plot(x_pol, y_pol, 'r')
axs[2].set_xlabel("motor speed [%]", fontsize=17)
axs[2].set_ylabel("measured force [N]", fontsize=17)
axs[2].set_title("counter clock-wise motor", fontsize=17)
axs[2].legend(labels)

plt.show()
print("Lower polyniomial: ", low_pol)
print("Higher polyniomial: ", high_pol)
print("Center polyniomial: ", center_pol)

print("done")



import tikzplotlib
tikzplotlib.save("/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/fit_poly.tex")
