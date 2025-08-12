import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# Get the home directory of the current user
home_directory = os.path.expanduser('~')

# Get the username from the home directory path
username = os.path.basename(home_directory)

# Read the CSV file into a Pandas DataFrame

data_file_path = f'/home/{username}/Floaty/ws/src/floaty_pkg/data/floaty_estimator/' 
data_file_base_name = "floaty_data"

with open(data_file_path+"iteration_num.txt", 'r+') as file:
    # Read the integer value from the file
    value = int(file.read())
    
file_path = data_file_path + data_file_base_name + "_" + str(value) + ".csv"
# file_path = f'/home/{username}/Floaty/ws/src/floaty_pkg/data/floaty_estimator/floaty_data_1280.csv' 
# file_path = f'/home/{username}/Floaty/ws/src/floaty_pkg/data/floaty_estimator/good_data/Good_video_1.csv' 
# file_path = f'/home/{username}/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Sys_Id_flying/New_setup/floaty_v3_good_flight_2.csv' 


data = pd.read_csv(file_path)

data_freq = 50

# Extract the 'Opt_roll' column data
opt_roll_data = data['roll']
opt_pitch_data = data['pitch']

optitrack_x_data = data['Optitrack_x']
optitrack_y_data = data['Optitrack_y']
optitrack_z_data = data['Optitrack_z']

est_x_data = data['est_x']
est_y_data = data['est_y']
est_z_data = data['est_z']

est_vx_data = data['est_vx']
est_vy_data = data['est_vy']
est_vz_data = data['est_vz']

optitrack_vx_data = np.zeros_like(optitrack_x_data)
optitrack_vy_data = np.zeros_like(optitrack_y_data)
optitrack_vz_data = np.zeros_like(optitrack_z_data)

optitrack_vx_data[0]=0
optitrack_vy_data[0]=0
optitrack_vz_data[0]=0

for i in range(len(optitrack_vx_data)-2):
    optitrack_vx_data[i+1]= (optitrack_x_data[i+2] - optitrack_x_data[i])*data_freq/2
    optitrack_vy_data[i+1]= (optitrack_y_data[i+2] - optitrack_y_data[i])*data_freq/2
    optitrack_vz_data[i+1]= (optitrack_z_data[i+2] - optitrack_z_data[i])*data_freq/2

optitrack_roll_data = data['roll']*180/3.14
optitrack_pitch_data = data['pitch']*180/3.14
optitrack_yaw_data = data['yaw']*180/3.14

est_roll_data = data['est_roll']*180/3.14
est_pitch_data = data['est_pitch']*180/3.14
est_yaw_data = data['est_yaw']*180/3.14


est_roll_rate_data = data['est_roll_rate']*180/3.14
est_pitch_rate_data = data['est_pitch_rate']*180/3.14
est_yaw_rate_data = data['est_yaw_rate']*180/3.14


gyro_x = data['gyro_x']*180/3.14
gyro_y = data['gyro_y']*180/3.14
gyro_z = data['gyro_z']*180/3.14

optitrack_roll_rate_data = np.zeros_like(optitrack_roll_data)
optitrack_pitch_rate_data = np.zeros_like(optitrack_pitch_data)
optitrack_yaw_rate_data = np.zeros_like(optitrack_yaw_data)

optitrack_roll_rate_data[0]=0
optitrack_pitch_rate_data[0]=0
optitrack_yaw_rate_data[0]=0

for i in range(len(optitrack_roll_rate_data)-2):
    optitrack_roll_rate_data[i+1]= (optitrack_roll_data[i+2] - optitrack_roll_data[i])*data_freq/2
    optitrack_pitch_rate_data[i+1]= (optitrack_pitch_data[i+2] - optitrack_pitch_data[i])*data_freq/2
    optitrack_yaw_rate_data[i+1]= (optitrack_yaw_data[i+2] - optitrack_yaw_data[i])*data_freq/2




control_f1 = data['control_f1']*180/3.14
control_f2 = data['control_f2']*180/3.14
control_f3 = data['control_f3']*180/3.14
control_f4 = data['control_f4']*180/3.14

command_m1 = data['command_m1']*180/3.14
command_m2 = data['command_m2']*180/3.14
command_m3 = data['command_m3']*180/3.14
command_m4 = data['command_m4']*180/3.14

est_f1 = data["est_f1"]*180/3.14
est_f2 = data["est_f2"]*180/3.14
est_f3 = data["est_f3"]*180/3.14
est_f4 = data["est_f4"]*180/3.14

optitrack_flap_angle = [val+0.05 for val in opt_roll_data]

# Create a sequential index for x-axis (assuming sequential data)
x_values = range(len(opt_roll_data))
# x_values = range(len(opt_roll_data)-2)

# Plot the sequential data of 'Opt_roll'
# plt.figure(figsize=(20, 8))
# plt.plot(x_values, opt_roll_data, marker='o', linestyle='-')
# plt.plot(x_values, opt_pitch_data, marker='o', linestyle='--')

# # -------------------  Position estimation -------------------
plt.figure(figsize=(20, 8))
plt.plot(x_values, optitrack_x_data, linestyle='-', color='red')
plt.plot(x_values, est_x_data, linestyle='--', color='red')

plt.plot(x_values, optitrack_y_data, linestyle='-', color='green')
plt.plot(x_values, est_y_data, linestyle='--', color='green')

plt.plot(x_values, optitrack_z_data, linestyle='-', color='blue')
plt.plot(x_values, est_z_data, linestyle='--', color='blue')
plt.xlabel('Index')
plt.ylabel('Position in meters')
plt.title("Floaty's position")
plt.grid(True)
plt.show()

# # -------------------  Velocity estimation -------------------
plt.figure(figsize=(20, 8))
plt.plot(x_values, optitrack_vx_data, linestyle='-', color='red')
plt.plot(x_values, est_vx_data, linestyle='--', color='red')

plt.plot(x_values, optitrack_vy_data, linestyle='-', color='green')
plt.plot(x_values, est_vy_data, linestyle='--', color='green')

plt.plot(x_values, optitrack_vz_data, linestyle='-', color='blue')
plt.plot(x_values, est_vz_data, linestyle='--', color='blue')
plt.xlabel('Index')
plt.ylabel('Velocity in meters/s')
plt.title("Floaty's Velocity")
plt.ylim((-2,2))
plt.grid(True)
plt.show()

# # -------------------  Orientation estimation -------------------
plt.figure(figsize=(20, 8))
plt.plot(x_values, optitrack_roll_data, linestyle='-', color='red')
plt.plot(x_values, est_roll_data, linestyle='--', color='red')

plt.plot(x_values, optitrack_pitch_data, linestyle='-', color='green')
plt.plot(x_values, est_pitch_data, linestyle='--', color='green')

plt.plot(x_values, optitrack_yaw_data, linestyle='-', color='blue')
plt.plot(x_values, est_yaw_data, linestyle='--', color='blue')
plt.xlabel('Index')
plt.ylabel('Orientation in degrees')
plt.title("Floaty's orientation")
plt.grid(True)
plt.show()

# # -------------------  Angular rates estimation -------------------
plt.figure(figsize=(20, 8))
plt.plot(x_values, gyro_x, linestyle='-', color='red')
plt.plot(x_values, est_roll_rate_data, linestyle='--', color='red')
plt.plot(x_values, optitrack_roll_rate_data, linestyle='-.', color='red')

plt.plot(x_values, gyro_y, linestyle='-', color='green')
plt.plot(x_values, est_pitch_rate_data, linestyle='--', color='green')
plt.plot(x_values, optitrack_pitch_rate_data, linestyle='-.', color='green')

plt.plot(x_values, gyro_z, linestyle='-', color='blue')
plt.plot(x_values, est_yaw_rate_data, linestyle='--', color='blue')
plt.plot(x_values, optitrack_yaw_rate_data, linestyle='-.', color='blue')
plt.xlabel('Index')
plt.ylabel('Angular rates in degrees')
plt.ylim((-500,500))
plt.title('Angular rates')
plt.grid(True)
plt.show()

# # -------------------  Flap estimation test -------------------
# plt.figure(figsize=(20, 8))
# plt.plot(x_values, control_f1[2:], linestyle='-', color='green')
# plt.plot(x_values, command_m1[2:], linestyle='-', color='green')
# plt.plot(x_values, optitrack_flap_angle[2:], linestyle='-', color='red')
# plt.plot(x_values, est_f1[2:], linestyle='--', color='red')

# plt.plot(x_values, control_f1, linestyle='-', color='red')
# plt.plot(x_values, optitrack_flap_angle, linestyle='-', color='red')

# plt.plot(x_values, control_f2, linestyle='-', color='green')
# plt.plot(x_values, est_f2, linestyle='--', color='green')

# plt.plot(x_values, control_f3, linestyle='-', color='blue')
# plt.plot(x_values, est_f3, linestyle='--', color='blue')
# plt.xlabel('Index')
# plt.ylabel('Estimated flap angles in degrees')
# plt.title('Estimated flap angles')
# plt.grid(True)
# plt.show()

# # -------------------  Flap command  -------------------

plt.figure(figsize=(20, 8))
plt.plot(x_values, command_m1, linestyle='--', color='red')
plt.plot(x_values, command_m2, linestyle='--', color='blue')
plt.plot(x_values, command_m3, linestyle='--', color='green')
plt.plot(x_values, command_m4, linestyle='--', color='black')
plt.xlabel('Index')
plt.ylabel('Flap angles in degrees')
plt.title('Control command measurement')
plt.grid(True)
plt.show()

# # -------------------  Yaw tracking -------------------

# Using the gyro_x to transfer the target_Yaw value
target_yaw_val = gyro_x
target_yaw_err = -1*(target_yaw_val-optitrack_yaw_data)

plt.figure(figsize=(20, 8))
plt.plot(x_values, target_yaw_val, linestyle='-', color='red', label='Target Yaw Value')
plt.plot(x_values, optitrack_yaw_data, linestyle='-', color='blue', label='Optitrack Yaw Data')
plt.plot(x_values, est_yaw_data, linestyle='--', color='blue', label='Estimated Yaw Data / Error')
plt.plot(x_values, target_yaw_err, linestyle='--', color='red', label='Yaw Error to Target')
plt.xlabel('Index')
plt.ylabel('Yaw in degrees')
plt.title("Yaw and Yaw error")
# Display the legend
plt.legend()

plt.grid(True)
plt.show()



# # -------------------  Flap command  -------------------

# Create a 3D plot
fig = plt.figure(figsize=(20, 8))
ax = fig.add_subplot(111, projection='3d')

data_start=0
data_end = 5000
data_end = len(optitrack_x_data)

num_points = data_end-data_start
# colors = np.arange(num_points) / num_points  # Normalize index to [0, 1]

sampling_freq = 50
end_time = int(num_points/sampling_freq) # To calculate the length of the data
colors = np.linspace(0, end_time, num_points)

# # ------------------
# # Plot the 3D path
# ax.plot(optitrack_x_data, optitrack_y_data, optitrack_z_data, marker='o', linestyle='-', color='b')

# # ------------------
# Plot the 3D path with color gradient

sc = ax.scatter(optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end], c=colors, cmap='viridis', marker='o', alpha=0.8, vmin=0, vmax=end_time)

# ax.scatter([0], [0], [0], color='red', marker='o', alpha=0.5, s=500)

# Add colorbar
cbar = fig.colorbar(sc)
cbar.set_label('Time [s]', fontsize=16)

# Set the same axis limits
limit_min = min(optitrack_x_data.min(), optitrack_y_data.min())
limit_max = max(optitrack_x_data.max(), optitrack_y_data.max())

# ax.set_xlim([limit_min, limit_max])
# ax.set_ylim([limit_min, limit_max])
ax.set_xlim([-0.33, 0.33])
ax.set_ylim([-0.33, 0.33])

# Set labels and title
ax.set_xlabel('X [m]', fontsize=14)
ax.set_ylabel('Y [m]', fontsize=14)
ax.set_zlabel('Z [m]', fontsize=14)
ax.set_title('3D Path Visualization', fontsize=16)

# Show the plot
plt.show()

# Create a figure with 3 subplots for each projection (XY, XZ, YZ)
fig, axs = plt.subplots(1, 3, figsize=(15, 5))
red_color = (214/255,39/255,40/255)

# XY projection (XZ plane)
axs[0].scatter(optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], c=colors, cmap='viridis', alpha=0.5)
axs[0].scatter([0], [0], color=red_color, marker='x', s=200, linewidths=4, alpha=0.85)
axs[0].scatter([-0.03], [0.08], color=red_color, marker='o', s=100, linewidths=3, alpha=0.85)
axs[0].set_xlabel('X [m]', fontsize=14)
axs[0].set_ylabel('Y [m]', fontsize=14)
axs[0].set_xlim([-0.2, 0.2])
axs[0].set_ylim([-0.2, 0.2])
axs[0].set_title('XY Plane', fontsize=16)

# XZ projection (YZ plane)
axs[1].scatter(optitrack_x_data[data_start:data_end], optitrack_z_data[data_start:data_end], c=colors, cmap='viridis', alpha=0.5)
axs[1].scatter([0], [0], color=red_color, marker='x', s=200, linewidths=4, alpha=0.85)
axs[1].scatter([-0.03], [-0.65], color=red_color, marker='o', s=100, linewidths=3, alpha=0.85)
axs[1].set_xlabel('X [m]', fontsize=14)
axs[1].set_ylabel('Z [m]', fontsize=14)
axs[1].set_xlim([-0.2, 0.2])
axs[1].set_ylim([-0.7, 0.32])
axs[1].set_title('XZ Plane', fontsize=16)

# YZ projection (YZ plane)
axs[2].scatter(optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end], c=colors, cmap='viridis', alpha=0.5)
axs[2].scatter([0], [0], color=red_color, marker='x', s=200, linewidths=4, alpha=0.85)
axs[2].scatter([+0.08], [-0.65], color=red_color, marker='o', s=100, linewidths=3, alpha=0.85)
axs[2].set_xlabel('Y [m]', fontsize=14)
axs[2].set_ylabel('Z [m]', fontsize=14)
axs[2].set_xlim([-0.2, 0.2])
axs[2].set_ylim([-0.7, 0.32])
axs[2].set_title('YZ Plane', fontsize=16)

# Adjust layout to avoid overlap
plt.tight_layout()

# Show plot
plt.show()



# # -------------------  Orientation estimation -------------------
import tikzplotlib
plt.figure(figsize=(20, 8))

data_start=1100
data_end = 2300
data_end = len(optitrack_x_data)
time_values = np.linspace(0,30,data_end-data_start)
plt.plot(time_values, optitrack_roll_data[data_start:data_end], linestyle='-', color='#d62728', label='Roll')
# plt.plot(x_values, est_roll_data, linestyle='--', color='red')

plt.plot(time_values, optitrack_pitch_data[data_start:data_end], linestyle='-', color='#2ca02c', label='Pitch')
# plt.plot(x_values, est_pitch_data, linestyle='--', color='green')

plt.plot(time_values, optitrack_yaw_data[data_start:data_end], linestyle='-', color='#1f77b4', label='Yaw')
# plt.plot(x_values, est_yaw_data, linestyle='--', color='blue')
plt.xlabel('Time [s]')
plt.ylabel('Error in orientation')
plt.title("Floaty's orientation")
plt.grid(True)

# Add legend
plt.legend(loc='lower right')

plt.show()

def tikzplotlib_fix_ncols(obj):
    """
    workaround for matplotlib 3.6 renamed legend's _ncol to _ncols, which breaks tikzplotlib
    """
    if hasattr(obj, "_ncols"):
        obj._ncol = obj._ncols
    for child in obj.get_children():
        tikzplotlib_fix_ncols(child)

tikzplotlib.save("floaty_orientation.tikz")

input("a")