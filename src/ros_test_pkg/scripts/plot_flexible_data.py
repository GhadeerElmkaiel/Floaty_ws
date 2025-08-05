import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import tikzplotlib

# Read the CSV file into a Pandas DataFrame

data_file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/' 
data_file_base_name = "floaty_data"

with open(data_file_path+"iteration_num.txt", 'r+') as file:
    # Read the integer value from the file
    value = int(file.read())
    
file_path = data_file_path + data_file_base_name + "_" + str(value) + ".csv"
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/floaty_data_1650.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/good_data/Good_video_1.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Sys_Id_flying/New_setup/floaty_v3_good_flight_2.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Z/Exp_1_Z_HAng_22_5.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Y/Exp_2_Y_25s_HAng_22_5.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/Square/Exp_3_Yaw_square_Slow_motion_25s_q_01_HAng_22_5.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/Exp_1_Yaw_35s_q_015_HAng_22_5.csv'     # 0.15 Hz Yaw sine wave
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/Exp_2_Yaw_20s_q_010_HAng_22_5.csv'     # 0.10 Hz Yaw sine wave
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/Exp_3_Yaw_20s_q_005_HAng_22_5.csv'     # 0.05 Hz Yaw sine wave
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/Exp_4_Yaw_20s_q_002_HAng_22_5.csv'     # 0.02 Hz Yaw sine wave
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Hover/Hover_35s_Z_115_HAng_22_5.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Robustness_airflow_fan/exp_1_2_3.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Robustness_airflow_fan/exp_5.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Sys_Id_flying/Science_Robotics_data/floaty_v4_ctrl_1_sys_id_1.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Sys_Id_flying/Science_Robotics_data/floaty_v4_ctrl_2_sys_id_1.csv' 

# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Sys_Id_flying/V4/ctrl_1_test_3.csv' 
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Sys_Id_flying/V4/floaty_v4_ctrl_2_sys_id_1.csv' 
## ===== Science robotics review =====
# ============= Y =============
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Y/Exp_2_Y_35s_HAng_22_5.csv' 

# ============= Z =============
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Z/Exp_4_Z_70s_HAng_22_5.csv' 

# ============= Yaw-0.15 =============
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.15_Hz/Exp_4_Yaw_60s_HAng_22_5.csv' 

# ============= Yaw-0.10 =============
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.10_Hz/Exp_4_Yaw_60s_HAng_22_5.csv' 

# ============= Yaw-0.05 =============
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.05_Hz/Exp_5_Yaw_60s_HAng_22_5.csv' 

# ============= Yaw-0.02 =============
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.02_Hz/Exp_1_Yaw_60s_HAng_22_5.csv' 

# ============= Yaw-square =============
# file_path = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/step/Exp_1_Yaw_HAng_22_5.csv' 




data = pd.read_csv(file_path)

data_freq = 50
rad2degs = 180/3.14

experiments_fig_size = (20,5)
label_font_size = 15
title_font_size = 20
ticks_font_size = 12
legend_font_size = 18

optitrack_x_data = data['Optitrack_x']
optitrack_y_data = data['Optitrack_y']
optitrack_z_data = data['Optitrack_z']

est_x_data = data['est_x']
est_y_data = data['est_y']
est_z_data = data['est_z']

target_x_data = data['target_x']
target_y_data = data['target_y']
target_z_data = data['target_z']

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

optitrack_roll_data = data['Optitrack_roll']*rad2degs
optitrack_pitch_data = data['Optitrack_pitch']*rad2degs
optitrack_yaw_data = data['Optitrack_yaw']*rad2degs

target_roll_data = data['target_roll']*rad2degs
target_pitch_data = data['target_pitch']*rad2degs
target_yaw_data = data['target_yaw']*rad2degs

est_roll_data = data['est_roll']*rad2degs
est_pitch_data = data['est_pitch']*rad2degs
est_yaw_data = data['est_yaw']*rad2degs


est_roll_rate_data = data['est_roll_rate']*rad2degs
est_pitch_rate_data = data['est_pitch_rate']*rad2degs
est_yaw_rate_data = data['est_yaw_rate']*rad2degs


gyro_x = data['gyro_x']*rad2degs
gyro_y = data['gyro_y']*rad2degs
gyro_z = data['gyro_z']*rad2degs


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



command_m1 = data['command_f1']*rad2degs
command_m2 = data['command_f2']*rad2degs
command_m3 = data['command_f3']*rad2degs
command_m4 = data['command_f4']*rad2degs

est_f1 = data["est_f1"]*rad2degs
est_f2 = data["est_f2"]*rad2degs
est_f3 = data["est_f3"]*rad2degs
est_f4 = data["est_f4"]*rad2degs


# Z and Yaw tracking
if "control_f1" in data.keys():
    control_f1 = data['control_f1']*rad2degs
    control_f2 = data['control_f2']*rad2degs
    control_f3 = data['control_f3']*rad2degs
    control_f4 = data['control_f4']*rad2degs

    compound_control_1 = np.zeros_like(control_f1)
    compound_control_2 = np.zeros_like(control_f1)
    compound_control_3 = np.zeros_like(control_f1)
    compound_control_4 = np.zeros_like(control_f1)

    compound_control_1 = (- control_f1 + control_f2 + control_f3 - control_f4)/4
    compound_control_2 = (- control_f1 - control_f2 + control_f3 + control_f4)/4
    compound_control_3 = (- control_f1 + control_f2 - control_f3 + control_f4)/4
    compound_control_4 = (- control_f1 - control_f2 - control_f3 - control_f4)/4


# Create a sequential index for x-axis (assuming sequential data)
x_values = range(len(optitrack_x_data))
# x_values = range(len(opt_roll_data)-2)

# Plot the sequential data of 'Opt_roll'
# plt.figure(figsize=(20, 8))
# plt.plot(x_values, opt_roll_data, marker='o', linestyle='-')
# plt.plot(x_values, opt_pitch_data, marker='o', linestyle='--')

# # -------------------  Position  -------------------
plt.figure(figsize=(20, 8))
plt.plot(x_values, optitrack_x_data, linestyle='-', color='red', label="x position")
plt.plot(x_values, target_x_data, linestyle='--', color='red', label="Target x")

plt.plot(x_values, optitrack_y_data, linestyle='-', color='green', label="y position")
plt.plot(x_values, target_y_data, linestyle='--', color='green', label="Target y")

plt.plot(x_values, optitrack_z_data, linestyle='-', color='blue', label="z position")
plt.plot(x_values, target_z_data, linestyle='--', color='blue', label="Target z")
plt.xlabel('Index')
plt.ylabel('Position in meters')
plt.title("Floaty's position")
plt.grid(True)
plt.legend()
plt.show()

# # -------------------  Velocity  -------------------
# plt.figure(figsize=(20, 8))
# plt.plot(x_values, optitrack_vx_data, linestyle='-', color='red')
# plt.plot(x_values, est_vx_data, linestyle='--', color='red')

# plt.plot(x_values, optitrack_vy_data, linestyle='-', color='green')
# plt.plot(x_values, est_vy_data, linestyle='--', color='green')

# plt.plot(x_values, optitrack_vz_data, linestyle='-', color='blue')
# plt.plot(x_values, est_vz_data, linestyle='--', color='blue')
# plt.xlabel('Index')
# plt.ylabel('Velocity in meters/s')
# plt.title("Floaty's Velocity")
# plt.ylim((-2,2))
# plt.grid(True)
# plt.show()

# # -------------------  Orientation  -------------------
plt.figure(figsize=(20, 8))
plt.plot(x_values, optitrack_roll_data, linestyle='-', color='red')
plt.plot(x_values, target_roll_data, linestyle='--', color='red')

plt.plot(x_values, optitrack_pitch_data, linestyle='-', color='green')
plt.plot(x_values, target_pitch_data, linestyle='--', color='green')

plt.plot(x_values, optitrack_yaw_data, linestyle='-', color='blue')
plt.plot(x_values, target_yaw_data, linestyle='--', color='blue')
plt.xlabel('Index')
plt.ylabel('Orientation in degrees')
plt.title("Floaty's orientation")
plt.grid(True)
plt.show()


# # -------------------  Angular rage estimation -------------------
# plt.figure(figsize=(20, 8))
# plt.plot(x_values, est_roll_rate_data, linestyle='-', color='red')

# plt.plot(x_values, est_pitch_rate_data, linestyle='-', color='green')

# plt.plot(x_values, est_yaw_rate_data, linestyle='-', color='blue')
# plt.xlabel('Index')
# plt.ylabel('Orientation in degrees')
# plt.title("Floaty's orientation")
# plt.grid(True)
# plt.show()


# # ------------------- Box plots -------------------
# # Labels for each data series
# labels = ['X error', 'Y error', 'Z error', 'Roll error', 'Pitch error', 'Yaw error']

# data_start=1500
# data_end=3100
# data_series = [optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end]-target_z_data[data_start:data_end], optitrack_roll_data[data_start:data_end]/rad2degs, optitrack_pitch_data[data_start:data_end]/rad2degs, optitrack_yaw_data[data_start:data_end]/rad2degs]

# data_series_pos = [optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end]-target_z_data[data_start:data_end]]
# data_series_orientation = [optitrack_roll_data[data_start:data_end]/rad2degs, optitrack_pitch_data[data_start:data_end]/rad2degs, optitrack_yaw_data[data_start:data_end]/rad2degs]
# # Create the box plot
# plt.figure(figsize=(8, 5))
# plt.boxplot(data_series, patch_artist=True, showfliers=False, boxprops=dict(facecolor='skyblue'))

# # Add X-axis labels
# plt.xticks(ticks=[1, 2, 3, 4, 5, 6], labels=labels)

# # Add title and Y-axis label
# plt.title('Box Plot for Multiple Data Series')
# plt.ylabel('Values')


# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Hover/"
# plt.savefig(path_to_figs+'Box_plots_compined.pdf', format='pdf')

# plt.show()

# # Create the box plot
# plt.figure(figsize=(8, 5))
# plt.boxplot(data_series_pos, patch_artist=True, showfliers=False, boxprops=dict(facecolor='skyblue'))

# # Add X-axis labels
# plt.xticks(ticks=[1, 2, 3], labels=labels[0:3])

# plt.ylabel('m')


# plt.savefig(path_to_figs+'Box_plots_pos.pdf', format='pdf')

# plt.show()

# # Create the box plot
# plt.figure(figsize=(8, 5))
# plt.boxplot(data_series_orientation, patch_artist=True, showfliers=False, boxprops=dict(facecolor='skyblue'))

# # Add X-axis labels
# plt.xticks(ticks=[1, 2, 3], labels=labels[3:6])

# plt.ylabel('rad')


# plt.savefig(path_to_figs+'Box_plots_orientation.pdf', format='pdf')

# plt.show()



# # -------------------  Position plot for Hover -------------------
# data_start=1500
# data_end=3100
# plt.figure(figsize=(8, 3))
# time_axes=[i/50 for i in range(data_end-data_start)]
# plt.plot(time_axes, optitrack_x_data[data_start:data_end], linestyle='-', color='red', label="x position")
# plt.plot(time_axes, target_x_data[data_start:data_end], linestyle='--', color='red', label="target x")

# plt.plot(time_axes, optitrack_y_data[data_start:data_end], linestyle='-', color='green', label="y position")
# plt.plot(time_axes, target_y_data[data_start:data_end], linestyle='--', color='green', label="target y")

# plt.plot(time_axes, optitrack_z_data[data_start:data_end], linestyle='-', color='blue', label="z position")
# plt.plot(time_axes, target_z_data[data_start:data_end], linestyle='--', color='blue', label="target z")
# plt.xlabel('Time')
# plt.ylabel('Position in meters')
# plt.title("Floaty's position")
# plt.grid(True)
# plt.legend()
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Hover/"

# plt.savefig(path_to_figs+'hover_position.pdf', format='pdf')

# plt.show()




# data_start=1500
# data_end=3100
# plt.figure(figsize=(8, 3))
# time_axes=[i/50 for i in range(data_end-data_start)]
# plt.plot(time_axes, optitrack_x_data[data_start:data_end] - target_x_data[data_start:data_end], linestyle='-', color='red', label="x error")

# plt.plot(time_axes, optitrack_y_data[data_start:data_end] - target_y_data[data_start:data_end], linestyle='-', color='green', label="y error")

# plt.plot(time_axes, optitrack_z_data[data_start:data_end] - target_z_data[data_start:data_end], linestyle='-', color='blue', label="z error")
# plt.xlabel('time')
# plt.ylabel('error')
# # plt.title("Floaty's position")
# plt.grid(True)
# plt.legend()
# plt.ylim((-0.15,0.15))

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Hover/"

# plt.savefig(path_to_figs+'hover_position_error.pdf', format='pdf')

# plt.show()

# # -------------------  Angular rates estimation -------------------
# plt.figure(figsize=(20, 8))
# plt.plot(x_values, gyro_x, linestyle='-', color='red')
# plt.plot(x_values, est_roll_rate_data, linestyle='--', color='red')
# plt.plot(x_values, optitrack_roll_rate_data, linestyle='-.', color='red')

# plt.plot(x_values, gyro_y, linestyle='-', color='green')
# plt.plot(x_values, est_pitch_rate_data, linestyle='--', color='green')
# plt.plot(x_values, optitrack_pitch_rate_data, linestyle='-.', color='green')

# plt.plot(x_values, gyro_z, linestyle='-', color='blue')
# plt.plot(x_values, est_yaw_rate_data, linestyle='--', color='blue')
# plt.plot(x_values, optitrack_yaw_rate_data, linestyle='-.', color='blue')
# plt.xlabel('Index')
# plt.ylabel('Angular rates in degrees')
# plt.ylim((-500,500))
# plt.title('Angular rates')
# plt.grid(True)
# plt.show()

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

# plt.figure(figsize=(20, 5))
# plt.plot(x_values, command_m1, linestyle='--', color='red')
# plt.plot(x_values, command_m2, linestyle='--', color='green')
# plt.plot(x_values, command_m3, linestyle='--', color='blue')
# plt.plot(x_values, command_m4, linestyle='--', color='black')
# plt.xlabel('Index')
# plt.ylabel('Flap angles in degrees')
# plt.title('Control command measurement')
# plt.grid(True)
# plt.show()


# # # -------------------  Flap Compound command  -------------------

# plt.figure(figsize=(20, 8))
# # plt.plot(x_values, compound_control_1, linestyle='-', color='red', label="X \ Pitch")
# plt.plot(x_values, compound_control_2, linestyle='-', color='green', label="Y \ Roll")
# # plt.plot(x_values, compound_control_3, linestyle='-', color='blue', label="Z")
# plt.plot(x_values, compound_control_4, linestyle='-', color='black', label="Yaw")
# plt.xlabel('Index')
# plt.ylabel('Flap angles in degrees')
# plt.title('Control compound command measurement')
# plt.legend()
# plt.grid(True)
# plt.show()

# # -------------------  Y tracking -------------------

# tracking_y_data = optitrack_y_data[1730:2860]
# tracking_y_target = target_y_data[1730:2860]
# x_axis_data = [i/50.0 for i in range(len(tracking_y_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_y_target, linestyle='-', color='black', label='target y')
# plt.plot(x_axis_data, tracking_y_data, linestyle='-', color='blue', label="robot's y")
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('y position [m]', fontsize = label_font_size)
# plt.title("tracking y", fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# plt.ylim((-0.25, 0.25))
# # Display the legend
# plt.legend(loc='lower right', fontsize = legend_font_size)

# plt.grid(True)
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Y/"
# plt.savefig(path_to_figs+'y.pdf', format='pdf')
# plt.show()

# y_command_valus = compound_control_2[1730:2860]
# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, y_command_valus, linestyle='-', color='black')
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('y control input', fontsize = label_font_size)
# plt.title('tracking y input', fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# plt.ylim((-45, 45))
# plt.grid(True)
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Y/"
# plt.savefig(path_to_figs+'y_input.pdf', format='pdf')
# plt.show()

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Y/"


# # exp 1
# start_data = 1206
# end_data = 2455
# end_data = start_data + 1130
# tracking_y_data = optitrack_y_data[start_data:end_data]
# tracking_y_target = target_y_data[start_data:end_data]/1.6
# x_axis_data = [i/50.0 for i in range(len(tracking_y_target))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_y_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'y_1.pdf', format='pdf')
# plt.show()


# # exp 2
# start_data = 1255
# end_data = 2499
# end_data = start_data + 1130
# tracking_y_data = optitrack_y_data[start_data:end_data]
# tracking_y_target = target_y_data[start_data:end_data]/1.6
# x_axis_data = [i/50.0 for i in range(len(tracking_y_target))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_y_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'y_2.pdf', format='pdf')
# plt.show()


# # exp 3
# start_data = 865
# end_data = 2365
# end_data = start_data + 1130
# tracking_y_data = optitrack_y_data[start_data:end_data]
# tracking_y_target = target_y_data[start_data:end_data]/1.6
# x_axis_data = [i/50.0 for i in range(len(tracking_y_target))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_y_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'y_3.pdf', format='pdf')
# plt.show()


# # exp 4
# start_data = 882
# end_data = 2382
# end_data = start_data + 1130
# tracking_y_data = optitrack_y_data[start_data:end_data]
# tracking_y_target = target_y_data[start_data:end_data]/1.6
# x_axis_data = [i/50.0 for i in range(len(tracking_y_target))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_y_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'y_4.pdf', format='pdf')
# plt.show()




# # -------------------  Z tracking -------------------

# tracking_z_data = optitrack_z_data[1100:2400]
# tracking_z_target = target_z_data[1100:2400]
# x_axis_data = [i/50.0 for i in range(len(tracking_z_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_z_target, linestyle='-', color='black', label='target z')
# plt.plot(x_axis_data, tracking_z_data, linestyle='-', color='blue', label="robot's z")
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('z position [m]', fontsize = label_font_size)
# plt.title("tracking z", fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# # Display the legend
# plt.legend(loc='lower right', fontsize = legend_font_size)

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Z/"
# plt.savefig(path_to_figs+'z.pdf', format='pdf')
# plt.show()
# tikzplotlib.save("z_tracking.tex")

# z_command_valus = compound_control_3[1100:2400]
# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, z_command_valus, linestyle='-', color='black')
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('z control input', fontsize = label_font_size)
# plt.title('tracking z input', fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Z/"
# plt.savefig(path_to_figs+'z_input.pdf', format='pdf')
# plt.show()
# tikzplotlib.save("z_input.tex")


# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Z/"


# # exp 1
# start_data = 2118
# end_data = start_data+1300
# tracking_z_data = optitrack_z_data[start_data:end_data]
# tracking_z_target = target_z_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_z_data))]


# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_z_data, linestyle='-', color='blue', label="robot's z")

# plt.savefig(path_to_figs+'z_1.pdf', format='pdf')
# plt.show()


# # exp 2
# start_data = 2521
# end_data = start_data+1300
# tracking_z_data = optitrack_z_data[start_data:end_data]
# tracking_z_target = target_z_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_z_data))]


# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_z_data, linestyle='-', color='blue', label="robot's z")

# plt.savefig(path_to_figs+'z_2.pdf', format='pdf')
# plt.show()



# # exp 3
# start_data = 993
# end_data = start_data+1300
# tracking_z_data = optitrack_z_data[start_data:end_data]
# tracking_z_target = target_z_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_z_data))]


# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_z_data, linestyle='-', color='blue', label="robot's z")

# plt.savefig(path_to_figs+'z_3.pdf', format='pdf')
# plt.show()



# # exp 4
# start_data = 2713
# end_data = start_data+1300
# tracking_z_data = optitrack_z_data[start_data:end_data]
# tracking_z_target = target_z_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_z_data))]


# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_z_data, linestyle='-', color='blue', label="robot's z")

# plt.savefig(path_to_figs+'z_4.pdf', format='pdf')
# plt.show()



# # -------------------  Yaw tracking square -------------------

# # Using the gyro_x to transfer the target_Yaw value
# tracking_yaw_data = optitrack_yaw_data[990:1200]
# tracking_yaw_target = target_yaw_data[990:1200]
# tracking_yaw_target[990] = -108
# tracking_yaw_target[991] = -108
# tracking_yaw_target[992] = -108
# tracking_yaw_target[993] = -108
# tracking_yaw_target[994] = -108
# x_axis_data = [i/50.0 -0.1 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='-', color='black', label='target yaw')
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue', label="robot's yaw")
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw angle [deg]', fontsize = label_font_size)
# plt.title("tracking yaw angle", fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)

# # Display the legend
# plt.legend(fontsize = legend_font_size, loc='lower right')

# plt.grid(True)
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# plt.savefig(path_to_figs+'yaw_square.pdf', format='pdf')

# plt.show()
# tikzplotlib.save("yaw_angle_tracking.tex")

# yaw_command_valus = compound_control_4[990:1200]
# yaw_command_valus[990] = 0
# yaw_command_valus[991] = -0.5
# yaw_command_valus[992] = -0.2
# yaw_command_valus[993] = 0.6
# yaw_command_valus[994] = 0.3
# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, yaw_command_valus, linestyle='-', color='black')
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw control input', fontsize = label_font_size)
# plt.title('tracking yaw input', fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# plt.ylim((-45, 45))

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# plt.savefig(path_to_figs+'yaw_square_input.pdf', format='pdf')
# plt.show()
# # tikzplotlib.save("yaw_angle_input.tex")

# # -------------------  Yaw tracking sine 0.15 Hz -------------------

# # Using the gyro_x to transfer the target_Yaw value
# tracking_yaw_data = optitrack_yaw_data[1300:3400] # 1300:3400
# tracking_yaw_target = target_yaw_data[1300:3400] # 1300:3400
# x_axis_data = [i/50.0 -0.1 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='-', color='black', label='target yaw')
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue', label="robot's yaw")
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw angle [deg]', fontsize = label_font_size)
# plt.title("tracking yaw angle", fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)

# # Display the legend
# plt.legend(fontsize = legend_font_size, loc='lower right')

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# plt.savefig(path_to_figs+'yaw_15.pdf', format='pdf')
# plt.show()
# tikzplotlib.save("yaw_angle_tracking.tex")

# yaw_command_valus = compound_control_4[1300:3400] # 1300:3400
# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, yaw_command_valus, linestyle='-', color='black')
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw control input', fontsize = label_font_size)
# plt.title('tracking yaw input', fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# plt.ylim((-45, 45))

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# plt.savefig(path_to_figs+'yaw_15_input.pdf', format='pdf')

# plt.show()

# # ==================== Extra Experiments ====================
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.15_Hz/"

# # exp 1
# start_data = 1740
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_1.pdf', format='pdf')
# plt.show()


# # exp 2
# start_data = 1010
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_2.pdf', format='pdf')
# plt.show()


# # exp 3
# start_data = 2124
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_3.pdf', format='pdf')
# plt.show()


# # exp 4
# start_data = 1125
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_4.pdf', format='pdf')
# plt.show()

# # exp 5
# start_data = 2010
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# tracking_yaw_target = -1*target_yaw_data[start_data:end_data] 
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='--', color='blue')

# plt.savefig(path_to_figs+'yaw_5.pdf', format='pdf')
# plt.show()





# # -------------------  Yaw tracking sine 0.10 Hz -------------------

# # Using the gyro_x to transfer the target_Yaw value
# tracking_yaw_data = optitrack_yaw_data[1150:3250] # 1150:3250
# tracking_yaw_target = target_yaw_data[1150:3250] # 1150:3250
# x_axis_data = [i/50.0 -0.1 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='-', color='black', label='target yaw')
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue', label="robot's yaw")
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw angle [deg]', fontsize = label_font_size)
# plt.title("tracking yaw angle", fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)

# # Display the legend
# plt.legend(fontsize = legend_font_size, loc='lower right')

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# plt.savefig(path_to_figs+'yaw_10.pdf', format='pdf')
# plt.show()
# tikzplotlib.save("yaw_angle_tracking.tex")

# yaw_command_valus = compound_control_4[1150:3250] # 1150:3250
# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, yaw_command_valus, linestyle='-', color='black')
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw control input', fontsize = label_font_size)
# plt.title('tracking yaw input', fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# plt.ylim((-45, 45))

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# plt.savefig(path_to_figs+'yaw_10_input.pdf', format='pdf')
# plt.show()

# # ==================== Extra Experiments ====================
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.10_Hz/"

# # exp 1
# start_data = 847
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_1.pdf', format='pdf')
# plt.show()


# # exp 2
# start_data = 1255
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_2.pdf', format='pdf')
# plt.show()


# # exp 3
# start_data = 1380
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_3.pdf', format='pdf')
# plt.show()


# # exp 4
# start_data = 966
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_4.pdf', format='pdf')
# plt.show()

# # exp 5
# start_data = 1910
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# tracking_yaw_target = -1*target_yaw_data[start_data:end_data] 
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='--', color='blue')

# plt.savefig(path_to_figs+'yaw_5.pdf', format='pdf')
# plt.show()



# # -------------------  Yaw tracking sine 0.05 and 0.02 Hz -------------------

# Using the gyro_x to transfer the target_Yaw value
# tracking_yaw_data = optitrack_yaw_data[1100:2200] # 1100:2500
# tracking_yaw_target = target_yaw_data[1100:2200] # 1100:2500
# x_axis_data = [i/50.0 -0.1 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='-', color='black', label='target yaw')
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue', label="robot's yaw")
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw angle [deg]', fontsize = label_font_size)
# plt.title("tracking yaw angle", fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)

# # Display the legend
# plt.legend(fontsize = legend_font_size, loc='lower right')

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# # plt.savefig(path_to_figs+'yaw_05.pdf', format='pdf')
# plt.savefig(path_to_figs+'yaw_02.pdf', format='pdf')
# plt.show()
# tikzplotlib.save("yaw_angle_tracking.tex")

# yaw_command_valus = compound_control_4[1100:2200] # 1100:2500
# plt.figure(figsize = experiments_fig_size)
# plt.plot(x_axis_data, yaw_command_valus, linestyle='-', color='black')
# plt.xlabel('time [s]', fontsize = label_font_size)
# plt.ylabel('yaw control input', fontsize = label_font_size)
# plt.title('tracking yaw input', fontsize = title_font_size)
# plt.xticks(fontsize = ticks_font_size)
# plt.yticks(fontsize = ticks_font_size)
# plt.ylim((-45, 45))

# plt.grid(True)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Yaw/"
# # plt.savefig(path_to_figs+'yaw_05_input.pdf', format='pdf')
# plt.savefig(path_to_figs+'yaw_02_input.pdf', format='pdf')

# plt.show()

# # ==================== Extra Experiments 0.05-Hz ====================
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.05_Hz/"

# # exp 1
# start_data = 896
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# tracking_yaw_target = -1*target_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='--', color='blue')

# plt.savefig(path_to_figs+'yaw_1.pdf', format='pdf')
# plt.show()


# # exp 2
# start_data = 860
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_2.pdf', format='pdf')
# plt.show()


# # exp 3
# start_data = 1147
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_3.pdf', format='pdf')
# plt.show()


# # exp 4
# start_data = 850
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_4.pdf', format='pdf')
# plt.show()

# # exp 5
# start_data = 1393
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_5.pdf', format='pdf')
# plt.show()


# # ==================== Extra Experiments 0.02-Hz ====================
# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/0.02_Hz/"

# # exp 1
# start_data = 1000
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# tracking_yaw_target = -1*target_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')
# plt.plot(x_axis_data, tracking_yaw_target, linestyle='--', color='blue')

# plt.savefig(path_to_figs+'yaw_1.pdf', format='pdf')
# plt.show()


# # exp 2
# start_data = 990
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_2.pdf', format='pdf')
# plt.show()


# # exp 3
# start_data = 1145
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_3.pdf', format='pdf')
# plt.show()


# # exp 4
# start_data = 1957
# end_data = start_data + 2000
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_4.pdf', format='pdf')
# plt.show()


# # exp 5
# start_data = 1880
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_5.pdf', format='pdf')
# plt.show()


# # exp 6
# start_data = 573
# end_data = start_data + 2000
# tracking_yaw_data = -1*optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_6.pdf', format='pdf')
# plt.show()



# # -------------------  Yaw tracking square  -------------------

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Yaw/step/"

# # exp 1
# start_data = 1606
# end_data = start_data + 210 # 500
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_1.pdf', format='pdf')
# plt.show()


# # exp 2
# # start_data = end_data
# start_data = start_data + 500
# end_data = start_data + 210 # 500 
# tracking_yaw_data = 90 - optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_2.pdf', format='pdf')
# plt.show()


# # exp 3
# # start_data = end_data
# start_data = start_data + 500
# end_data = start_data + 210 # 500 
# tracking_yaw_data = optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_3.pdf', format='pdf')
# plt.show()


# # exp 4
# # start_data = end_data
# start_data = start_data + 500
# end_data = start_data + 210 # 500 
# tracking_yaw_data = 90 - optitrack_yaw_data[start_data:end_data]
# x_axis_data = [i/50.0 for i in range(len(tracking_yaw_data))]

# plt.figure(figsize=experiments_fig_size)
# plt.plot(x_axis_data, tracking_yaw_data, linestyle='-', color='blue')

# plt.savefig(path_to_figs+'yaw_4.pdf', format='pdf')
# plt.show()



# # -------------------  Robustness side fan  -------------------

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/After_reviews/Robustness_airflow_fan/"

# # exp 5
# start_data = 2525
# end_data = start_data + 1000
# tracking_y_data = optitrack_y_data[start_data:end_data]
# tracking_x_data = optitrack_x_data[start_data:end_data]
# tracking_error = np.sqrt(np.square(tracking_y_data)+np.square(tracking_x_data))
# x_axis_data = [i/50.0 for i in range(len(tracking_y_data))]

# # Font settings for publication quality
# plt.rcParams.update({
#     'font.size': 10,             # Base font size
#     'axes.labelsize': 12,        # X and Y labels
#     'axes.titlesize': 14,        # Title
#     'xtick.labelsize': 10,       # X tick labels
#     'ytick.labelsize': 10,       # Y tick labels
#     'legend.fontsize': 10,       # Legend
#     'font.family': 'sans-serif', # Or 'serif'
#     'font.sans-serif': ['Arial', 'Helvetica', 'DejaVu Sans'], # Preferred sans-serif fonts
#     # 'font.serif': ['Times New Roman', 'Georgia', 'DejaVu Serif'], # Preferred serif fonts
#     'axes.linewidth': 1.2,       # Linewidth of plot spines
#     'lines.linewidth': 1.5,      # Linewidth of plotted lines
#     'xtick.major.width': 1,    # X major tick width
#     'ytick.major.width': 1,    # Y major tick width
#     'xtick.minor.width': 0.8,  # X minor tick width
#     'ytick.minor.width': 0.8,  # Y minor tick width
#     'xtick.major.size': 4,     # X major tick length
#     'ytick.major.size': 4,     # Y major tick length
#     'xtick.minor.size': 2,     # X minor tick length
#     'ytick.minor.size': 2,     # Y minor tick length
# })

# # Create figure and axes object
# fig, ax = plt.subplots(figsize=experiments_fig_size)

# # Plot the data
# ax.plot(x_axis_data, tracking_y_data, linestyle='-', color='blue', label='Tracking Data Y') # Added label for legend (if needed)
# # ax.plot(tracking_x_data, tracking_y_data, linestyle='-', color='blue', label='Tracking Data') # Added label for legend (if needed)

# # Set labels and title
# ax.set_xlabel("Time (s)")
# ax.set_ylabel("Y Position (units)") # Replace 'units' with actual units, e.g., 'm', 'mm'
# # ax.set_title("Experiment 5: Y-axis Tracking Over Time") # Titles are often omitted if figure captions are detailed

# # Add a grid for better readability (optional, but often helpful)
# ax.grid(True, linestyle='--', alpha=0.7, color='gray')

# # Despine (remove top and right plot borders)
# ax.spines['top'].set_visible(False)
# ax.spines['right'].set_visible(False)

# # Adjust tick parameters (e.g., direction, length) if needed
# ax.tick_params(axis='both', which='major', direction='out')
# # ax.tick_params(axis='both', which='minor', direction='out') # If you enable minor ticks

# # Optionally, set explicit X and Y limits if the auto-scaling isn't perfect
# # ax.set_xlim([min_x, max_x])
# # ax.set_ylim([min_y, max_y])
# ax.set_xlim(left=0, right=max(x_axis_data)) # Ensure x-axis starts at 0

# # If you had multiple lines, you would add a legend:
# # ax.legend()

# # Ensure layout is tight to prevent labels from being cut off
# plt.tight_layout()

# # Save the figure
# fig_filename = path_to_figs + 'exp_5_professional.pdf'
# plt.savefig(fig_filename, format='pdf', dpi=300, bbox_inches='tight')
# print(f"Figure saved as {fig_filename}")

# # Show the plot
# plt.show()

# # plt.figure(figsize=experiments_fig_size)
# # plt.plot(x_axis_data, tracking_y_data, linestyle='-', color='blue')

# # plt.savefig(path_to_figs+'exp_5.pdf', format='pdf')
# # plt.show()

# # -------------------  Hover  -------------------

# # Create a 3D plot

# from mpl_toolkits.mplot3d.art3d import Line3DCollection
# from matplotlib import cm

# fig = plt.figure(figsize=(8, 7))
# ax = fig.add_subplot(111, projection='3d')

# data_start=1500
# data_end = 3100
# # data_end = len(optitrack_x_data)

# num_points = data_end-data_start
# # colors = np.arange(num_points) / num_points  # Normalize index to [0, 1]

# sampling_freq = 50
# end_time = int(num_points/sampling_freq) # To calculate the length of the data
# colors = np.linspace(0, end_time, num_points)

# # Prepare the data for Line3DCollection
# points = np.array([optitrack_x_data[data_start:data_end],
#                    optitrack_y_data[data_start:data_end],
#                    optitrack_z_data[data_start:data_end]]).T.reshape(-1, 1, 3)
# segments = np.concatenate([points[:-1], points[1:]], axis=1)

# # Create a Line3DCollection with a color gradient
# cmap = cm.get_cmap('viridis')
# lc = Line3DCollection(segments, cmap=cmap, norm=plt.Normalize(0, end_time))
# lc.set_array(colors)
# lc.set_linewidth(1)

# # Add the collection to the plot
# ax.add_collection3d(lc)


# ax.set_xlim([-0.13, 0.13])
# ax.set_ylim([-0.13, 0.13])
# ax.set_zlim([1.0, 1.3])

# # # ------------------
# # # Plot the 3D path
# # ax.plot(optitrack_x_data, optitrack_y_data, optitrack_z_data, marker='o', linestyle='-', color='b')

# # # # ------------------
# # # Plot the 3D path with color gradient

# # sc = ax.scatter(optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end], c=colors, cmap='viridis', marker='o', alpha=0.8, vmin=0, vmax=end_time)

# # # ax.scatter([0], [0], [0], color='red', marker='o', alpha=0.5, s=500)

# # # Add colorbar
# # cbar_ax = fig.add_axes([0.1, 0.3, 0.03, 0.5])  # [left, bottom, width, height]
# # cbar = fig.colorbar(sc, cax=cbar_ax)
# # # cbar.set_label('time [s]', fontsize=16)

# # Set the same axis limits
# # limit_min = min(optitrack_x_data.min(), optitrack_y_data.min())
# # limit_max = max(optitrack_x_data.max(), optitrack_y_data.max())
# # ax.set_xlim([limit_min, limit_max])
# # ax.set_ylim([limit_min, limit_max])

# # # Set labels and title
# # ax.set_xlabel('x [m]', fontsize=14)
# # ax.set_ylabel('y [m]', fontsize=14)
# # ax.set_zlabel('z [m]', fontsize=14)
# # ax.set_title('3D path visualization', fontsize=16)

# # Add 2D projections on the walls with color mapping
# # ax.scatter(optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], zs=ax.get_zlim()[0], zdir='z', c=cmap(colors / end_time), alpha=0.1, s=1)  # XY plane
# # ax.scatter(optitrack_x_data[data_start:data_end], optitrack_z_data[data_start:data_end], zs=ax.get_ylim()[0], zdir='y', c=cmap(colors / end_time), alpha=0.1, s=1)  # XZ plane
# # ax.scatter(optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end], zs=ax.get_xlim()[0], zdir='x', c=cmap(colors / end_time), alpha=0.1, s=1)  # YZ plane

# ax.plot(optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], zs=ax.get_zlim()[0], zdir='z', c='gray', alpha=0.3)  # XY plane  , s=1
# ax.plot(optitrack_x_data[data_start:data_end], optitrack_z_data[data_start:data_end], zs=ax.get_ylim()[0], zdir='y', c='gray', alpha=0.3)  # XZ plane  , s=1
# ax.plot(optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end], zs=ax.get_xlim()[0], zdir='x', c='gray', alpha=0.3)  # YZ plane  , s=1

# # Set the viewing angle
# ax.view_init(elev=27, azim=45)  

# plt.colorbar(lc, ax=ax, shrink=0.5, aspect=10)

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Hover/"
# plt.savefig(path_to_figs+'3d_plot_with_projections.pdf', format='pdf')

# # Show the plot
# plt.show()

# # Create a figure with 3 subplots for each projection (XY, XZ, YZ)
# fig, axs = plt.subplots(1, 3, figsize=(15, 5))
# red_color = (214/255,39/255,40/255)

# axs[0].grid(True, linestyle='--', alpha=0.3, color='gray')
# axs[1].grid(True, linestyle='--', alpha=0.3, color='gray')
# axs[2].grid(True, linestyle='--', alpha=0.3, color='gray')

# # XY projection (XZ plane)
# axs[0].plot(optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], c='gray', alpha=0.5)
# # axs[0].scatter(optitrack_x_data[data_start:data_end], optitrack_y_data[data_start:data_end], c=colors, cmap='viridis', alpha=0.5)
# # axs[0].scatter([0], [0], color=red_color, marker='x', s=200, linewidths=4, alpha=0.85)
# # axs[0].scatter([-0.03], [0.08], color=red_color, marker='o', s=100, linewidths=3, alpha=0.85)
# axs[0].set_xlabel('x [m]', fontsize=14)
# axs[0].set_ylabel('y [m]', fontsize=14)
# axs[0].set_xlim([-0.2, 0.2])
# axs[0].set_ylim([-0.2, 0.2])
# axs[0].set_title('XY plane', fontsize=16)

# # XZ projection (YZ plane)
# axs[1].plot(optitrack_x_data[data_start:data_end], optitrack_z_data[data_start:data_end], c='gray', alpha=0.5)
# # axs[1].scatter(optitrack_x_data[data_start:data_end], optitrack_z_data[data_start:data_end], c=colors, cmap='viridis', alpha=0.5)
# # axs[1].scatter([0], [0], color=red_color, marker='x', s=200, linewidths=4, alpha=0.85)
# # axs[1].scatter([-0.03], [-0.65], color=red_color, marker='o', s=100, linewidths=3, alpha=0.85)
# axs[1].set_xlabel('x [m]', fontsize=14)
# axs[1].set_ylabel('z [m]', fontsize=14)
# axs[1].set_xlim([-0.2, 0.2])
# axs[1].set_ylim([1.0, 1.35])
# axs[1].set_title('XZ plane', fontsize=16)

# # YZ projection (YZ plane)
# axs[2].plot(optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end], c='gray', alpha=0.5)
# # axs[2].scatter(optitrack_y_data[data_start:data_end], optitrack_z_data[data_start:data_end], c=colors, cmap='viridis', alpha=0.5)
# # axs[2].scatter([0], [0], color=red_color, marker='x', s=200, linewidths=4, alpha=0.85)
# # axs[2].scatter([+0.08], [-0.65], color=red_color, marker='o', s=100, linewidths=3, alpha=0.85)
# axs[2].set_xlabel('y [m]', fontsize=14)
# axs[2].set_ylabel('z [m]', fontsize=14)
# axs[2].set_xlim([-0.2, 0.2])
# axs[2].set_ylim([1.0, 1.35])
# axs[2].set_title('YZ plane', fontsize=16)

# # Adjust layout to avoid overlap
# plt.tight_layout()

# path_to_figs = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/floaty_estimator/Science_Robotics_Experiments/Hover/"
# plt.savefig(path_to_figs+'hover_projections.pdf', format='pdf')

# # Show plot
# plt.show()




data_start=1500
data_end = 3100
# data_end = len(optitrack_x_data)
num_points = data_end-data_start
# colors = np.arange(num_points) / num_points  # Normalize index to [0, 1]

sampling_freq = 50
end_time = int(num_points/sampling_freq) # To calculate the length of the data
colors = np.linspace(0, end_time, num_points)
time = np.linspace(0, end_time, num_points)


# -------------------  Position error -------------------
plt.figure(figsize=(10, 3))
plt.plot(time, optitrack_x_data[data_start:data_end] - target_x_data[data_start:data_end], linestyle='-', color='red', label="x position error")
plt.plot(time, optitrack_y_data[data_start:data_end] - target_y_data[data_start:data_end], linestyle='-', color='green', label="y position error")
plt.plot(time, optitrack_z_data[data_start:data_end] - target_z_data[data_start:data_end], linestyle='-', color='blue', label="z position error")
plt.ylim([-0.2, 0.2])
plt.xlabel('time [s]')
plt.ylabel('error [m]')
plt.title("error in position")
plt.grid(True)
plt.legend()
plt.show()


# -------------------  Orientation error -------------------
plt.figure(figsize=(10, 3))
plt.plot(time, optitrack_roll_data[data_start:data_end], linestyle='-', color='red', label="roll error")
plt.plot(time, optitrack_pitch_data[data_start:data_end], linestyle='-', color='green', label="pitch error")
plt.plot(time, optitrack_yaw_data[data_start:data_end], linestyle='-', color='blue', label="yaw error")
plt.ylim([-45, 45])
plt.xlabel('time [s]')
plt.ylabel('error [deg]')
plt.title("error in orientation")
plt.grid(True)
plt.legend()
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