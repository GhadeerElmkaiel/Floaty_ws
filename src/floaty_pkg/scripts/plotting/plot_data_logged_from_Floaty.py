import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a Pandas DataFrame
file_path = '/home/floaty/Floaty/ws/src/floaty_pkg/data/floaty_estimator/floaty_data_63.csv' 
data = pd.read_csv(file_path)

# Extract the 'Opt_roll' column data
opt_roll_data = data['roll']
opt_pitch_data = data['pitch']

optitrack_x_data = data['Optitrack_x']
optitrack_y_data = data['Optitrack_y']
optitrack_z_data = data['Optitrack_z']

est_x_data = data['est_x']
est_y_data = data['est_y']
est_z_data = data['est_z']


optitrack_roll_data = data['roll']*180/3.14
optitrack_pitch_data = data['pitch']*180/3.14
optitrack_yaw_data = data['yaw']*180/3.14

est_roll_data = data['est_roll']*180/3.14
est_pitch_data = data['est_pitch']*180/3.14
est_yaw_data = data['est_yaw']*180/3.14


est_roll_rate_data = data['est_roll_rate']*180/3.14
est_pitch_rate_data = data['est_pitch_rate']*180/3.14
est_yaw_rate_data = data['est_yaw_rate']*180/3.14


gyro_x = data['gyro_x']
gyro_y = data['gyro_y']
gyro_z = data['gyro_z']


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

plt.plot(x_values, gyro_y, linestyle='-', color='green')
plt.plot(x_values, est_pitch_rate_data, linestyle='--', color='green')

plt.plot(x_values, gyro_z, linestyle='-', color='blue')
plt.plot(x_values, est_yaw_rate_data, linestyle='--', color='blue')
plt.xlabel('Index')
plt.ylabel('Angular rates in degrees')
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
plt.plot(x_values, command_m3, linestyle='--', color='yellow')
plt.plot(x_values, command_m4, linestyle='--', color='black')
plt.xlabel('Index')
plt.ylabel('Flap angles in degrees')
plt.title('Unstable frequency measurement')
plt.grid(True)
plt.show()

input("a")