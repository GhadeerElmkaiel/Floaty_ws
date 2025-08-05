import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os
import tikzplotlib
import time


def calc_theoretical_values_at_sensor(angles):
    rho = 1.225
    v_air = 8.819
    Cd = 1.2
    dSensor = 0.08
    flap_area = 0.0084
    base_area = 0.0064
    dCp = 0.0742857142
    const = 0.5*(v_air**2)*rho*Cd
    # F_max = const*(base_area+4*flap_area)
    F_total = np.array([0, 0, -const*base_area])
    F_flaps = np.zeros((4,3))
    T_total = np.array([0, 0, 0])
    T_flaps = np.zeros((4,3))
    for i in range(4):
        phy_i = np.pi/2*i
        F_flaps[i] = (flap_area*const*(np.cos(angles[i])**2))*np.array([np.sin(phy_i)*np.sin(angles[i]), np.cos(phy_i)*np.sin(angles[i]), -np.cos(angles[i])])
        F_total =  np.add(F_total, F_flaps[i])
        d_to_flap_i = np.array([dCp*np.cos(phy_i), -dCp*np.sin(phy_i), dSensor])
        T_flaps[i,:] = np.cross(d_to_flap_i, F_flaps[i])
        T_total = np.add(T_total, T_flaps[i])

    return F_total, T_total

# def calc_theoretical_values_at_base(angles):
#     rho = 1.225
#     v_air = 8.819
#     Cd = 1.2
#     dSensor = 0.0
#     flap_area = 0.0084
#     base_area = 0.0064
#     dCp = 0.0742857142
#     const = 0.5*(v_air**2)*rho*Cd
#     # F_max = const*(base_area+4*flap_area)
#     F_total = np.array([0, 0, -const*base_area])
#     F_flaps = np.zeros((4,3))
#     T_total = np.array([0, 0, 0])
#     T_flaps = np.zeros((4,3))
#     for i in range(4):
#         phy_i = np.pi/2*i
#         F_flaps[i] = (flap_area*const*(np.cos(angles[i])**2))*np.array([np.sin(phy_i)*np.sin(angles[i]), np.cos(phy_i)*np.sin(angles[i]), -np.cos(angles[i])])
#         F_total =  np.add(F_total, F_flaps[i])
#         d_to_flap_i = np.array([dCp*np.cos(phy_i), -dCp*np.sin(phy_i), dSensor])
#         T_flaps[i,:] = np.cross(d_to_flap_i, F_flaps[i])
#         T_total = np.add(T_total, T_flaps[i])

#     return F_total, T_total


# def move_measurements_to_base(measurements):

#     dSensor = 0.008
#     F_measured = measurements[:3]
#     T_measured = measurements[3:]
#     F_at_base = F_measured
#     T_at_base = np.array([0, 0, 0])
#     sensor_to_base = [0, 0, dSensor]
#     T_at_base = np.add(T_measured, -1*np.cross(sensor_to_base, F_measured))

#     return F_at_base, T_at_base

def motor_angles_str_to_rad(angles_str):
    angles_rad = []
    for angle in angles_str:
        rad_val = (float(angle)-60)*np.pi/180
        angles_rad.append(rad_val)
    return angles_rad


plt.style.use("ggplot")

current_file_dir = os.path.dirname(os.path.realpath(__file__))
# data_file_dir = current_file_dir +"/../../data/static_system_identification/current_experiment/"
data_file_dir = current_file_dir +"/../../data/static_system_identification/experiment_2/"

# folders_dir = [x[0] for x in os.walk(data_file_dir)  if x[0]!= data_file_dir and not "Figures" in x[0] and not "test" in x[0] and not "Rotated" in x[0]]
folders_dir = [x[0] for x in os.walk(data_file_dir)  if x[0]!= data_file_dir and not "Figures" in x[0] and not "test" in x[0]]

for folder_dir in folders_dir:
    folders = folder_dir.split("/")
    folder_name = folders[-1]
    # folder_name = "2"
    # folder_dir = '/home/gelmkaiel/Floaty/ws/src/floaty_pkg/scripts/static_system_identification/../../data/static_system_identification/2'
    files = [f for f in os.listdir(folder_dir) if os.path.isfile(os.path.join(folder_dir, f))]
    if "1" in folder_name:
        motor=1
    elif "2" in folder_name:
        motor=2
    elif "3" in folder_name:
        motor=3
    elif "4" in folder_name:
        motor=4

    data_average = np.zeros([6,len(files)])
    data_std = np.zeros([6,len(files)])
    theoretical_vals = np.zeros([6,len(files)])

    # For rotating the sensor axis to be the same as the robot's axis
    data_average_rotated = np.zeros([6,len(files)])
    data_std_rotated = np.zeros([6,len(files)])
    theoretical_vals_rotated = np.zeros([6,len(files)])

    angles_values = np.zeros(len(files))

    i = 0
    for file in files:
        extention_split = file.split(".")
        motors_split = extention_split[0].split("_")
        motor_angle_val = int(motors_split[-5+motor])

        # For theoretical values
        angles_in_deg_str = motors_split[-4:]
        angles_rad = motor_angles_str_to_rad(angles_in_deg_str)
        theoretical_forces, theoretical_torques = calc_theoretical_values_at_sensor(angles_rad)
        theoretical_vals[0:3,i] = theoretical_forces
        theoretical_vals[3:6,i] = theoretical_torques

        # if motor%2==0:
        #     relative_angle_vale = motor_angle_val - 60
        # else:
        #     relative_angle_vale = 60 - motor_angle_val
        relative_angle_vale = motor_angle_val - 60
    
        df = pd.read_csv(folder_dir +"/"+ file, header=None)
        np_arr = df.to_numpy()

        # For rotating the sensor axis to be the same as the robot's axis
        np_arr_rotated = np.zeros(np_arr.shape)
        for k in range(np_arr.shape[0]):
            np_arr_rotated[k,0] = (1/np.sqrt(2))*(np_arr[k,0]-np_arr[k,1])
            np_arr_rotated[k,1] = (1/np.sqrt(2))*(np_arr[k,0]+np_arr[k,1])
            np_arr_rotated[k,2] = np_arr[k,2]
            np_arr_rotated[k,3] = (1/np.sqrt(2))*(np_arr[k,3]-np_arr[k,4])
            np_arr_rotated[k,4] = (1/np.sqrt(2))*(np_arr[k,3]+np_arr[k,4])
            np_arr_rotated[k,5] = np_arr[k,5]
        angles_values[i] = relative_angle_vale

        theoretical_vals_rotated[0,i] = (1/np.sqrt(2))*(theoretical_vals[0,i] - theoretical_vals[1,i])
        theoretical_vals_rotated[1,i] = (1/np.sqrt(2))*(theoretical_vals[0,i] + theoretical_vals[1,i])
        theoretical_vals_rotated[2,i] = theoretical_vals[2,i]
        theoretical_vals_rotated[3,i] = (1/np.sqrt(2))*(theoretical_vals[3,i] - theoretical_vals[4,i])
        theoretical_vals_rotated[4,i] = (1/np.sqrt(2))*(theoretical_vals[3,i] + theoretical_vals[4,i])
        theoretical_vals_rotated[5,i] = theoretical_vals[5,i]

        for j in range(6):
            avg = np.average(np_arr[:,j])
            std = np.std(np_arr[:,j])
            data_average[j,i] = avg
            data_std[j,i] = std

            avg_rot = np.average(np_arr_rotated[:,j])
            std_rot = np.std(np_arr_rotated[:,j])
            data_average_rotated[j,i] = avg_rot
            data_std_rotated[j,i] = std_rot

        i = i+1

    fig = plt.figure()
    idx   = np.argsort(angles_values)
    angles_values = np.array(angles_values)[idx]
    markers = ["+", "*", "o", "+", "*", "o"]
    for i in range(3):
        # Sort the angles of the files (as in general, they are not sorted)
        data_average[i,:] = np.array(data_average[i,:])[idx]
        data_std[i,:] = np.array(data_std[i,:])[idx]
        theoretical_vals[i,:] = np.array(theoretical_vals[i,:])[idx]
        plt.errorbar(angles_values, data_average[i], data_std[i])
        plt.scatter(angles_values, theoretical_vals[i],marker = markers[i])

    plt.xlabel("angle")
    plt.title(folder_name+" force")
    
    # plt.legend(["x-force", "y-force", "z-force"])
    plt.legend([" Theoretical x-force", " Theoretical y-force", " Theoretical z-force", "x-force",  "y-force", "z-force"])
    plt.savefig(data_file_dir + "Figures/" + folder_name+"_force.png")
    # time.sleep(4)
    tikzplotlib.save(data_file_dir + "Figures/" + folder_name+"_force.tex")

    plt.show()
    plt.close()


    fig = plt.figure()
    for i in range(3,6):
        # Sort the angles of the files (as in general, they are not sorted)
        data_average[i,:] = np.array(data_average[i,:])[idx]
        data_std[i,:] = np.array(data_std[i,:])[idx]
        theoretical_vals[i,:] = np.array(theoretical_vals[i,:])[idx]
        plt.errorbar(angles_values, data_average[i], data_std[i])
        plt.scatter(angles_values, theoretical_vals[i],marker = markers[i])

    plt.xlabel("angle")
    plt.title(folder_name+" torque")

    # plt.legend(["x-torque", "y-torque", "z-torque"])
    plt.legend([" Theoretical x-torque", " Theoretical y-torque", " Theoretical z-torque", "x-torque",  "y-torque", "z-torque"])
    plt.savefig(data_file_dir + "Figures/" + folder_name+"_torque.png")
    # time.sleep(4)
    tikzplotlib.save(data_file_dir + "Figures/" + folder_name+"_torque.tex")
    plt.show()

    plt.close()


    # Plots of the values after rotating the axis
    fig = plt.figure()
    for i in range(3):
        # Sort the angles of the files (as in general, they are not sorted)
        data_average_rotated[i,:] = np.array(data_average_rotated[i,:])[idx]
        data_std_rotated[i,:] = np.array(data_std_rotated[i,:])[idx]
        theoretical_vals_rotated[i,:] = np.array(theoretical_vals_rotated[i,:])[idx]
        plt.errorbar(angles_values, data_average_rotated[i], data_std_rotated[i])
        plt.scatter(angles_values, theoretical_vals_rotated[i],marker = markers[i])

    plt.xlabel("angle")
    plt.title(folder_name+" force rotated")
    
    # plt.legend(["x-force", "y-force", "z-force"])
    plt.legend([" Theoretical x-force", " Theoretical y-force", " Theoretical z-force", "x-force",  "y-force", "z-force"])
    plt.savefig(data_file_dir + "Figures/" + folder_name+"_force_rotated.png")
    # time.sleep(4)
    tikzplotlib.save(data_file_dir + "Figures/" + folder_name+"_force_rotated.tex")

    plt.show()
    plt.close()


    fig = plt.figure()
    for i in range(3,6):
        # Sort the angles of the files (as in general, they are not sorted)
        data_average_rotated[i,:] = np.array(data_average_rotated[i,:])[idx]
        data_std_rotated[i,:] = np.array(data_std_rotated[i,:])[idx]
        theoretical_vals_rotated[i,:] = np.array(theoretical_vals_rotated[i,:])[idx]
        plt.errorbar(angles_values, data_average_rotated[i], data_std_rotated[i])
        plt.scatter(angles_values, theoretical_vals_rotated[i],marker = markers[i])

    plt.xlabel("angle")
    plt.title(folder_name+" torque rotated")
    # plt.legend(["x-torque", "y-torque", "z-torque"])
    plt.legend([" Theoretical x-torque", " Theoretical y-torque", " Theoretical z-torque", "x-torque",  "y-torque", "z-torque"])
    plt.savefig(data_file_dir + "Figures/" + folder_name+"_torque_rotated.png")
    # time.sleep(4)
    tikzplotlib.save(data_file_dir + "Figures/" + folder_name+"_torque_rotated.tex")
    plt.show()

    plt.close()

print("Done!")
