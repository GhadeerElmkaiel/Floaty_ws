import numpy as np
import pandas as pd
import os


def move_measurements_to_base(measurements):

    dSensor = 0.08
    F_measured = measurements[:3]
    T_measured = measurements[3:]
    F_at_base = F_measured
    T_at_base = np.array([0, 0, 0])
    sensor_to_base = [0, 0, dSensor]
    T_at_base = np.add(T_measured, -1*np.cross(sensor_to_base, F_measured))

    return F_at_base, T_at_base

def motor_angles_str_to_rad(angles_str):
    angles_rad = []
    for angle in angles_str:
        rad_val = (float(angle)-60)*np.pi/180
        angles_rad.append(rad_val)
    return angles_rad



current_file_dir = os.path.dirname(os.path.realpath(__file__))

data_file_dir = current_file_dir +"/../../data/static_system_identification/Kinks_fine_range/"

folders_dir = [x[0] for x in os.walk(data_file_dir)  if x[0]!= data_file_dir and not "Figures" in x[0] and not "csv" in x[0]]

for folder_dir in folders_dir:
    folders = folder_dir.split("/")
    folder_name = folders[-1]

    files = [f for f in os.listdir(folder_dir) if os.path.isfile(os.path.join(folder_dir, f))]

    motor=1
    if "1" in folder_name:
        motor=1
    elif "hov-25-flap-3" == folder_name:
        motor=3
    elif "2" in folder_name:
        motor=2
    elif "3" in folder_name:
        motor=3
    elif "4" in folder_name:
        motor=4
    # if "4_hover_35" == folder_name:
    #     motor=4

    data_average = np.zeros([6,len(files)])
    data_std = np.zeros([6,len(files)])
    # theoretical_vals = np.zeros([6,len(files)])

    # For rotating the sensor axis to be the same as the robot's axis
    data_average_rotated = np.zeros([6,len(files)])
    data_std_rotated = np.zeros([6,len(files)])
    # theoretical_vals_rotated = np.zeros([6,len(files)])

    # data_average_at_base_rotated = np.zeros([6,len(files)]) # The measurements moved to base frame 
    # data_std_at_base_rotated = np.zeros([6,len(files)])
    # theoretical_vals_at_base_rotated = np.zeros([6,len(files)]) # The theoretical values at base frame
    angles_values = np.zeros(len(files))
    all_angles_values = np.zeros((len(files),4))

    i = 0
    for file in files:
        extention_split = file.split(".")
        motors_split = extention_split[0].split("_")
        motor_angle_val = int(motors_split[-5+motor])

        # For theoretical values
        angles_in_deg_str = motors_split[-4:]
        angles_rad = motor_angles_str_to_rad(angles_in_deg_str)
        # theoretical_forces, theoretical_torques = calc_theoretical_values_at_base(angles_rad)
        # theoretical_forces, theoretical_torques = calc_theoretical_values_kink_at_base(angles_rad)
        # theoretical_vals[0:3,i] = theoretical_forces
        # theoretical_vals[3:6,i] = theoretical_torques

        all_angles_values[i,:] = [int(val) - 60 for val in angles_in_deg_str]
        # if motor%2==0:
        #     relative_angle_vale = motor_angle_val - 60
        # else:
        #     relative_angle_vale = 60 - motor_angle_val
        relative_angle_vale = motor_angle_val - 60
    
        df = pd.read_csv(folder_dir +"/"+ file, header=None)
        np_arr_sensor = df.to_numpy()

        # For rotating the sensor axis to be the same as the robot's axis
        np_arr = np.zeros(np_arr_sensor.shape)
        np_arr_rotated = np.zeros(np_arr.shape)
        for k in range(np_arr.shape[0]):
            np_arr[k,:3], np_arr[k,3:] = move_measurements_to_base(np_arr_sensor[k,:])
            np_arr_rotated[k,0] = (1/np.sqrt(2))*(np_arr[k,0]-np_arr[k,1])
            np_arr_rotated[k,1] = (1/np.sqrt(2))*(np_arr[k,0]+np_arr[k,1])
            np_arr_rotated[k,2] = np_arr[k,2]
            np_arr_rotated[k,3] = (1/np.sqrt(2))*(np_arr[k,3]-np_arr[k,4])
            np_arr_rotated[k,4] = (1/np.sqrt(2))*(np_arr[k,3]+np_arr[k,4])
            np_arr_rotated[k,5] = np_arr[k,5]
        angles_values[i] = relative_angle_vale

        # theoretical_vals_rotated[0,i] = (1/np.sqrt(2))*(theoretical_vals[0,i] - theoretical_vals[1,i])
        # theoretical_vals_rotated[1,i] = (1/np.sqrt(2))*(theoretical_vals[0,i] + theoretical_vals[1,i])
        # theoretical_vals_rotated[2,i] = theoretical_vals[2,i]
        # theoretical_vals_rotated[3,i] = (1/np.sqrt(2))*(theoretical_vals[3,i] - theoretical_vals[4,i])
        # theoretical_vals_rotated[4,i] = (1/np.sqrt(2))*(theoretical_vals[3,i] + theoretical_vals[4,i])
        # theoretical_vals_rotated[5,i] = theoretical_vals[5,i]

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

    idx   = np.argsort(angles_values)
    angles_values = np.array(angles_values)[idx]

    for i in range(6):
        # Sort the angles of the files (as in general, they are not sorted)
        data_average[i,:] = np.array(data_average[i,:])[idx]
        data_std[i,:] = np.array(data_std[i,:])[idx]
        # theoretical_vals[i,:] = np.array(theoretical_vals[i,:])[idx]


    # for i in range(3,6):
    #     # Sort the angles of the files (as in general, they are not sorted)
    #     data_average[i,:] = np.array(data_average[i,:])[idx]
    #     data_std[i,:] = np.array(data_std[i,:])[idx]
    #     theoretical_vals[i,:] = np.array(theoretical_vals[i,:])[idx]



    for i in range(6):
        # Sort the angles of the files (as in general, they are not sorted)
        data_average_rotated[i,:] = np.array(data_average_rotated[i,:])[idx]
        data_std_rotated[i,:] = np.array(data_std_rotated[i,:])[idx]
        # theoretical_vals_rotated[i,:] = np.array(theoretical_vals_rotated[i,:])[idx]


    # for i in range(3,6):
    #     # Sort the angles of the files (as in general, they are not sorted)
    #     data_average_rotated[i,:] = np.array(data_average_rotated[i,:])[idx]
    #     data_std_rotated[i,:] = np.array(data_std_rotated[i,:])[idx]
    #     theoretical_vals_rotated[i,:] = np.array(theoretical_vals_rotated[i,:])[idx]


    measurements_names = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    angles_names = ["Flap_1", "Flap_2", "Flap_3", "Flap_4"]

    for i in range(4):
        # Sort the angles of the files (as in general, they are not sorted)
        all_angles_values[:,i] = np.array(all_angles_values[:,i])[idx]
    pd_angles = pd.DataFrame(all_angles_values, columns=angles_names)
    pd_angles = pd_angles.reset_index(drop=True)

    pd_avg = pd.DataFrame(np.transpose(data_average), columns=measurements_names)
    pd_avg = pd_avg.reset_index(drop=True)
    
    pd_std = pd.DataFrame(np.transpose(data_std), columns=measurements_names)
    pd_std = pd_std.reset_index(drop=True)
    

    pd_avg_rot = pd.DataFrame(np.transpose(data_average_rotated), columns=measurements_names)
    pd_avg_rot = pd_avg_rot.reset_index(drop=True)
    
    pd_std_rot = pd.DataFrame(np.transpose(data_std_rotated), columns=measurements_names)
    pd_std_rot = pd_std_rot.reset_index(drop=True)
    

    pd_angles.to_csv(data_file_dir + "csv_files/" + folder_name + "_angles.csv")
    pd_avg.to_csv(data_file_dir + "csv_files/" + folder_name + "_avg.csv")
    pd_std.to_csv(data_file_dir + "csv_files/" + folder_name + "_std.csv")
    pd_avg_rot.to_csv(data_file_dir + "csv_files/" + folder_name + "_avg_rot.csv")
    pd_std_rot.to_csv(data_file_dir + "csv_files/" + folder_name + "_std_rot.csv")

print("Done!")
