import numpy as np
import pandas as pd


# Paths to data
pth_to_file = __file__
folders_to_pkg = pth_to_file.split("/")
pth_to_pkg = '/'.join(folders_to_pkg[:-3])
pth_to_data = pth_to_pkg + "/data/static_system_identification/Science_robotics/"
pth_Z_1 = pth_to_data + "Z/data_with_angles_77_42_77_42.csv"
pth_Z_2 = pth_to_data + "Z/data_with_angles_87_32_87_32.csv"

pth_Y_1 = pth_to_data + "Roll_y/data_with_angles_77_32_87_42.csv"
pth_Y_2 = pth_to_data + "Roll_y/data_with_angles_87_42_77_32.csv"

pth_X_1 = pth_to_data + "Pitch_x/data_with_angles_77_42_87_32.csv"
pth_X_2 = pth_to_data + "Pitch_x/data_with_angles_87_32_77_42.csv"

delat_angle = 10*np.pi/180 # 10 degrees 
robot_mass = 0.34  # 340 grams

def calculate_statistics(file_name):
    # Load the data
    try:
        data = pd.read_csv(file_name, header=None)
    except FileNotFoundError:
        print(f"Error: File '{file_name}' not found.")
        return
    
    # Ensure correct number of columns
    if data.shape[1] != 6:
        print("Error: Expected 6 columns (forces and torques), but found", data.shape[1])
        return
    
    # Column names
    columns = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
    data.columns = columns
    
    # Compute statistics
    means = data.mean()
    stds = data.std()
    
    # Display results
    # print("Mean values:")
    # print(means)
    # print("\nVariance values:")
    # print(variances)
    
    return means, stds

# Example usage
if __name__ == "__main__":

    data = pd.read_csv(pth_Z_1, header=None)
    N_data = data.shape[0]
    mean_std_scaler = 1/np.sqrt(N_data)
    # Calculate the z input constant
    means_1, stds_1 = calculate_statistics(pth_Z_1)
    means_2, stds_2 = calculate_statistics(pth_Z_2)
    print(means_2)
    print(stds_2)

    delta_force = means_1["Fz"] - means_2["Fz"]
    delta_acc = delta_force/robot_mass
    z_input_const = delta_acc/delat_angle
    std_const = np.sqrt(stds_1["Fz"]**2+stds_2["Fz"]**2)/(robot_mass*delat_angle)*mean_std_scaler

    print("Z input constant: ", z_input_const)
    print("Z input constant std: ", std_const)

    # Calculate the y input constant
    means_1, stds_1 = calculate_statistics(pth_Y_1)
    means_2, stds_2 = calculate_statistics(pth_Y_2)

    delta_force = means_1["Fy"] - means_2["Fy"]
    delta_acc = delta_force/robot_mass
    y_input_const = delta_acc/delat_angle
    std_const = np.sqrt(stds_1["Fy"]**2+stds_2["Fy"]**2)/(robot_mass*delat_angle)*mean_std_scaler

    print("Y input constant: ", y_input_const)
    print("Y input constant std: ", std_const)

    # Calculate the y input constant
    means_1, stds_1 = calculate_statistics(pth_X_1)
    means_2, stds_2 = calculate_statistics(pth_X_2)

    delta_force = means_1["Fx"] - means_2["Fx"]
    delta_acc = delta_force/robot_mass
    x_input_const = delta_acc/delat_angle
    std_const = np.sqrt(stds_1["Fx"]**2+stds_2["Fx"]**2)/(robot_mass*delat_angle)*mean_std_scaler

    print("X input constant: ", x_input_const)
    print("X input constant std: ", std_const)

    
