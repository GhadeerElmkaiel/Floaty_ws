import rospy
from floaty_msgs.msg import floaty_info_msg, floaty_flexible_msg
import csv

first_data = True
data_file_path = None
file_name = None
data_file_base_name = None
flexible_msg_data_title = None

def create_file():
    global data_file_path, file_name
    with open(data_file_path+"iteration_num.txt", 'r+') as file:
        # Read the integer value from the file
        value = int(file.read())
        
        # Increment the value by 1
        value += 1
        
        # Go back to the beginning of the file to overwrite the existing value
        file.seek(0)
        
        # Write the updated value back to the file
        file.write(str(value))
    file_name = data_file_path + data_file_base_name + "_" + str(value) + ".csv"


    with open(file_name, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(['idx', 'est_x', 'est_y', 'est_z', 'est_vx', 'est_vy', 'est_vz', 'est_roll', 'est_pitch', 'est_yaw', 'est_roll_rate', 'est_pitch_rate', 'est_yaw_rate', 'est_f1', 'est_f2', 'est_f3', 'est_f4', 'Optitrack_x', 'Optitrack_y', 'Optitrack_z', 'roll', 'pitch', 'yaw', 'gyro_x', 'gyro_y', 'gyro_z', 'command_m1', 'command_m2', 'command_m3', 'command_m4', 'control_f1', 'control_f2', 'control_f3', 'control_f4'])
    rospy.loginfo("Created file: " + file_name)
    return 


def create_flexible_file(data_names):
    global data_file_path, file_name
    with open(data_file_path+"iteration_num.txt", 'r+') as file:
        # Read the integer value from the file
        value = int(file.read())
        
        # Increment the value by 1
        value += 1
        
        # Go back to the beginning of the file to overwrite the existing value
        file.seek(0)
        
        # Write the updated value back to the file
        file.write(str(value))
    file_name = data_file_path + data_file_base_name + "_" + str(value) + ".csv"

    names = ["data_idx"] + data_names
    with open(file_name, 'w') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(names)
    rospy.loginfo("Created file: " + file_name)
    return 


def log_floaty_info_cb(msg):
    global first_data, file_name
    if first_data: 
        create_file()
        first_data = False

    data_idx = msg.idx

    # ---------------------------------------------
    # Get position from the estimtator
    est_x_floaty = msg.Est_x
    est_y_floaty = msg.Est_y
    est_z_floaty = msg.Est_z

    # ---------------------------------------------
    # Getvelocity from the estimtator
    est_vx_floaty = msg.Est_vx
    est_vy_floaty = msg.Est_vy
    est_vz_floaty = msg.Est_vz
    
    # ---------------------------------------------
    # Get angles from the estimtator
    est_roll_floaty = msg.Est_roll
    est_pitch_floaty = msg.Est_pitch
    est_yaw_floaty = msg.Est_yaw

    # ---------------------------------------------
    # Get angluar rates from the estimtator
    est_roll_rate_floaty = msg.Est_roll_rate
    est_pitch_rate_floaty = msg.Est_pitch_rate
    est_yaw_rate_floaty = msg.Est_yaw_rate
    
    # ---------------------------------------------
    # Get flaps values from the estimtator
    est_f1 = msg.Est_f1
    est_f2 = msg.Est_f2
    est_f3 = msg.Est_f3
    est_f4 = msg.Est_f4

    # ---------------------------------------------
    # Get position and angles the Optitrack
    Optitrack_x = msg.Opt_x
    Optitrack_y = msg.Opt_y
    Optitrack_z = msg.Opt_z
    
    roll = msg.Opt_roll
    pitch = msg.Opt_pitch
    yaw = msg.Opt_yaw

    # ---------------------------------------------
    # Get gyro information
    gyro_x = msg.gyro_x
    gyro_y = msg.gyro_y
    gyro_z = msg.gyro_z

    # ---------------------------------------------
    # Get control values from the estimtator
    command_m1 = msg.Command_m1
    command_m2 = msg.Command_m2
    command_m3 = msg.Command_m3
    command_m4 = msg.Command_m4

    # ---------------------------------------------
    # Get control values from the estimtator
    control_f1 = msg.Control_f1
    control_f2 = msg.Control_f2
    control_f3 = msg.Control_f3
    control_f4 = msg.Control_f4

    new_row = [data_idx, est_x_floaty, est_y_floaty, est_z_floaty, est_vx_floaty, est_vy_floaty, est_vz_floaty, est_roll_floaty, est_pitch_floaty, est_yaw_floaty, est_roll_rate_floaty, est_pitch_rate_floaty, est_yaw_rate_floaty, est_f1, est_f2, est_f3, est_f4, Optitrack_x, Optitrack_y, Optitrack_z, roll, pitch, yaw, gyro_x, gyro_y, gyro_z, command_m1, command_m2, command_m3, command_m4, control_f1, control_f2, control_f3, control_f4]

    with open(file_name, 'a') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(new_row)
    



def flexible_log_floaty_info_cb(msg):
    global first_data, file_name, flexible_msg_data_title
    data_names = msg.names
    if first_data: 
        create_flexible_file(data_names)
        first_data = False

    data_idx = msg.idx
    title = msg.title
    values = msg.values
    data_to_write = [data_idx]
    for val in values:
        data_to_write.append(val)

    if title == flexible_msg_data_title:
        with open(file_name, 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerow(data_to_write)
    


if __name__ == '__main__':
    rospy.init_node("log_floaty_info_node")
    floaty_info_publish_topic_name = "/crazyradio/" + rospy.get_param("/crazyradio/floaty_info_publish_topic_name", "floaty_info")
    floaty_flexible_info_publish_topic_name = "/crazyradio/" + rospy.get_param("/crazyradio/floaty_flexible_info_publish_topic_name", "flexible_floaty_info")
    data_file_path = rospy.get_param("/floaty_info_recorder/data_file_path", "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/iterative_learning_algorithm/floaty_estimator/")
    data_file_base_name = rospy.get_param("/floaty_info_recorder/data_file_base_name", "floaty_data")    
    flexible_msg_data_title = rospy.get_param("/crazyradio/flexible_msg_data_title")

    pos_sub = rospy.Subscriber(floaty_info_publish_topic_name, floaty_info_msg, log_floaty_info_cb)
    pos_sub = rospy.Subscriber(floaty_flexible_info_publish_topic_name, floaty_flexible_msg, flexible_log_floaty_info_cb)

    rospy.loginfo("log_floaty_info node initialized correctly")
    rospy.spin()

