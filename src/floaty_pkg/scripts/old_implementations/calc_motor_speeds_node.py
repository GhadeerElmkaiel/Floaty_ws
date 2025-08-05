import numpy as np
import rospy
from floaty_msgs.srv import string_srv
from floaty_msgs.srv import speed_update_srv, record_data_srv, record_data_srvRequest
from floaty_msgs.srv import maestro_srv, maestro_srvRequest
# from floaty_msgs.srv import iter_update_srv
import os

# pth_to_file = __file__
# folders_to_pkg = pth_to_file.split("/")
# pth_to_pkg = '/'.join(folders_to_pkg[:-2])
# pth_to_sounds = '/'.join([pth_to_pkg, 'sounds', ''])
pth_to_recorded_data = '/'


def start_gp_calculation(file_name, speeds):
    gp_srv_name = "/gp_process/calculate_gp_from_data"
    rospy.wait_for_service(gp_srv_name)
    try:
        gp_srv = rospy.ServiceProxy(gp_srv_name, speed_update_srv)
        result = gp_srv(file_name, speeds)
        speeds = []

        limit = 50
        for i in range(len(result.updated_speeds)):
            if result.updated_speeds[i]>limit:
                speeds.append(limit)
            else:
                speeds.append(result.updated_speeds[i])

        return speeds
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def calc_motor_speeds_callback(req):
    pth_to_recorded_data = req.data

    # delay to get ready
    # rospy.sleep(7)

    # ns = rospy.get_namespace()
    # alpha = rospy.get_param("/iterative_learning/update_rate_decay")
    max_iter_num = rospy.get_param("/iterative_learning/max_number_of_iterations")
    original_update_rate = rospy.get_param("/gp_process/motors_speed_update_rate")

    old_speeds = [50, 50, 50, 50, 50, 50]
    for i in range(max_iter_num):

        file_name = pth_to_recorded_data+"iterative_learning_data_num_"+str(i)+".csv"
        print(old_speeds)
        new_speeds = start_gp_calculation(file_name, old_speeds)

        new_update_rate = original_update_rate/(1+1.3*i)

        if new_update_rate<0.05:
            new_update_rate = 0.05
        rospy.set_param("/gp_process/motors_speed_update_rate", new_update_rate)

        # Update the speeds

        old_speeds = new_speeds

    return 1



def iterative_update_service():
    rospy.init_node("calc_motor_speeds_node")
    srvice = rospy.Service('calc_motor_speeds_srv', string_srv, calc_motor_speeds_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        iterative_update_service()
    except rospy.ROSInterruptException:
        pass