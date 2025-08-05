import numpy as np
import rospy
from floaty_msgs.srv import string_srv
from floaty_msgs.srv import speed_update_srv, record_data_srv, record_data_srvRequest
from floaty_msgs.srv import maestro_srv, maestro_srvRequest
from floaty_msgs.srv import set_flaps_angles_srv, set_flaps_angles_srvRequest
from floaty_msgs.srv import static_sys_id_srv
import os

pth_to_file = __file__
folders_to_pkg = pth_to_file.split("/")
pth_to_pkg = '/'.join(folders_to_pkg[:-2])
pth_to_recorded_data = '/'.join([pth_to_pkg, 'data/static_system_identification', ''])


def record_ati_data(file_name, num_of_points):
    ATI_srv_name = "/ATI_recorder/collect_data_service"
    rospy.wait_for_service(ATI_srv_name)
    try:
        collect_data_srv = rospy.ServiceProxy(ATI_srv_name, record_data_srv)
        
        request = record_data_srvRequest()
        request.data_to_record = num_of_points
        request.file_name = file_name

        result = collect_data_srv(request)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Move the motors
def move_flaps(flaps_angles):
    radio_srv_name =  "/cf_radio/set_flaps_angles"
    rospy.wait_for_service(radio_srv_name)
    try:
        move_motor_srv = rospy.ServiceProxy(radio_srv_name, record_data_srv)
        
        request = set_flaps_angles_srvRequest()
        request.angles = flaps_angles

        result = move_motor_srv(request)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def static_system_identification_callback(req):
    ns = rospy.get_namespace()
    data_path = rospy.get_param(ns+"/data_file_path")

    number_of_measurements = req.number_of_measurements
    default_angles = req.default_angles
    min_angle = req.min_angle
    max_angle = req.max_angle
    angle_resolution = req.angle_resolution
    motors_to_move_direction = req.motors_to_move_direction

    num_of_angles_to_test = int((max_angle-min_angle)/angle_resolution)
    for i in range(num_of_angles_to_test + 1):
        change_in_angle = min_angle + i*angle_resolution
        flaps_angles = [0, 0, 0, 0]
        for j in range(4):
            flap_angle_change = change_in_angle*motors_to_move_direction
            flaps_angles[j] = default_angles[j]+flap_angle_change

        rospy.loginfo("Moving motors")   
        move_flaps(flaps_angles)
        rospy.sleep(1)
        rospy.loginfo("Start recording data")   
        file_name = data_path+"data_with_angles_{m1}_{m2}_{m3}_{m4}.csv".format(int(flaps_angles[0]), int(flaps_angles[1]), int(flaps_angles[2]), int(flaps_angles[3]))
        x = record_ati_data(file_name, number_of_measurements)
        rospy.loginfo("End recording data")



def system_identification():
    
    # Wait for maestro service to be ready
    update_speeds_srv_name = "/maestro_control/move_motors_service"
    rospy.wait_for_service(update_speeds_srv_name)
    # Wait for cf radio service to be ready
    radio_srv_name = "/cf_radio/set_flaps_angles"
    rospy.wait_for_service(radio_srv_name)
    # Wait for ATI data recording service to be ready
    ATI_srv_name = "/ATI_recorder/collect_data_service"
    rospy.wait_for_service(ATI_srv_name)

    
    rospy.init_node("static_system_identification_node")
    srvice = rospy.Service('static_system_identification_srv', static_sys_id_srv, static_system_identification_callback)
    print("Static system identification service ready!")

    rospy.spin()


if __name__ == '__main__':
    try:
        system_identification()
    except rospy.ROSInterruptException:
        pass