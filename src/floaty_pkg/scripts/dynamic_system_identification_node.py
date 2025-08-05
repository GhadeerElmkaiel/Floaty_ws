import numpy as np
import rospy
from floaty_msgs.srv import speed_update_srv, record_data_srv, record_data_srvRequest
from floaty_msgs.srv import maestro_srv, maestro_srvRequest, int_srv, int_srvRequest
from floaty_msgs.srv import set_cf_param_srv, set_cf_param_srvRequest
from floaty_msgs.srv import set_flaps_angles_srv, set_flaps_angles_srvRequest
from floaty_msgs.srv import dynamic_sys_id_srv
from netft_rdt_driver.srv import String_cmd, String_cmdRequest

import subprocess

rospy.logdebug("Running dynamic sys id node")

pth_to_file = __file__
folders_to_pkg = pth_to_file.split("/")
pth_to_pkg = '/'.join(folders_to_pkg[:-2])
pth_to_recorded_data = '/'.join([pth_to_pkg, 'data/dynamic_system_identification', ''])
# windy_stop_speed = 4000

def remove_ati_bias():
    srv_name = "/ft_sensor/bias_cmd"
    bias_srv = rospy.ServiceProxy(srv_name, String_cmd)

    req = String_cmdRequest()
    req.cmd = 'bias'

    result = bias_srv(req)
    return True


def record_ati_data(file_name, num_of_points):
    ATI_srv_name = "/ATI_recorder/collect_data_service"
    # rospy.wait_for_service(ATI_srv_name)
    try:
        collect_data_srv = rospy.ServiceProxy(ATI_srv_name, record_data_srv)
        
        request = record_data_srvRequest()
        request.data_to_record = int(num_of_points)
        request.file_name = file_name

        result = collect_data_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


def record_vrpn_data(file_name, num_of_points):
    vrpn_srv_name = "/vrpn_recorder/collect_data_service"
    rospy.wait_for_service(vrpn_srv_name)
    try:
        collect_data_srv = rospy.ServiceProxy(vrpn_srv_name, record_data_srv)
        
        request = record_data_srvRequest()
        request.data_to_record = int(num_of_points)
        request.file_name = file_name

        result = collect_data_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


def record_data(file_name, num_of_points):
    vrpn_srv_name = "/topic_recorder/collect_data_service"
    rospy.wait_for_service(vrpn_srv_name)
    try:
        collect_data_srv = rospy.ServiceProxy(vrpn_srv_name, record_data_srv)
        
        request = record_data_srvRequest()
        request.data_to_record = int(num_of_points)
        request.file_name = file_name

        result = collect_data_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False
    

    

def ramp_windy_motors(speed):
    ramp_windy_motors = "/maestro_control/ramp_motors_same_speed_service"
    rospy.wait_for_service(ramp_windy_motors)
    try:
        ramp_motor_srv = rospy.ServiceProxy(ramp_windy_motors, int_srv)
        
        request = int_srvRequest()
        request.data = speed

        result = ramp_motor_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Maestro ramp speeds service call failed: %s"%e)
        return False

# Move the motors
def move_flaps(flaps_to_move):
    radio_srv_name =  "/crazyradio/set_parameters_value"
    rospy.wait_for_service(radio_srv_name)
    try:
        move_motor_srv = rospy.ServiceProxy(radio_srv_name, set_cf_param_srv)
        
        for flap in flaps_to_move:
            request = set_cf_param_srvRequest()

            request.group_name = "extModeControl"
            request.param_name = "ctrType{}".format(int(flap))
            request.value = "2"
            rospy.loginfo(request.param_name)   
            rospy.loginfo(request.group_name)   
            rospy.loginfo(request.value)   

            result = move_motor_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


# Move the motors
def flaps_default_angle(angles):
    radio_srv_name =  "/crazyradio/set_flaps_angles"
    rospy.wait_for_service(radio_srv_name)
    try:
        move_motor_srv = rospy.ServiceProxy(radio_srv_name, set_flaps_angles_srv)
        
        request = set_flaps_angles_srvRequest()
        request.angles = angles
        result = move_motor_srv(request)

        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def dynamic_system_identification_callback(req):

    windy_motor_speed = rospy.get_param("/maestro_control/windy_motor_speed")
    windy_stop_speed = rospy.get_param("/maestro_control/windy_stop_speed")
    record_duration = rospy.get_param("/ft_sensor/record_duration")
    use_ati_data_directly = rospy.get_param("/ft_sensor/use_ati_data_directly")
    ati_rate = rospy.get_param("/ATI_recorder/ATI_record_rate")

    radio_srv_name =  "/crazyradio/set_parameters_value"
    rospy.wait_for_service(radio_srv_name)

    number_of_periods = req.number_of_periods
    default_angles = req.default_angles
    motors_to_move = req.motors_to_move


    # calculate the number of points to record
    if use_ati_data_directly:
        num_of_data_points_to_record = 10*number_of_periods*ati_rate
    else:
        num_of_data_points_to_record = 10*number_of_periods/record_duration

    str_num_points = str(num_of_data_points_to_record)
    rospy.loginfo("num_of_data_points_to_record: "+str_num_points)   

    # Extra safty measurement to ensure that the motors are not rotating
    ramp_windy_motors(windy_stop_speed)
    # Wait until fully stopped
    rospy.sleep(1.5)

    # Reset the ati bias
    remove_ati_bias()
    rospy.sleep(0.8)

    flaps_default_angle(default_angles)

    # Start Windy
    ramp_windy_motors(windy_motor_speed)
    rospy.sleep(0.5)


    file_name = "dynamic_sys_id"
    for id in motors_to_move:
        file_name = file_name+"_{}".format(int(id))
    file_name = file_name+".csv"

    rospy.loginfo("Moving motors")   
    move_flaps(motors_to_move)
    rospy.loginfo("Start recording data")   
    res = record_ati_data(file_name, num_of_data_points_to_record)
    rospy.loginfo("End recording data")
    if res:
        rospy.loginfo("Recording data confirmed")
    else:
        rospy.logerr("Error in recording data!")

    # Return flaps to default angles
    flaps_default_angle(default_angles)

    # Stop Windy's motors at the end of experiment
    ramp_windy_motors(windy_stop_speed)

    # Wait until fully stopped
    rospy.sleep(2.5)

    # Reset the ati bias
    remove_ati_bias()



def dynamic_system_identification_vrpn_callback(req):

    windy_motor_speed = rospy.get_param("/maestro_control/windy_motor_speed")
    windy_stop_speed = rospy.get_param("/maestro_control/windy_stop_speed")
    record_duration = rospy.get_param("/vrpn_recorder/record_duration")
    use_ati_data_directly = rospy.get_param("/vrpn_recorder/use_data_directly")
    vrpn_rate = rospy.get_param("/vrpn_recorder/vrpn_rate")

    radio_srv_name =  "/crazyradio/set_parameters_value"
    rospy.wait_for_service(radio_srv_name)

    number_of_periods = req.number_of_periods
    default_angles = req.default_angles
    motors_to_move = req.motors_to_move


    # calculate the number of points to record
    if use_ati_data_directly:
        num_of_data_points_to_record = 10*number_of_periods*vrpn_rate
    else:
        num_of_data_points_to_record = 10*number_of_periods/record_duration

    str_num_points = str(num_of_data_points_to_record)
    rospy.loginfo("num_of_data_points_to_record: "+str_num_points)   

    # Extra safty measurement to ensure that the motors are not rotating
    ramp_windy_motors(windy_stop_speed)
    # Wait until fully stopped
    rospy.sleep(1.5)

    # Reset the ati bias
    remove_ati_bias()
    rospy.sleep(0.8)

    flaps_default_angle(default_angles)

    # Start Windy
    ramp_windy_motors(windy_motor_speed)
    rospy.sleep(0.5)


    file_name = "dynamic_sys_id"
    for id in motors_to_move:
        file_name = file_name+"_{}".format(int(id))
    file_name = file_name+".csv"

    rospy.loginfo("Moving motors")   
    move_flaps(motors_to_move)
    rospy.loginfo("Start recording data")   
    res = record_vrpn_data(file_name, num_of_data_points_to_record)
    rospy.loginfo("End recording data")
    if res:
        rospy.loginfo("Recording data confirmed")
    else:
        rospy.logerr("Error in recording data!")

    # Return flaps to default angles
    flaps_default_angle(default_angles)

    # Stop Windy's motors at the end of experiment
    ramp_windy_motors(windy_stop_speed)

    # Wait until fully stopped
    rospy.sleep(2.5)

    # Reset the ati bias
    remove_ati_bias()


def dynamic_system_identification_NatNet_callback(req):

    windy_motor_speed = rospy.get_param("/maestro_control/windy_motor_speed")
    windy_stop_speed = rospy.get_param("/maestro_control/windy_stop_speed")
    record_duration = rospy.get_param("/topic_recorder/record_duration")
    use_ati_data_directly = rospy.get_param("/topic_recorder/use_data_directly")
    vrpn_rate = rospy.get_param("/NatNet/default_rate")

    radio_srv_name =  "/crazyradio/set_parameters_value"
    rospy.wait_for_service(radio_srv_name)

    number_of_periods = req.number_of_periods
    default_angles = req.default_angles
    motors_to_move = req.motors_to_move


    # calculate the number of points to record
    if use_ati_data_directly:
        num_of_data_points_to_record = 10*number_of_periods*vrpn_rate
    else:
        num_of_data_points_to_record = 10*number_of_periods/record_duration

    str_num_points = str(num_of_data_points_to_record)
    rospy.loginfo("num_of_data_points_to_record: "+str_num_points)   

    # Extra safty measurement to ensure that the motors are not rotating
    ramp_windy_motors(windy_stop_speed)
    # Wait until fully stopped
    rospy.sleep(1.5)

    # Reset the ati bias
    remove_ati_bias()
    rospy.sleep(0.8)

    flaps_default_angle(default_angles)

    # Start Windy
    ramp_windy_motors(windy_motor_speed)
    rospy.sleep(0.5)


    file_name = "dynamic_sys_id"
    for id in motors_to_move:
        file_name = file_name+"_{}".format(int(id))
    file_name = file_name+".csv"

    rospy.loginfo("Moving motors")   
    move_flaps(motors_to_move)
    rospy.loginfo("Start recording data")   
    # res = record_vrpn_data(file_name, num_of_data_points_to_record)
    res = record_data(file_name, num_of_data_points_to_record)
    rospy.loginfo("End recording data")
    if res:
        rospy.loginfo("Recording data confirmed")
    else:
        rospy.logerr("Error in recording data!")

    # Return flaps to default angles
    flaps_default_angle(default_angles)

    # Stop Windy's motors at the end of experiment
    ramp_windy_motors(windy_stop_speed)

    # Wait until fully stopped
    rospy.sleep(2.5)

    # Reset the ati bias
    remove_ati_bias()


def system_identification():
    
    rospy.init_node("dynamic_system_identification_node")
    srvice = rospy.Service('dynamic_system_identification_srv', dynamic_sys_id_srv, dynamic_system_identification_callback)
    srvice = rospy.Service('dynamic_system_identification_vrpn_srv', dynamic_sys_id_srv, dynamic_system_identification_vrpn_callback)
    srvice = rospy.Service('dynamic_system_identification_NatNet_srv', dynamic_sys_id_srv, dynamic_system_identification_NatNet_callback)
    print("dynamic system identification service ready!")

    # Wait for maestro service to be ready
    update_speeds_srv_name = "/maestro_control/move_motors_service"
    rospy.wait_for_service(update_speeds_srv_name)
    ramp_speeds_srv_name = "/maestro_control/ramp_motors_same_speed_service"
    rospy.wait_for_service(ramp_speeds_srv_name)
    # # Wait for cf radio service to be ready

    radio_srv_name = "/crazyradio/set_parameters_value"
    rospy.wait_for_service(radio_srv_name)
    # Wait for ATI data recording service to be ready
    ATI_srv_name = "/ATI_recorder/collect_data_service"
    rospy.wait_for_service(ATI_srv_name)

    Sendor_bias_srv = "/ft_sensor/bias_cmd"
    rospy.wait_for_service(Sendor_bias_srv)


    # Initialize windy by stopping the motors
    windy_stop_speed = rospy.get_param("/maestro_control/windy_stop_speed")
    ramp_windy_motors(windy_stop_speed)
    
    rospy.sleep(1)

    def_angles = rospy.get_param("/floaty/default_angles")
    flaps_default_angle(def_angles)
    rospy.sleep(0.5)


    # Reset the ati bias
    remove_ati_bias()
    print("Initial bias removal")

    rospy.spin()


if __name__ == '__main__':
    rospy.logdebug("Running dynamic sys id node")
    try:
        system_identification()
    except rospy.ROSInterruptException:
        pass
