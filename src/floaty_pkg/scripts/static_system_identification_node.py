import rospy
from floaty_msgs.srv import record_data_srv, record_data_srvRequest
from floaty_msgs.srv import int_srv, int_srvRequest
from floaty_msgs.srv import set_flaps_angles_srvRequest
from floaty_msgs.srv import static_sys_id_srv
from floaty_msgs.srv import empty_srv, empty_srvRequest
from floaty_msgs.srv import maestro_srvRequest, maestro_srv
from netft_rdt_driver.srv import String_cmd, String_cmdRequest

import subprocess


pth_to_file = __file__
folders_to_pkg = pth_to_file.split("/")
pth_to_pkg = '/'.join(folders_to_pkg[:-2])
# pth_to_recorded_data = '/'.join([pth_to_pkg, 'data/static_system_identification', ''])
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
    rospy.wait_for_service(ATI_srv_name)
    try:
        collect_data_srv = rospy.ServiceProxy(ATI_srv_name, record_data_srv)
        
        request = record_data_srvRequest()
        request.data_to_record = num_of_points
        # request.file_name = pth_to_recorded_data+file_name
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
 
def ramp_windy_motors_flying_speeds():
    motors = [51, 61, 46, 52, 47, 42, 52]  
    ramp_windy_motors = "/maestro_control/ramp_motors_service"
    rospy.wait_for_service(ramp_windy_motors)
    try:
        ramp_motor_srv = rospy.ServiceProxy(ramp_windy_motors, maestro_srv)
        
        request = maestro_srvRequest()
        request.speeds = motors

        result = ramp_motor_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Maestro ramp speeds service call failed: %s"%e)
        return False
    

def stop_windy_motors():
    stop_windy_motors = "/maestro_control/stop_motors_service"
    rospy.wait_for_service(stop_windy_motors)
    try:
        stop_motor_srv = rospy.ServiceProxy(stop_windy_motors, empty_srv)
        
        request = empty_srvRequest()

        result = stop_motor_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Maestro motors stopped")
        return False

# Move the motors
def move_flaps(flaps_angles):
    radio_srv_name =  "/cf_radio/set_flaps_angles"
    rospy.wait_for_service(radio_srv_name)
    try:
        move_motor_srv = rospy.ServiceProxy(radio_srv_name, record_data_srv)
        
        request = set_flaps_angles_srvRequest()
        request.angles = flaps_angles

        result = move_motor_srv(request)
        return result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# Move the motors with python script
def move_flaps_script(flaps_angles):
    ns = rospy.get_namespace()
    path_to_script = rospy.get_param(ns+"pth_to_radio_script")
    cmd_edited = "python3 "+ path_to_script
    for i, ang in enumerate(flaps_angles):
        id = i+1
        angle_command = ang
        if id > 4 or angle_command>60 or angle_command<-60:
            rospy.loginfo("Incorrect request to send motor {} to angle {}".format(id, angle_command))
            continue
        cmd_edited = cmd_edited + " " + str(angle_command)
    rospy.loginfo(cmd_edited)
    p = subprocess.Popen(cmd_edited, stdout = subprocess.PIPE, shell = True)
    rospy.sleep(0.1)


def static_system_identification_callback(req):

    # Start Windy
    ns = rospy.get_namespace()
    windy_motor_speed = rospy.get_param(ns+"windy_motor_speed")
    windy_stop_speed = rospy.get_param(ns+"windy_stop_speed")

    number_of_measurements = req.number_of_measurements
    default_angles = req.default_angles
    min_angle = req.min_angle
    max_angle = req.max_angle
    angle_resolution = req.angle_resolution
    motors_to_move_direction = req.motors_to_move_direction

    # Extra safty measurement to ensure that the motors are not rotating
    ramp_windy_motors(windy_stop_speed)
    stop_windy_motors()
    # Wait until fully stopped
    rospy.sleep(2.5)

    # Reset the ati bias
    remove_ati_bias()
    rospy.sleep(1.5)

    
    # ramp_windy_motors(windy_motor_speed)
    ramp_windy_motors_flying_speeds()
    rospy.sleep(2)

    num_of_angles_to_test = int((max_angle-min_angle)/angle_resolution)
    for i in range(num_of_angles_to_test + 1):
        change_in_angle = min_angle + i*angle_resolution
        flaps_angles = [0, 0, 0, 0]
        for j in range(4):
            flap_angle_change = change_in_angle*motors_to_move_direction[j]
            flaps_angles[j] = default_angles[j]+flap_angle_change

        
        rospy.loginfo("Test configuration {}".format(i+1))
        rospy.loginfo("Flaps angles: [{}, {}, {}, {}]".format(flaps_angles[0], flaps_angles[1], flaps_angles[2], flaps_angles[3]))   

        rospy.loginfo("Moving motors")   
        move_flaps_script(flaps_angles)
        # Wait for flaps to move
        rospy.sleep(3)
        rospy.loginfo("Start recording data")   
        path_to_file = rospy.get_param("/ATI_recorder/data_file_path")
        file_name = path_to_file + "data_with_angles_{}_{}_{}_{}.csv".format(int(flaps_angles[0]+60), int(flaps_angles[1]+60), int(flaps_angles[2]+60), int(flaps_angles[3]+60))
        res = record_ati_data(file_name, number_of_measurements)
        rospy.loginfo("End recording data")
        rospy.loginfo(file_name)
        if res:
            rospy.loginfo("Recording data confirmed")
        else:
            rospy.logerr("Error in recording data!")

    # Stop Windy's motors at the end of experiment
    ramp_windy_motors(windy_stop_speed)
    stop_windy_motors()
    # Wait until fully stopped
    rospy.sleep(2.5)

    # Reset the ati bias
    remove_ati_bias()

def system_identification():
    
    # Wait for maestro service to be ready
    update_speeds_srv_name = "/maestro_control/move_motors_service"
    rospy.wait_for_service(update_speeds_srv_name)
    ramp_speeds_srv_name = "/maestro_control/ramp_motors_same_speed_service"
    rospy.wait_for_service(ramp_speeds_srv_name)
    # # Wait for cf radio service to be ready
    # radio_srv_name = "/cf_radio/set_flaps_angles"
    # rospy.wait_for_service(radio_srv_name)
    # Wait for ATI data recording service to be ready
    ATI_srv_name = "/ATI_recorder/collect_data_service"
    rospy.wait_for_service(ATI_srv_name)

    Sendor_bias_srv = "/ft_sensor/bias_cmd"
    rospy.wait_for_service(Sendor_bias_srv)

    

    # Initialize windy by stopping the motors
    ns = rospy.get_namespace()
    windy_stop_speed = rospy.get_param(ns+"windy_stop_speed")
    stop_windy_motors()

    # Reset the ati bias
    rospy.sleep(1)
    remove_ati_bias()
    
    rospy.init_node("static_system_identification_node")
    srvice = rospy.Service('static_system_identification_srv', static_sys_id_srv, static_system_identification_callback)
    print("Static system identification service ready!")

    rospy.spin()


if __name__ == '__main__':
    try:
        system_identification()
    except rospy.ROSInterruptException:
        pass