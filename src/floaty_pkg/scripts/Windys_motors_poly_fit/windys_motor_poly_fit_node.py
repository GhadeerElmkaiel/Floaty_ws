import rospy
from floaty_msgs.srv import set_flaps_angles_srvRequest, set_flaps_angles_srv
from floaty_msgs.srv import record_data_srv, record_data_srvRequest
from floaty_msgs.srv import maestro_srv, maestro_srvRequest
from floaty_msgs.srv import int_srv, motor_thrust_measure_srv
from netft_rdt_driver.srv import String_cmd, String_cmdRequest
from time import sleep


def remove_ati_bias():
    srv_name = "/ft_sensor/bias_cmd"
    bias_srv = rospy.ServiceProxy(srv_name, String_cmd)

    req = String_cmdRequest()
    req.cmd = 'bias'

    result = bias_srv(req)
    return True


def update_speeds(new_speeds):
    update_speeds_srv_name = "/maestro_control/ramp_motors_service"
    # rospy.wait_for_service(update_speeds_srv_name)
    try:
        update_speeds_srv = rospy.ServiceProxy(update_speeds_srv_name, maestro_srv)
        
        # TODO
        # I need to change the speeds to use percent 0-100 instead of the range 4000-8000
        # new_speeds = [int(4000 + 40*new_speed_percent) for new_speed_percent  in new_speeds_percent]
        request = maestro_srvRequest()
        request.speeds = new_speeds
        print(new_speeds)
        result = update_speeds_srv(request)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


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


# # Move the motors
# def flaps_default_angle(angles):
#     radio_srv_name =  "/crazyradio/set_flaps_angles"
#     rospy.wait_for_service(radio_srv_name)
#     try:
#         move_motor_srv = rospy.ServiceProxy(radio_srv_name, set_flaps_angles_srv)
        
#         request = set_flaps_angles_srvRequest()
#         request.angles = angles
#         result = move_motor_srv(request)

#         return result
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)


def collect_and_fit_callback(req):
    path_to_data = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/upper/"
    update_speeds_srv_name = "/maestro_control/ramp_motors_service"
    rospy.wait_for_service(update_speeds_srv_name)
    ATI_srv_name = "/ATI_recorder/collect_data_service"
    rospy.wait_for_service(ATI_srv_name)
    print("Setting flaps angles")
    Sendor_bias_srv = "/ft_sensor/bias_cmd"
    rospy.wait_for_service(Sendor_bias_srv)

    number_of_motors = rospy.get_param("/maestro_control/number_of_motors")
    # flaps_default_angle([60, 60, 60, 60])
    motor_id = req.motor_id
    start_speed = req.start_speed
    end_speed = req.end_speed
    increment = req.increment
    measurments_per_speed = req.measurments_per_speed

    new_speed = [0 for _ in range(number_of_motors)]
    update_speeds(new_speed)

    rospy.sleep(1)

    # Reset the ati bias
    remove_ati_bias()
    rospy.sleep(0.8)
    

    for speed in range(start_speed, end_speed, increment):
        print("Setting speed to: ", speed)
        new_speed = [0 for _ in range(number_of_motors)]
        new_speed[motor_id] = speed
        update_speeds(new_speed)
        sleep(2)
        print("Recording data")
        if not record_ati_data(path_to_data + str(speed) + ".csv", measurments_per_speed):
            print("Failed to record data")
            return False
        print("Done recording data")
    
    print("Stop motor")
    # Stop the wind tunnel motors
    new_speed = [0 for _ in range(number_of_motors)]
    update_speeds(new_speed)

    rospy.sleep(1)

    return True


if __name__ == '__main__':
    # Initialize the low-level drivers
    try:
        rospy.init_node("windys_motor_poly_fit_node")

        collect_and_fit_servoce = rospy.Service('collect_data_and_fit_motor_poly', motor_thrust_measure_srv, collect_and_fit_callback)
    except rospy.ROSInterruptException:
        rospy.logerr("Error in radio code")

    rospy.spin()
