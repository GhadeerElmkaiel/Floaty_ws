import rospy
from floaty_msgs.srv import empty_srvRequest, int_srvRequest, set_cf_param_srvRequest, maestro_srvRequest
from floaty_msgs.srv import empty_srv, int_srv, set_cf_param_srv, maestro_srv

import sys

send_position_srv_name="send_position"


wind_tunnel_speed = 66
wait_time_before_running_controller = 0.3 # 0.5
run_time = 30.0
# run_time = 60.0
landing_time = 8
takeoff_time = 6
# takeoff_time = 600
pushup_time = 1.3
delay = 10
hover_angle = 0
angle_counter = 0
steps_to_stop = 1
time_between_stop_steps = 0.3

# For 6 motos
# motors = [46, 45, 48, 47, 44, 42]
# motors = [48, 47, 50, 49, 46, 44]
# motors = [50, 49, 52, 51, 48, 46] # Good for Manual controller 5 with floaty at hight 50-70 cm
# motors = [51, 50, 53, 52, 49, 47]
# motors = [53, 52, 55, 54, 51, 49]
# motors = [55, 54, 57, 56, 53, 51]
# motors = [60, 59, 62, 61, 58, 56]
# motors = [63, 62, 65, 64, 61, 59] # Optimized via online learning algorithm
# motors = [66, 65, 68, 67, 64, 62] 
# motors = [68, 67, 70, 69, 66, 64] 


# For 7 motors
# motors = [35, 42, 47, 55, 48, 49, 55]
# motors = [46, 45, 48, 47, 44, 42, 60]   # Good value with a bit hiegher speed over hexagon
# motors = [48, 47, 50, 49, 46, 44, 65]


# motors = [48, 53, 48, 49, 42, 44, 72]   # The speeds used for first position control
# motors = [44, 51, 44, 45, 38, 40, 70]   
# motors = [44, 54, 44, 47, 38, 40, 70]   
# motors_low = [38, 47, 38, 41, 36, 37, 36]   
# motors_low = [40, 49, 40, 43, 38, 39, 40]   
# motors_low = [45, 53, 45, 47, 40, 40, 55]   
# motors = [54, 57, 54, 55, 48, 50, 72]
# motors = [41, 40, 43, 42, 39, 37, 62]

# Floaty V2
# Floaty V3
# Good but 1 and 4 seems to be low
motors_low = [56, 57, 53, 55, 50, 50, 55]   
motors = [56, 57, 53, 55, 50, 50, 55]   
# Testing
motors = [56, 59, 51, 57, 50, 47, 55]   
# Add more speed for 
motors = [53, 61, 48, 54, 47, 44, 52]   

# # Best speeds for flying with V3 with new setup with counter weight
motors = [51, 61, 46, 52, 47, 42, 52]   

# # Test with higher M2 speed
motors = [53, 66, 48, 54, 50, 44, 54]   

# # Z tracking speeds
# motors = [54, 64, 49, 55, 50, 45, 60]   

# # Y tracking speeds
# motors = [51, 61, 46, 52, 47, 42, 52]   

# motors = [31, 31, 36, 32, 37, 32, 32]   

# # Add more speed for 
# motors = [54, 64, 49, 55, 50, 45, 62]   
# motors = [30, 30, 30, 30, 30, 30, 30]

# motors = [53, 53, 53, 53, 53, 53, 53]   


# Floaty V4
motors = [55, 68, 50, 56, 53, 47, 56]   

landing_motors = [41,51, 38, 42, 39, 36, 43]
takeoff_motors = [41,51, 38, 42, 39, 36, 43]

# With counter weight
# landing_motors = [39,49, 36, 40, 37, 34, 42]
# landing_motors = [38,47, 36, 38, 36, 34, 40]   # less

pushup_motors = [speed+5 for speed in motors]  

# --------------------------------------
# # No extre legs + weight
# motors = [47, 51, 47, 48, 42, 44, 54]
# Plus legs + weight
# motors = [53, 54, 50, 52, 47, 47, 55]   

wind_tunnel_speed_srv = None
wind_tunnel_diff_speeds_srv = None
wind_tunnel_stop_srv = None
set_parameters_value_srv = None


def set_wind_tunnel_speed():
    global wind_tunnel_speed, wind_tunnel_speed_srv
    request = int_srvRequest()
    request.data = wind_tunnel_speed
    wind_tunnel_speed_srv(request)


def set_wind_tunnel_diff_speeds(motors):
    global wind_tunnel_diff_speeds_srv
    request = maestro_srvRequest()
    request.speeds = motors
    wind_tunnel_diff_speeds_srv(request)

def stop_wind_tunnel():
    global wind_tunnel_stop_srv        
    request = empty_srvRequest()
    wind_tunnel_stop_srv(request)
    return

def gradual_stop_wind_tunnel():
    global wind_tunnel_speed_srv
    request = int_srvRequest()
    request.data = 0
    wind_tunnel_speed_srv(request)
    return


    # global wind_tunnel_stop_srv
    # request = maestro_srvRequest()
    # for i in range(steps_to_stop):
    #     ratio = 1-(i+1)/(steps_to_stop+1)
    #     speeds = [int(x*ratio) for x in motors]
    #     request.speeds = speeds
    #     wind_tunnel_diff_speeds_srv(request)
    #     rospy.sleep(time_between_stop_steps)
        
    # request = empty_srvRequest()
    # wind_tunnel_stop_srv(request)
    # return


def set_control_parameter(value):
    global set_parameters_value_srv
    request = set_cf_param_srvRequest()
    request.group_name = "extCtrl"
    request.param_name = "manual"
    request.value = value
    set_parameters_value_srv(request)
    return


def set_general_parameter(group, name, value):
    global set_parameters_value_srv
    request = set_cf_param_srvRequest()
    request.group_name = group
    request.param_name = name
    request.value = value
    set_parameters_value_srv(request)
    return



if __name__ == '__main__':
    # global send_position_srv_name
    # optitrack_topic = rospy.get_param('/NatNet/data_topic', "/Optitrack/Floaty")
    rospy.init_node("experiment_control_node")
    rospy.loginfo("Waiting for needed servecies")

    # Wait for crazyradio service
    set_parameters_value_srv_name = "/crazyradio/" + rospy.get_param("crazyradio/set_parameters_value_srv_name", "set_parameters_value")
    rospy.wait_for_service(set_parameters_value_srv_name)
    set_parameters_value_srv = rospy.ServiceProxy(set_parameters_value_srv_name, set_cf_param_srv)
    rospy.loginfo("crazyradio service ready!")

    # Wait for maestro services
    ramp_motors_same_speed_srv_name = "/maestro_control/ramp_motors_same_speed_service"
    ramp_motors_diff_speeds_srv_name = "/maestro_control/ramp_motors_service"
    stop_motors_srv_name = "/maestro_control/stop_motors_service"
    rospy.wait_for_service(ramp_motors_same_speed_srv_name)
    rospy.wait_for_service(ramp_motors_diff_speeds_srv_name)
    rospy.wait_for_service(stop_motors_srv_name)
    wind_tunnel_speed_srv = rospy.ServiceProxy(ramp_motors_same_speed_srv_name, int_srv)
    wind_tunnel_diff_speeds_srv = rospy.ServiceProxy(ramp_motors_diff_speeds_srv_name, maestro_srv)
    wind_tunnel_stop_srv = rospy.ServiceProxy(stop_motors_srv_name, empty_srv)
    rospy.loginfo("maestro services ready!")

    # For safty, turnoff the wind tunnel
    stop_wind_tunnel()

    rospy.sleep(0.1)

    set_control_parameter("2")

    rospy.loginfo("=======================")
    rospy.loginfo("Experiment node started")
    rospy.loginfo("=======================")

    while(True):
        print("=========================================================")
        print("Wind tunnel speed: {}".format(wind_tunnel_speed))
        print("Motors speeds: {}, {}, {}, {}, {}, {}, {}".format(motors[0], motors[1], motors[2], motors[3], motors[4], motors[5], motors[6]))
        print("Wait time before running controller: {}".format(wait_time_before_running_controller))
        print("Auto-exp Run-time: {}".format(run_time))
        print("Auto-exp Delay: {}".format(delay))
        print("current hover angle: {}".format(hover_angle))
        print("=========================================================")
        print("Choose an option:")
        print("0- Exit")
        print("1- Start wind tunnel then control")
        print("2- Stop experiment")
        print("3- Run experiment after delay for run-time then stop")
        print("4- Run take-off flying landing protocol")
        print("5- Run flying then landing")
        print("6- Run take-off and landing only")
        print("7- Start wind tunnel alone")
        print("8- Start controller alone")
        print("a- Less Steep configuration")
        print("----------")

        print("S- Change wind tunnel speed")
        print("T- Change wait time before running controller")
        print("R- Change run time for the automatic experiment")
        print("D- Change delay time before the automatic experiment")
        print("M- Change wind tunnel motors speeds")
        print("L- Lower wind tunnel motors speeds")
        # print("Y1- Yaw configuration small difference")
        # print("Y2- Yaw configuration large difference")
        print("XP- P control param for X pos")
        print("YP- P control param for Y pos")
        print("ZP- P control param for Z pos")
        print("VXP- P control param for X vel")
        print("VYP- P control param for Y vel")
        print("VZP- P control param for Z vel")
        print("RP- P control param for roll")
        print("PP- P control param for pitch")
        print("yP- P control param for yaw")
        print("RRP- P control param for roll rate")
        print("PRP- P control param for pitch rate")
        print("YRP- P control param for yaw rate")

        # print("St- stability test")
        # print("'+/='- increase hovering angle")
        # print("'-'- decrease hovering angle")
        print("=========================================================")
        option = input("")
        if option == "0":
            sys.exit()
            break

        elif option == "1":
            set_wind_tunnel_speed()
            rospy.sleep(wait_time_before_running_controller)
            set_control_parameter("0")

        elif option == "2":
            stop_wind_tunnel()
            set_control_parameter("2")

        elif option == "3":
            stop_wind_tunnel()
            # set_control_parameter("12")
            # Delay before running the experiment
            rospy.sleep(delay)

            # Run the experiment
            set_wind_tunnel_speed()
            rospy.sleep(wait_time_before_running_controller)
            set_control_parameter("0")

            # Wait for the run-time
            rospy.sleep(run_time)

            # Stop experiment
            gradual_stop_wind_tunnel()
            set_control_parameter("2")

            
        elif option == "4":
            stop_wind_tunnel()
            # # ===== Function description =====
            # - Wait delay
            # - Start taking off speed
            # - Run controller
            # - Wait takeoff_time
            # - Start flying speed
            # - Wait run_time
            # - Start landing speed
            # - landing_time
            # - Stop controller

            rospy.sleep(delay)

            # Start taking-off wind tunnel speed
            set_wind_tunnel_diff_speeds(takeoff_motors)
            # Start controller
            rospy.sleep(wait_time_before_running_controller)
            set_control_parameter("0")
            rospy.sleep(takeoff_time)

            # # Start flying wind tunnel speed
            # set_wind_tunnel_diff_speeds(pushup_motors)
            # # Wait for the run-time
            # rospy.sleep(pushup_time)

            # Start flying wind tunnel speed
            set_wind_tunnel_diff_speeds(motors)
            # Wait for the run-time
            rospy.sleep(run_time)

            # Start landing wind tunnel speed
            set_wind_tunnel_diff_speeds(landing_motors)
            rospy.sleep(landing_time)

            # Stop experiment
            gradual_stop_wind_tunnel()
            set_control_parameter("2")

            
        elif option == "5":
            stop_wind_tunnel()
            # set_control_parameter("12")
            # Delay before running the experiment
            rospy.sleep(delay)

            # Run the experiment
            set_wind_tunnel_diff_speeds(motors)
            rospy.sleep(wait_time_before_running_controller)
            set_control_parameter("0")

            # Wait for the run-time
            rospy.sleep(run_time)

            # Start landing wind tunnel speed
            set_wind_tunnel_diff_speeds(landing_motors)
            rospy.sleep(landing_time)


            # Stop experiment
            gradual_stop_wind_tunnel()
            set_control_parameter("2")


        elif option == "6":
            stop_wind_tunnel()
            # # ===== Function description =====
            # - Wait delay
            # - Start taking off speed
            # - Run controller
            # - Wait takeoff_time
            # - Start flying speed
            # - Wait run_time
            # - Start landing speed
            # - landing_time
            # - Stop controller

            rospy.sleep(delay)

            # Start taking-off wind tunnel speed
            set_wind_tunnel_diff_speeds(takeoff_motors)
            # Start controller
            rospy.sleep(wait_time_before_running_controller)
            set_control_parameter("0")
            rospy.sleep(takeoff_time)

            # Start landing wind tunnel speed
            set_wind_tunnel_diff_speeds(landing_motors)
            rospy.sleep(landing_time)

            # Stop experiment
            gradual_stop_wind_tunnel()
            set_control_parameter("2")

            

        elif option == "7":
            # set_wind_tunnel_speed()
            rospy.sleep(delay)
            set_wind_tunnel_diff_speeds(motors)
            rospy.sleep(run_time)

            # Stop experiment
            gradual_stop_wind_tunnel()
            set_control_parameter("2")
            
        elif option == "8":
            rospy.sleep(delay)
            set_control_parameter("0")

        elif option == "A" or option == "a":
            set_control_parameter("5")

        elif option == "S" or option == "s":
            wind_tunnel_speed = int(input("Enter wind tunnel speed: "))
            print("Changed wind tunnel speed to %d".format(wind_tunnel_speed))

        elif option == "T" or option == "t":
            wait_time_before_running_controller = float(input("Enter wait time before running controller: "))
            print("Changed wait time before running controller to %d".format(wait_time_before_running_controller))

        elif option == "R" or option == "r":
            run_time = float(input("Enter new run-time: "))
            print("Changed run time to %d".format(run_time))

        elif option == "D" or option == "d":
            delay = float(input("Enter new Delay: "))
            print("Changed Delay to %d".format(delay))

        elif option == "L" or option == "l":
            motors = motors_low

        elif option == "M" or option == "m":
            motors_string = input("Enter new motors speeds: ")
            int_array = [int(x) for x in motors_string.split()]
            if len(int_array) !=6:
                print("Wrong num of speeds!")
            else:
                for i in range(6):
                    motors[i] = int_array[i]
                print("Changed motors speeds to :"+motors_string)

        # elif option == "Y1" or option == "y1":
        #     set_control_parameter("6")


        # elif option == "Y2" or option == "y2":
        #     set_control_parameter("7")

        elif option.lower() == "xp":
            val = input("Enter new X position control param: ")
            set_general_parameter("extCtrl", "x_P", val)

        elif option.lower() == "yp":
            val = input("Enter new Y position control param: ")
            set_general_parameter("extCtrl", "y_P", val)

        elif option.lower() == "zp":
            val = input("Enter new Z position control param: ")
            set_general_parameter("extCtrl", "z_P", val)

        elif option.lower() == "vxp":
            val = input("Enter new X velocity control param: ")
            set_general_parameter("extCtrl", "vx_P", val)

        elif option.lower() == "vyp":
            val = input("Enter new Y velocity control param: ")
            set_general_parameter("extCtrl", "vy_P", val)

        elif option.lower() == "vzp":
            val = input("Enter new Z velocity control param: ")
            set_general_parameter("extCtrl", "vz_P", val)

        elif option.lower() == "rp":
            val = input("Enter new roll P control param: ")
            set_general_parameter("extCtrl", "roll_P", val)

        elif option.lower() == "pp":
            val = input("Enter new pitch P control param: ")
            set_general_parameter("extCtrl", "pitch_P", val)

        elif option.lower() == "yp":
            val = input("Enter new yaw P control param: ")
            set_general_parameter("extCtrl", "yaw_P", val)

        elif option.lower() == "rrp":
            val = input("Enter new roll rate P control param: ")
            set_general_parameter("extCtrl", "roll_rate_P", val)

        elif option.lower() == "prp":
            val = input("Enter new pitch rate P control param: ")
            set_general_parameter("extCtrl", "pitch_rate_P", val)

        elif option.lower() == "yrp":
            val = input("Enter new yaw rate P control param: ")
            set_general_parameter("extCtrl", "yaw_rate_P", val)

        # elif option == "st" or option == "St" or option == "ST"  or option == "sT":
        #     angle_counter = 0
        #     param_to_send = 10 + angle_counter
        #     set_control_parameter(str(param_to_send))


        # elif option == "+"or option == "=":
        #     angle_counter = angle_counter+1
        #     if angle_counter>20:
        #         angle_counter = 20
        #     param_to_send = 10 + angle_counter
        #     set_control_parameter(str(param_to_send))

        # elif option == "-":
        #     angle_counter = angle_counter-1
        #     if angle_counter<0:
        #         angle_counter = 0
        #     param_to_send = 10 + angle_counter
        #     set_control_parameter(str(param_to_send))
        
        
        hover_angle = 180/3.14*angle_counter*0.05

    rospy.spin()

