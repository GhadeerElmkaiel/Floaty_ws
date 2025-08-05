import cflib.crtp
import time
import rospy
import numpy as np
import math
from floaty_msgs.srv import empty_srv, int_srv, maestro_srv
from floaty_msgs.srv import empty_srvRequest, int_srvRequest


class WindTunnelRunner:
    def __init__(self):
        # ROS Setup
        rospy.init_node("wind_tunnel_runner")
        ramp_motors_same_speed_srv_name = "/maestro_control/ramp_motors_same_speed_service"
        ramp_motors_diff_speeds_srv_name = "/maestro_control/ramp_motors_service"
        stop_motors_srv_name = "/maestro_control/stop_motors_service"
        rospy.wait_for_service(ramp_motors_same_speed_srv_name)
        rospy.wait_for_service(ramp_motors_diff_speeds_srv_name)
        rospy.wait_for_service(stop_motors_srv_name)
        self.wind_tunnel_same_speed_srv = rospy.ServiceProxy(ramp_motors_same_speed_srv_name, int_srv)
        self.wind_tunnel_diff_speeds_srv = rospy.ServiceProxy(ramp_motors_diff_speeds_srv_name, maestro_srv)
        self.wind_tunnel_stop_srv = rospy.ServiceProxy(stop_motors_srv_name, empty_srv)

    def stop_wind_tunnel(self):        
        request = empty_srvRequest()
        self.wind_tunnel_stop_srv(request)
        return
    
    def ramp_wind_tunnel_same_speed(self, speed):        
        request = int_srvRequest()
        request.data = speed
        self.wind_tunnel_same_speed_srv(request)
        print("Running wind tunnal at speed {}".format(speed))
        return
    
    def ramp_wind_tunnel(self, speeds):        
        request = maestro_srv()
        request.speeds = speeds
        self.wind_tunnel_diff_speeds_srv(request)
        return


    def run(self, loops=2, loop_time=12, min_speed = 20, max_speed=27, divs = 7, time_delay=6):
        print("Waiting for {} s delay".format(time_delay))
        time.sleep(time_delay)
        speed_step = (max_speed-min_speed)/divs
        time_step = loop_time/(divs*2)
        for i in range(loops):
            current_speed = int(min_speed)
            for j in range(divs):
                self.ramp_wind_tunnel_same_speed(current_speed)
                time.sleep(time_step)
                current_speed = int(current_speed + speed_step)

            current_speed = int(max_speed)
            for j in range(divs):
                self.ramp_wind_tunnel_same_speed(current_speed)
                time.sleep(time_step)
                current_speed = int(current_speed - speed_step)
        self.stop_wind_tunnel()


if __name__ == '__main__':
    wind_tunnel_runner = WindTunnelRunner()
    try:
        # min_speed = 20
        # max_speed=27
        min_speed = 30
        max_speed=45
        wind_tunnel_runner.run(min_speed=min_speed, max_speed=max_speed)
    except rospy.ROSInterruptException:
        pass