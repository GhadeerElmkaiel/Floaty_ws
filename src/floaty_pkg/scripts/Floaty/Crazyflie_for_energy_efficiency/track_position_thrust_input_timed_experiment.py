# #!/usr/bin/env python
# import time
# import rospy
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.utils import uri_helper
# from geometry_msgs.msg import PoseStamped

# # ----- PID Controller Class -----
# class PID:
#     def __init__(self, kp, ki, kd, setpoint=0):
#         self.kp = kp
#         self.ki = ki
#         self.kd = kd
#         self.setpoint = setpoint
#         self.integral = 0.0
#         self.last_error = 0.0
#         self.last_time = None

#     def update(self, measurement):
#         current_time = time.time()
#         error = self.setpoint - measurement
#         if self.last_time is None:
#             dt = 0.01
#         else:
#             dt = current_time - self.last_time

#         self.integral += error * dt
#         derivative = (error - self.last_error) / dt if dt > 0 else 0.0

#         output = self.kp * error + self.ki * self.integral + self.kd * derivative

#         self.last_error = error
#         self.last_time = current_time
#         return output

# # ----- Global Variables and Goal -----
# # Current position (updated via ROS)
# x = 0.0
# y = 0.0
# z = 0.0

# # Desired goal position
# class Position:
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z

# Goal = Position(0.0, 0.0, 0.5)  # For example, hover at 0.5 m altitude

# # ----- Create PID Controllers for each axis -----
# # Note: Gains must be tuned experimentally.
# pid_x = PID(kp=1.0, ki=0.0, kd=0.2, setpoint=Goal.x)
# pid_y = PID(kp=1.0, ki=0.0, kd=0.2, setpoint=Goal.y)
# # For altitude, the output will adjust the thrust command.
# pid_z = PID(kp=3000.0, ki=1000.0, kd=500.0, setpoint=Goal.z)

# # Hover thrust (this is an approximate value; adjust as needed)
# hover_thrust = 40000

# # ----- ROS Callback to Update Pose -----
# def update_pose_callback(msg):
#     global x, y, z
#     x = msg.pose.position.x
#     y = msg.pose.position.y
#     z = msg.pose.position.z
#     # (Optional) Update PID setpoints if Goal is dynamic

# # ----- Control Loop (PID Computation) -----
# def control_loop(event):
#     global x, y, z, Goal, hover_thrust, pid_x, pid_y, pid_z, cf

#     # Compute PID outputs for lateral control:
#     # Here we assume:
#     #   - The pitch command (in degrees) controls movement along the x-axis.
#     #   - The roll command (in degrees) controls movement along the y-axis.
#     # Adjust signs as needed for your coordinate system.
#     desired_pitch = pid_x.update(x)    # Positive pitch tilts the drone forward
#     desired_roll = -pid_y.update(y)      # Negative roll tilts the drone left/right

#     # Compute the altitude (z) error and adjust thrust:
#     thrust_adjust = pid_z.update(z)
#     commanded_thrust = hover_thrust + thrust_adjust
#     commanded_thrust = max(10000, min(60000, commanded_thrust))  # clamp thrust to safe limits

#     # Yaw rate is set to zero (you can add a PID for yaw if needed)
#     yaw_rate = 0

#     rospy.loginfo("x: {:.2f}, y: {:.2f}, z: {:.2f} | roll: {:.2f}, pitch: {:.2f}, thrust: {:.0f}".format(
#         x, y, z, desired_roll, desired_pitch, commanded_thrust))

#     # Send the computed setpoint (roll, pitch, yaw_rate, thrust) to the Crazyflie.
#     cf.commander.send_setpoint(desired_roll, desired_pitch, yaw_rate, int(commanded_thrust))

# # ----- Main Routine -----
# if __name__ == '__main__':
#     rospy.init_node("crazyflie_pid_controller_node")

#     # Initialize the low-level drivers
#     cflib.crtp.init_drivers()

#     # Subscribe to external pose updates (e.g., from OptiTrack)
#     pos_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, update_pose_callback)

#     # Get Crazyflie URI and set up connection
#     uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#     cf = Crazyflie(rw_cache='./cache')
#     cf.connected.add_callback(lambda uri: rospy.loginfo("Connected to %s" % uri))
#     cf.connection_failed.add_callback(lambda uri, msg: rospy.loginfo("Connection failed to %s: %s" % (uri, msg)))
#     cf.connection_lost.add_callback(lambda uri, msg: rospy.loginfo("Connection lost to %s: %s" % (uri, msg)))

#     with SyncCrazyflie(uri, cf=cf) as scf:
#         cf = scf.cf
#         # Ensure we use the PID controller mode
#         cf.param.set_value('stabilizer.controller', '1')

#         # Unlock the motors by sending a zero thrust command
#         cf.commander.send_setpoint(0.0, 0.0, 0, 0)
#         time.sleep(0.1)  # Wait for 100 ms
        
#         # Optionally, send an initial external position update (if required)
#         cf.extpos.send_extpose(0, 0, 0, 0, 0, 0, 1)
#         input("Press Enter to start PID control...")

#         # Start the control loop timer (e.g., 100 Hz)
#         rospy.Timer(rospy.Duration(0.01), control_loop)

#         rospy.spin()




from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
import cflib.crtp
import time
from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np
import math
from floaty_msgs.srv import empty_srv, int_srv, maestro_srv
from floaty_msgs.srv import empty_srvRequest, int_srvRequest

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

wind_tunnel_running = False
# wind_tunnel_running = True

def quaternion_to_yaw(qx, qy, qz, qw):
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

class PID:
    def __init__(self, Kp, Ki, Kd, limit):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.limit = limit
        self.integral = 0
        self.prev_error = 0
        
    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return np.clip(output, -self.limit, self.limit)
if wind_tunnel_running:
    # PID wind tunne;
    pid_x = PID(Kp=5.0, Ki=0.0, Kd=2.0, limit=15)  # Roll angle (deg)
    pid_y = PID(Kp=5.0, Ki=0.0, Kd=2.0, limit=15)  # Pitch angle (deg)
    # pid_z = PID(Kp=30000, Ki=2000, Kd=5000, limit=24000)  # Thrust (PWM)
    pid_z = PID(Kp=10000, Ki=500, Kd=3000, limit=24000)  # Thrust (PWM)
    pid_yaw = PID(Kp=0.3, Ki=0.0, Kd=0.08, limit=100)  # Yaw rate (deg/s)

else:
    # PID Controllers (Tune these values!)
    pid_x = PID(Kp=15.5, Ki=0.35, Kd=5.5, limit=15)  # Roll angle (deg)
    pid_y = PID(Kp=15.5, Ki=0.35, Kd=5.5, limit=15)  # Pitch angle (deg)
    # pid_z = PID(Kp=30000, Ki=2000, Kd=5000, limit=24000)  # Thrust (PWM)
    pid_z = PID(Kp=24000, Ki=2000, Kd=11000, limit=17000)  # Thrust (PWM)
    pid_yaw = PID(Kp=0.3, Ki=0.0, Kd=0.08, limit=100)  # Yaw rate (deg/s)


class PositionTracker:
    def __init__(self):
        if wind_tunnel_running:
            self.GOAL = np.array([0.0, 0.0, 1.0])  # x, y, z
        else:
            self.GOAL = np.array([0.0, 0.0, 0.4])  # x, y, z
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.last_time = time.time()
        self.GOAL_YAW = 0.0  # Desired yaw angle in degrees

        # ROS Setup
        rospy.init_node("crazyflie_tracker")
        cflib.crtp.init_drivers()
        self.pos_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, self.pose_callback)
        ramp_motors_same_speed_srv_name = "/maestro_control/ramp_motors_same_speed_service"
        ramp_motors_diff_speeds_srv_name = "/maestro_control/ramp_motors_service"
        stop_motors_srv_name = "/maestro_control/stop_motors_service"
        rospy.wait_for_service(ramp_motors_same_speed_srv_name)
        rospy.wait_for_service(ramp_motors_diff_speeds_srv_name)
        rospy.wait_for_service(stop_motors_srv_name)
        self.wind_tunnel_speed_srv = rospy.ServiceProxy(ramp_motors_same_speed_srv_name, int_srv)
        self.wind_tunnel_diff_speeds_srv = rospy.ServiceProxy(ramp_motors_diff_speeds_srv_name, maestro_srv)
        self.wind_tunnel_stop_srv = rospy.ServiceProxy(stop_motors_srv_name, empty_srv)

        # Crazyflie Connection
        self.cf = Crazyflie(rw_cache='./cache')
        self.scf = SyncCrazyflie(uri, cf=self.cf)
        
    def pose_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.current_yaw = quaternion_to_yaw(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        print("Got position update {:.2f}, y: {:.2f}, z: {:.2f}, yaw: {:.2f}".format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, self.current_yaw))

    def stop_wind_tunnel(self):        
        request = empty_srvRequest()
        self.wind_tunnel_stop_srv(request)
        return
    
    def ramp_wind_tunnel(self, speed):        
        request = int_srvRequest()
        request.data = speed
        self.wind_tunnel_speed_srv(request)
        return


    def run(self):
        with self.scf as scf:
            cf = scf.cf
            cf.param.set_value('stabilizer.controller', '0')  # Manual mode

            if wind_tunnel_running:
                self.ramp_wind_tunnel(30)
            # Arm motors
            for _ in range(60):
                cf.commander.send_setpoint(0, 0, 0, 0)
                time.sleep(0.1)
                
            last_time = time.time()
            
            end_time = 2 + last_time

            # while not rospy.is_shutdown():
            while last_time < end_time:
                dt = time.time() - last_time
                last_time = time.time()
                
                # Calculate errors
                error = self.GOAL - self.current_pos
                
                # PID Calculations
                pitch = pid_x.update(error[0], dt)    # X-error → Pitch
                roll = -pid_y.update(error[1], dt)   # Y-error → - Roll
                if wind_tunnel_running:
                    thrust = int(pid_z.update(error[2], dt) + 25000)  # Base thrust + correction With wind tunnel

                    cf.param.set_value('pid_rate.pitch_kp', '250.0')
                    cf.param.set_value('pid_rate.roll_kp', '250.0')

                    cf.param.set_value('pid_attitude.pitch_kp', '6.0')
                    cf.param.set_value('pid_attitude.roll_kp', '6.0')
                else:
                    thrust = int(pid_z.update(error[2], dt) + 43000)  # Base thrust + correction

                # Calculate yaw error
                yaw_error = self.GOAL_YAW - self.current_yaw

                # Normalize yaw error to [-180, 180] degrees
                # yaw_error = (yaw_error + 180) % 360 - 180
                yaw_error = (yaw_error + 360) % 360 - 180

                # PID calculation for yaw rate
                yaw_rate = pid_yaw.update(yaw_error, dt)  # Yaw rate in degrees/s


                # Send manual commands (roll, pitch, yaw_rate, thrust)
                cf.commander.send_setpoint(roll, pitch, yaw_rate, thrust)
                
                time.sleep(0.01)  # ~100 Hz

            # Land
            self.stop_wind_tunnel()

            for _ in range(100):
                cf.commander.send_setpoint(0, 0, 0, 30000)  # Reduce thrust gradually
                time.sleep(0.01)
            cf.commander.send_stop_setpoint()

if __name__ == '__main__':
    tracker = PositionTracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        pass