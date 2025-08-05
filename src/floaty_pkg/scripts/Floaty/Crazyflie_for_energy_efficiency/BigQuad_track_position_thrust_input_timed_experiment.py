#!/usr/bin/env python
import time
import rospy
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from floaty_msgs.srv import empty_srv, int_srv, maestro_srv
from floaty_msgs.srv import empty_srvRequest, int_srvRequest

from std_msgs.msg import Float64

# Crazyflie URI
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

RUN_WIND_TUNNEL = False
# RUN_WIND_TUNNEL = True

# --- CONFIGURATION FOR YOUR BIGQUAD ---
# THESE VALUES ARE CRITICAL AND MUST BE TUNED EXPERIMENTALLY FOR YOUR SPECIFIC BUILD!

# 1. Base Thrust (0-65535) - The thrust value that makes your BigQuad hover.
#    Start very low and increase cautiously during testing.

# For a fresh battery
# BIGQUAD_HOVER_THRUST = 24200  
# BIGQUAD_HOVER_THRUST_WIND_TUNNEL = 15000  

BIGQUAD_HOVER_THRUST = 24500  
BIGQUAD_HOVER_THRUST_WIND_TUNNEL = 21000  
if RUN_WIND_TUNNEL:
    BIGQUAD_HOVER_THRUST = BIGQUAD_HOVER_THRUST_WIND_TUNNEL  

# 2. Thrust Limits (0-65535)
MIN_THRUST_BIGQUAD = 13000  # EXAMPLE - Should be just above where motors reliably spin
MAX_THRUST_BIGQUAD = 35000  # EXAMPLE - Max thrust you want to allow for safety

# 3. PID Gains (These will be VERY different from a stock Crazyflie)
#    Start with very low P gains, I=0, D=0. Tune methodically.
#    X, Y control position by commanding roll/pitch angles.
#    Z controls altitude by adjusting thrust.
#    Yaw controls heading by commanding yaw rate.

# Position PID (outputs desired roll/pitch angles in degrees)
# Limit output: Max roll/pitch angle command
# Integral limit: Max accumulated integral error (to prevent windup)
KP_POS = 50.0     # EXAMPLE
KI_POS = 10.0    # Start with 0
KD_POS = 10.0     # Start with 0
POS_PID_LIMIT_OUTPUT = 15.0  # Max roll/pitch angle (degrees)
POS_PID_INTEGRAL_LIMIT = 10.0 # Max integral value for position

if RUN_WIND_TUNNEL:
    KP_POS = 90.0     # EXAMPLE
    KI_POS = 10.0    # EXAMPLE - Start with 0
    KD_POS = 30.0     # EXAMPLE - Start with 0
    POS_PID_LIMIT_OUTPUT = 35.0  # Max roll/pitch angle (degrees)
    POS_PID_INTEGRAL_LIMIT = 20.0 # Max integral value for position


# Altitude PID (outputs thrust adjustment to add/subtract from hover thrust)
KP_ALT = 1500.0  # EXAMPLE
KI_ALT = 0.0   # EXAMPLE - Start with 0
KD_ALT = 20.0  # EXAMPLE - Start with 0
# KI_ALT = 200.0   # EXAMPLE - Start with 0
# KD_ALT = 1500.0  # EXAMPLE - Start with 0
ALT_PID_LIMIT_OUTPUT = 8000.0 # Max thrust adjustment
ALT_PID_INTEGRAL_LIMIT = 5000.0 # Max integral value for altitude

if RUN_WIND_TUNNEL:
    KP_ALT = 800.0  # EXAMPLE
    KI_ALT = 100.0   # EXAMPLE - Start with 0
    KD_ALT = 90.0  # EXAMPLE - Start with 0
    # KI_ALT = 200.0   # EXAMPLE - Start with 0
    # KD_ALT = 1500.0  # EXAMPLE - Start with 0
    ALT_PID_LIMIT_OUTPUT = 8000.0 # Max thrust adjustment
    ALT_PID_INTEGRAL_LIMIT = 5000.0 # Max integral value for altitude

# Yaw PID (outputs desired yaw rate in degrees/second)
KP_YAW = 0.65     # EXAMPLE
KI_YAW = 0.05    # EXAMPLE - Start with 0
KD_YAW = 0.08    # EXAMPLE - Start with 0
YAW_PID_LIMIT_OUTPUT = 70.0  # Max yaw rate (degrees/s)
YAW_PID_INTEGRAL_LIMIT = 50.0  # Max integral value for yaw

# --- Helper Functions ---
def quaternion_to_yaw_degrees(qx, qy, qz, qw):
    """Convert quaternion to yaw in degrees."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

class PID:
    def __init__(self, Kp, Ki, Kd, output_limit, integral_limit=None, setpoint=0.0, name=""):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit) if integral_limit is not None else None
        self.setpoint = setpoint
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None
        self.name = name # For logging

        # Store individual terms for logging
        self.P_term = 0.0
        self.I_term = 0.0
        self.D_term = 0.0
        self.output_raw = 0.0 # Before clipping

    def update(self, measurement, current_time):
        error = self.setpoint - measurement
        
        if self.last_time is None:
            dt = 0.01 # Assume 100Hz for the first iteration if needed
        else:
            dt = current_time - self.last_time
        
        if dt <= 0: # Prevent division by zero or negative dt
            return 0.0 

        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / dt

        self.P_term = self.Kp * error
        self.I_term = self.Ki * self.integral
        self.D_term = self.Kd * derivative
        self.output_raw = self.P_term + self.I_term + self.D_term

        self.prev_error = error
        self.last_time = current_time
        return np.clip(self.output_raw, -self.output_limit, self.output_limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def set_setpoint(self, new_setpoint): # Helper to change setpoint
        self.setpoint = new_setpoint
        # Optionally reset integral and prev_error if setpoint changes significantly,
        # but for gradual ramp, it might be okay not to.
        # self.integral = 0.0 
        # self.prev_error = 0.0 # Or new_setpoint - current_measurement

class BigQuadOffboardController:
    def __init__(self, cf, goal_pos, goal_yaw_deg):

        # --- ROS Publishers for Plotting ---
        self.pub_error_x = rospy.Publisher('pid_debug/error_x', Float64, queue_size=1)
        self.pub_p_term_x = rospy.Publisher('pid_debug/p_term_x', Float64, queue_size=1)
        self.pub_i_term_x = rospy.Publisher('pid_debug/i_term_x', Float64, queue_size=1)
        self.pub_d_term_x = rospy.Publisher('pid_debug/d_term_x', Float64, queue_size=1)
        self.pub_output_x = rospy.Publisher('pid_debug/output_x', Float64, queue_size=1) # This is pitch_command

        self.pub_error_y = rospy.Publisher('pid_debug/error_y', Float64, queue_size=1)
        self.pub_p_term_y = rospy.Publisher('pid_debug/p_term_y', Float64, queue_size=1)
        self.pub_i_term_y = rospy.Publisher('pid_debug/i_term_y', Float64, queue_size=1)
        self.pub_d_term_y = rospy.Publisher('pid_debug/d_term_y', Float64, queue_size=1)
        self.pub_output_y = rospy.Publisher('pid_debug/output_y', Float64, queue_size=1) # This is roll_command

        self.pub_error_z = rospy.Publisher('pid_debug/error_z', Float64, queue_size=1)
        self.pub_p_term_z = rospy.Publisher('pid_debug/p_term_z', Float64, queue_size=1)
        self.pub_i_term_z = rospy.Publisher('pid_debug/i_term_z', Float64, queue_size=1)
        self.pub_d_term_z = rospy.Publisher('pid_debug/d_term_z', Float64, queue_size=1)
        self.pub_output_z = rospy.Publisher('pid_debug/output_z_thrust_adj', Float64, queue_size=1) # thrust_adjustment
        self.pub_final_thrust = rospy.Publisher('pid_debug/final_thrust', Float64, queue_size=1) # thrust_command

        self.pub_error_yaw = rospy.Publisher('pid_debug/error_yaw', Float64, queue_size=1)
        self.pub_p_term_yaw = rospy.Publisher('pid_debug/p_term_yaw', Float64, queue_size=1)
        self.pub_i_term_yaw = rospy.Publisher('pid_debug/i_term_yaw', Float64, queue_size=1)
        self.pub_d_term_yaw = rospy.Publisher('pid_debug/d_term_yaw', Float64, queue_size=1)
        self.pub_output_yaw = rospy.Publisher('pid_debug/output_yaw_rate', Float64, queue_size=1) # yaw_rate_command

        self.pub_current_x = rospy.Publisher('pid_debug/current_x', Float64, queue_size=1)
        self.pub_current_y = rospy.Publisher('pid_debug/current_y', Float64, queue_size=1)
        self.pub_current_z = rospy.Publisher('pid_debug/current_z', Float64, queue_size=1)
        self.pub_setpoint_z = rospy.Publisher('pid_debug/setpoint_z', Float64, queue_size=1)


        self.cf = cf
        self.goal_pos = np.array(goal_pos)  # [x, y, z]
        self.goal_yaw_deg = goal_yaw_deg

        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.current_yaw_deg = 0.0
        self.running_iter = 0

        # --- Takeoff Parameters ---
        self.takeoff_initial_z_setpoint = 0.15  # Start ramping Z from this altitude (e.g., just above ground)
                                                # Make sure this is safe and drone won't hit ground
        self.takeoff_ramp_rate_z = 0.1          # meters per second for Z ramp
        self.is_in_takeoff_phase = True         # Flag to indicate takeoff ramp
    
        # Boundery limits
        self.BOUNDARY_RADIUS_SQ = 0.09
        self.BOUNDARY_MIN_Z = 0.0
        self.BOUNDARY_MAX_Z = 0.99
        self.MIN_Z_YAW_TRACK = 0.25

        # Initialize PID controllers
        self.pid_x = PID(KP_POS, KI_POS, KD_POS, POS_PID_LIMIT_OUTPUT, POS_PID_INTEGRAL_LIMIT, setpoint=self.goal_pos[0], name="PID_X")
        self.pid_y = PID(KP_POS, KI_POS, KD_POS, POS_PID_LIMIT_OUTPUT, POS_PID_INTEGRAL_LIMIT, setpoint=self.goal_pos[1], name="PID_Y")
        # self.pid_z = PID(KP_ALT, KI_ALT, KD_ALT, ALT_PID_LIMIT_OUTPUT, ALT_PID_INTEGRAL_LIMIT, setpoint=self.goal_pos[2])
        self.pid_z = PID(KP_ALT, KI_ALT, KD_ALT, ALT_PID_LIMIT_OUTPUT, ALT_PID_INTEGRAL_LIMIT, setpoint=self.takeoff_initial_z_setpoint, name="PID_Z")

        self.pid_yaw = PID(KP_YAW, KI_YAW, KD_YAW, YAW_PID_LIMIT_OUTPUT, YAW_PID_INTEGRAL_LIMIT, setpoint=self.goal_yaw_deg, name="PID_Yaw")
        
        self.is_flying = False
        self.last_pose_time = None
        self.last_control_time = None
        self.last_thrust_command = None

        # ROS Subscriber for pose
        self.pose_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, self.pose_callback) # Adjust topic name
        rospy.loginfo("Subscribed to OptiTrack pose topic.")

        ramp_motors_same_speed_srv_name = "/maestro_control/ramp_motors_same_speed_service"
        ramp_motors_diff_speeds_srv_name = "/maestro_control/ramp_motors_service"
        stop_motors_srv_name = "/maestro_control/stop_motors_service"
        rospy.wait_for_service(ramp_motors_same_speed_srv_name)
        rospy.wait_for_service(ramp_motors_diff_speeds_srv_name)
        rospy.wait_for_service(stop_motors_srv_name)
        self.wind_tunnel_speed_srv = rospy.ServiceProxy(ramp_motors_same_speed_srv_name, int_srv)
        self.wind_tunnel_diff_speeds_srv = rospy.ServiceProxy(ramp_motors_diff_speeds_srv_name, maestro_srv)
        self.wind_tunnel_stop_srv = rospy.ServiceProxy(stop_motors_srv_name, empty_srv)

    def pose_callback(self, msg):
        self.last_pose_time = rospy.get_time()
        self.current_pos[0] = msg.pose.position.x
        self.current_pos[1] = msg.pose.position.y
        self.current_pos[2] = msg.pose.position.z

        # self.current_pos[0] = 0
        # self.current_pos[1] = 0
        # self.current_pos[2] = 0.5
        self.current_yaw_deg = quaternion_to_yaw_degrees(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        self.current_quat[0] = msg.pose.orientation.x
        self.current_quat[1] = msg.pose.orientation.y
        self.current_quat[2] = msg.pose.orientation.z
        self.current_quat[3] = msg.pose.orientation.w

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

        
        rospy.loginfo("Waiting for first pose update from OptiTrack...")
        while self.last_pose_time is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown(): return
        rospy.loginfo("First pose received. Initializing PIDs.")

        # Reset PIDs with current time
        current_time_ros = rospy.get_time()
        self.pid_x.last_time = current_time_ros
        self.pid_y.last_time = current_time_ros
        self.pid_z.last_time = current_time_ros
        self.pid_yaw.last_time = current_time_ros
        self.pid_x.prev_error = self.pid_x.setpoint - self.current_pos[0]
        self.pid_y.prev_error = self.pid_y.setpoint - self.current_pos[1]
        # self.pid_z.prev_error = self.pid_z.setpoint - self.current_pos[2]
        self.pid_z.prev_error = self.takeoff_initial_z_setpoint - self.current_pos[2]
        
        yaw_error_init = self.goal_yaw_deg - self.current_yaw_deg
        yaw_error_init = (yaw_error_init + 180) % 360 - 180
        self.pid_yaw.prev_error = yaw_error_init

        rospy.loginfo(f"Initial Z setpoint: {self.pid_z.setpoint:.2f}m. Current Z: {self.current_pos[2]:.2f}m")
        rospy.loginfo(f"Targeting final Z: {self.goal_pos[2]:.2f}m via ramp.")

        rospy.loginfo("Setting stabilizer.controller to 1 (PID mode).")
        self.cf.param.set_value('stabilizer.controller', '1')
        # self.cf.param.set_value('stabilizer.controller', '0')
        time.sleep(0.1)
        # It's also good to set deck.bcBigQuad.enable = 1 here if not set by default
        # self.cf.param.set_value('deck.bcBigQuad.enable', '1') # Ensure BigQuad deck is active

        if RUN_WIND_TUNNEL:
            self.ramp_wind_tunnel(40)

        rospy.loginfo("Unlocking motors by sending 0 thrust for a moment...")
        for _ in range(20): # Send for ~0.2 seconds
            self.cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            time.sleep(0.01)
        
        self.is_flying = True
        rospy.loginfo("Starting BigQuad offboard control loop.")

        last_control_time = rospy.get_time() # For precise dt in ramp

        rate = rospy.Rate(100) # Control loop frequency (e.g., 100 Hz)
        while not rospy.is_shutdown() and self.is_flying:
            current_time_ros = rospy.get_time()
            actual_dt = current_time_ros - last_control_time
            last_control_time = current_time_ros
            self.last_control_time = last_control_time

            if self.last_pose_time is None or (current_time_ros - self.last_pose_time > 0.5):
                rospy.logwarn_throttle(1.0, "Stale OptiTrack data. Sending zero setpoint for safety.")
                self.cf.commander.send_setpoint(0, 0, 0, 0) # Safety hover or land
                rate.sleep()
                continue


            # --- Boundary Check ---
            current_x = self.current_pos[0]
            current_y = self.current_pos[1]
            current_z = self.current_pos[2]

            dist_sq_from_origin_xy = current_x**2 + current_y**2
            
            outside_boundary = False
            reason = ""

            if dist_sq_from_origin_xy > self.BOUNDARY_RADIUS_SQ:
                outside_boundary = True
                reason = "Exceeded radial boundary (dist_xy={:.2f}m > radius={:.2f}m)".format(
                    math.sqrt(dist_sq_from_origin_xy), self.BOUNDARY_RADIUS)
            elif current_z < self.BOUNDARY_MIN_Z:
                outside_boundary = True
                reason = "Below Z boundary ({:.2f}m < {:.2f}m)".format(current_z, self.BOUNDARY_MIN_Z)
            elif current_z > self.BOUNDARY_MAX_Z:
                outside_boundary = True
                reason = "Above Z boundary ({:.2f}m > {:.2f}m)".format(current_z, self.BOUNDARY_MAX_Z)

            if outside_boundary:
                rospy.logwarn("BOUNDARY BREACHED: {}. Initiating emergency break.".format(reason))
                self.emergency_break() # This sets self.is_flying = False
                break # Exit the control loop immediately


            # --- Takeoff Ramp Logic for Z Setpoint ---
            if self.is_in_takeoff_phase:
                current_z_setpoint = self.pid_z.setpoint
                if current_z_setpoint < self.goal_pos[2]:
                    # Increment setpoint. Use actual_dt for smoother ramp if loop rate varies.
                    increment = self.takeoff_ramp_rate_z * actual_dt 
                    new_z_setpoint = min(current_z_setpoint + increment, self.goal_pos[2])
                    self.pid_z.set_setpoint(new_z_setpoint)
                    rospy.loginfo_throttle(0.2, f"Takeoff Ramp: Z_setpoint={new_z_setpoint:.3f}m (target: {self.goal_pos[2]:.2f}m)")
                else:
                    self.is_in_takeoff_phase = False
                    self.pid_z.set_setpoint(self.goal_pos[2]) # Ensure it's exactly the goal
                    rospy.loginfo(f"Takeoff ramp complete. Z setpoint: {self.pid_z.setpoint:.2f}m")


            # PID calculations
            # IMPORTANT: Roll/Pitch signs depend on your OptiTrack coordinate system
            # and how it maps to the Crazyflie's desired motion.
            # Assuming:
            # - OptiTrack X positive is "forward" for the drone.
            # - OptiTrack Y positive is "left" for the drone.
            # - Crazyflie send_setpoint:
            #   - Roll: Positive makes drone roll right.
            #   - Pitch: Positive makes drone pitch nose UP.
            
            # If error_x > 0 (drone is behind goal on X), we want to pitch forward (nose DOWN).
            # Pitch from PID_X will be positive (move in positive X). send_setpoint pitch is + for nose UP.
            # So, pitch_command = -pid_x.update(...)
            # pitch_command = -self.pid_x.update(self.current_pos[0], current_time_ros)
            pitch_command = self.pid_x.update(self.current_pos[0], current_time_ros)

            # If error_y > 0 (drone is to the right of goal on Y, since OptiY is left), we want to roll left.
            # Roll from PID_Y will be positive (move in positive Y). send_setpoint roll is + for roll RIGHT.
            # So, roll_command = -pid_y.update(...)
            roll_command = -self.pid_y.update(self.current_pos[1], current_time_ros)
            
            thrust_adjustment = self.pid_z.update(self.current_pos[2], current_time_ros)
            thrust_command = int(BIGQUAD_HOVER_THRUST + thrust_adjustment)
            thrust_command = np.clip(thrust_command, MIN_THRUST_BIGQUAD, MAX_THRUST_BIGQUAD)
            self.last_thrust_command = thrust_command

            yaw_error = self.goal_yaw_deg - self.current_yaw_deg
            yaw_error = (yaw_error + 180) % 360 - 180 # Normalize to [-180, 180]

            # Don't track yaw when close to ground
            if current_z < self.MIN_Z_YAW_TRACK:
            # if self.running_iter < 100:
                yaw_error = 0
            yaw_rate_command = self.pid_yaw.update(yaw_error, current_time_ros) # pid_yaw setpoint is 0 if goal_yaw = current_yaw

            rospy.loginfo_throttle(0.5, # Log every 0.5 seconds
                "P:({:.2f},{:.2f},{:.2f}) Y:{:.1f} | E:({:.2f},{:.2f},{:.2f}) EY:{:.1f} | C: R={:.1f} P={:.1f} YR={:.1f} T={}"
                .format(self.current_pos[0], self.current_pos[1], self.current_pos[2], self.current_yaw_deg,
                        self.pid_x.prev_error, self.pid_y.prev_error, self.pid_z.prev_error, yaw_error,
                        roll_command, pitch_command, yaw_rate_command, thrust_command))

            self.cf.extpos.send_extpose(self.current_pos[0], self.current_pos[1], self.current_pos[2], self.current_quat[0], self.current_quat[1], self.current_quat[2], self.current_quat[3])

            self.cf.commander.send_setpoint(roll_command, pitch_command, yaw_rate_command, thrust_command)
            self.running_iter = self.running_iter + 1
            

            # Publish PID debug values
            self.pub_error_x.publish(Float64(self.pid_x.prev_error)) # prev_error is the error from the *last* update
            self.pub_p_term_x.publish(Float64(self.pid_x.P_term))
            self.pub_i_term_x.publish(Float64(self.pid_x.I_term))
            self.pub_d_term_x.publish(Float64(self.pid_x.D_term))
            self.pub_output_x.publish(Float64(pitch_command))

            self.pub_error_y.publish(Float64(self.pid_y.prev_error))
            self.pub_p_term_y.publish(Float64(self.pid_y.P_term))
            self.pub_i_term_y.publish(Float64(self.pid_y.I_term))
            self.pub_d_term_y.publish(Float64(self.pid_y.D_term))
            self.pub_output_y.publish(Float64(roll_command))

            self.pub_error_z.publish(Float64(self.pid_z.prev_error))
            self.pub_p_term_z.publish(Float64(self.pid_z.P_term))
            self.pub_i_term_z.publish(Float64(self.pid_z.I_term))
            self.pub_d_term_z.publish(Float64(self.pid_z.D_term))
            self.pub_output_z.publish(Float64(thrust_adjustment))
            self.pub_final_thrust.publish(Float64(thrust_command/100))


            self.pub_error_yaw.publish(Float64(yaw_error)) # Use the current normalized yaw error
            self.pub_p_term_yaw.publish(Float64(self.pid_yaw.P_term))
            self.pub_i_term_yaw.publish(Float64(self.pid_yaw.I_term))
            self.pub_d_term_yaw.publish(Float64(self.pid_yaw.D_term))
            self.pub_output_yaw.publish(Float64(yaw_rate_command))

            # Publish current positions and Z setpoint for comparison
            self.pub_current_x.publish(Float64(self.current_pos[0]))
            self.pub_current_y.publish(Float64(self.current_pos[1]))
            self.pub_current_z.publish(Float64(self.current_pos[2]))
            self.pub_setpoint_z.publish(Float64(self.pid_z.setpoint))


            rate.sleep()
            
        # self.emergency_break()
        # if RUN_WIND_TUNNEL:
        #     self.stop_wind_tunnel()

        self.land()

    def land_controlled(self):
        landing_thrust = self.last_thrust_command
        last_control_time = self.last_control_time

        while landing_thrust > MIN_THRUST_BIGQUAD - 2000:
            current_time_ros = rospy.get_time()
            actual_dt = current_time_ros - last_control_time
            last_control_time = current_time_ros

            # --- Boundary Check ---
            current_x = self.current_pos[0]
            current_y = self.current_pos[1]
            current_z = self.current_pos[2]

            dist_sq_from_origin_xy = current_x**2 + current_y**2
            
            outside_boundary = False
            reason = ""

            if dist_sq_from_origin_xy > self.BOUNDARY_RADIUS_SQ:
                outside_boundary = True
                reason = "Exceeded radial boundary (dist_xy={:.2f}m > radius={:.2f}m)".format(
                    math.sqrt(dist_sq_from_origin_xy), self.BOUNDARY_RADIUS)
            elif current_z < self.BOUNDARY_MIN_Z:
                outside_boundary = True
                reason = "Below Z boundary ({:.2f}m < {:.2f}m)".format(current_z, self.BOUNDARY_MIN_Z)
            elif current_z > self.BOUNDARY_MAX_Z:
                outside_boundary = True
                reason = "Above Z boundary ({:.2f}m > {:.2f}m)".format(current_z, self.BOUNDARY_MAX_Z)

            if outside_boundary:
                rospy.logwarn("BOUNDARY BREACHED: {}. Initiating emergency break.".format(reason))
                self.emergency_break() # This sets self.is_flying = False
                break # Exit the control loop immediately


            # --- Takeoff Ramp Logic for Z Setpoint ---
            if self.is_in_takeoff_phase:
                current_z_setpoint = self.pid_z.setpoint
                if current_z_setpoint < self.goal_pos[2]:
                    # Increment setpoint. Use actual_dt for smoother ramp if loop rate varies.
                    increment = self.takeoff_ramp_rate_z * actual_dt 
                    new_z_setpoint = min(current_z_setpoint + increment, self.goal_pos[2])
                    self.pid_z.set_setpoint(new_z_setpoint)
                    rospy.loginfo_throttle(0.2, f"Takeoff Ramp: Z_setpoint={new_z_setpoint:.3f}m (target: {self.goal_pos[2]:.2f}m)")
                else:
                    self.is_in_takeoff_phase = False
                    self.pid_z.set_setpoint(self.goal_pos[2]) # Ensure it's exactly the goal
                    rospy.loginfo(f"Takeoff ramp complete. Z setpoint: {self.pid_z.setpoint:.2f}m")
            
            pitch_command = self.pid_x.update(self.current_pos[0], current_time_ros)

            # If error_y > 0 (drone is to the right of goal on Y, since OptiY is left), we want to roll left.
            # Roll from PID_Y will be positive (move in positive Y). send_setpoint roll is + for roll RIGHT.
            # So, roll_command = -pid_y.update(...)
            roll_command = -self.pid_y.update(self.current_pos[1], current_time_ros)
            
            thrust_adjustment = self.pid_z.update(self.current_pos[2], current_time_ros)
            thrust_command = int(landing_thrust + thrust_adjustment)
            thrust_command = np.clip(thrust_command, MIN_THRUST_BIGQUAD-2500, MAX_THRUST_BIGQUAD)

            yaw_error = self.goal_yaw_deg - self.current_yaw_deg
            yaw_error = (yaw_error + 180) % 360 - 180 # Normalize to [-180, 180]

            # Don't track yaw when close to ground
            if current_z < self.MIN_Z_YAW_TRACK:
            # if self.running_iter < 100:
                yaw_error = 0
            yaw_rate_command = self.pid_yaw.update(yaw_error, current_time_ros) # pid_yaw setpoint is 0 if goal_yaw = current_yaw

            rospy.loginfo_throttle(0.5, # Log every 0.5 seconds
                "P:({:.2f},{:.2f},{:.2f}) Y:{:.1f} | E:({:.2f},{:.2f},{:.2f}) EY:{:.1f} | C: R={:.1f} P={:.1f} YR={:.1f} T={}"
                .format(self.current_pos[0], self.current_pos[1], self.current_pos[2], self.current_yaw_deg,
                        self.pid_x.prev_error, self.pid_y.prev_error, self.pid_z.prev_error, yaw_error,
                        roll_command, pitch_command, yaw_rate_command, thrust_command))

            self.cf.extpos.send_extpose(self.current_pos[0], self.current_pos[1], self.current_pos[2], self.current_quat[0], self.current_quat[1], self.current_quat[2], self.current_quat[3])

            self.cf.commander.send_setpoint(roll_command, pitch_command, yaw_rate_command, thrust_command)
            landing_thrust = landing_thrust-100
            time.sleep(0.01)

    def land(self):
        rospy.loginfo("Landing sequence initiated.")
        self.is_flying = False
        if RUN_WIND_TUNNEL:
            request = empty_srvRequest()
            self.wind_tunnel_stop_srv(request)
        
        # Gently reduce thrust
        # Get current thrust from last command or estimate from hover
        landing_thrust = BIGQUAD_HOVER_THRUST * 0.95 # Start a bit below hover
        if landing_thrust < MIN_THRUST_BIGQUAD:
            landing_thrust = MIN_THRUST_BIGQUAD + 1000 # Ensure it's above min initially

        while landing_thrust > MIN_THRUST_BIGQUAD - 2000: # Go a bit below min to ensure motors stop
            landing_thrust -= 300 # Decrease thrust, TUNE THIS RATE
            if landing_thrust < 0: landing_thrust = 0
            self.cf.commander.send_setpoint(0, 0, 0, int(landing_thrust))
            rospy.loginfo(f"Landing thrust: {int(landing_thrust)}")
            time.sleep(0.05)

        rospy.loginfo("Sending stop setpoint to disarm motors.")
        self.cf.commander.send_stop_setpoint()
        time.sleep(0.1)
        # self.cf.close_link() # SyncCrazyflie handles this

    def emergency_break(self):
        rospy.loginfo("Emergency breaking.")
        self.is_flying = False
        
        rospy.loginfo("Sending zero roll/pitch/yaw_rate and zero thrust.")
        self.cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        if RUN_WIND_TUNNEL:
            request = empty_srvRequest()
            self.wind_tunnel_stop_srv(request)
        rospy.loginfo("Sending stop setpoint to disarm motors.")
        self.cf.commander.send_stop_setpoint()
        time.sleep(0.1)
        # self.cf.close_link() # SyncCrazyflie handles this

def main():
    print("starting")
    rospy.init_node('bigquad_offboard_controller_node')
    cflib.crtp.init_drivers()

    # --- Define Goal ---
    # IMPORTANT: Ensure your OptiTrack system is calibrated and Z=0 is ground.
    goal_position = [0.0, 0.0, 0.8]  # Target X, Y, Z in meters
    goal_yaw = 0.0  # Target yaw in degrees

    stop_motors_srv_name = "/maestro_control/stop_motors_service"
    rospy.wait_for_service(stop_motors_srv_name)
    wind_tunnel_stop_srv = rospy.ServiceProxy(stop_motors_srv_name, empty_srv)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf_instance = scf.cf
        
        # --- Pre-Flight Parameter Checks/Sets (Optional but Recommended) ---
        # These should ideally be set once via CFClient and saved.
        # If you set them here, they might revert if the CF reboots without saving.
        # Example: Ensure BigQuad deck is enabled and ESC protocol is set
        # current_esc_protocol = cf_instance.param.get_value('deck.bcBigQuad.escProtocol')
        # rospy.loginfo(f"Current deck.bcBigQuad.escProtocol: {current_esc_protocol}")
        # cf_instance.param.set_value('deck.bcBigQuad.escProtocol', '0') # 0 for PWM, 3 for DSHOT150, 4 for DSHOT300 etc.
        # cf_instance.param.set_value('deck.bcBigQuad.escRate', '400') # If PWM
        # cf_instance.param.set_value('deck.bcBigQuad.enable', '1')
        # time.sleep(0.2) # Allow time for params to be set

        controller = BigQuadOffboardController(cf_instance, goal_position, goal_yaw)
        try:
            controller.run()
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received. Landing...")
            # controller.land()
            controller.emergency_break()
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")
            # controller.land() # Attempt to land on error
            if "/maestro_control/stop_motors_service" in e:
                controller.land()
            else:
                controller.emergency_break()
        finally:
            rospy.loginfo("Controller run finished or exited. Ensuring motors are stopped.")
            # The emergency_break() at the end of run() or in except blocks should handle this.
            # If cf_instance is still valid, an extra stop command can be sent, but be careful if link is already closed.
            
            request = empty_srvRequest()
            wind_tunnel_stop_srv(request)

            if cf_instance and cf_instance.is_connected():
                rospy.loginfo("Sending final stop setpoint just in case.")
                cf_instance.commander.send_setpoint(0,0,0,0) # Belt and suspenders
                cf_instance.commander.send_stop_setpoint()

            rospy.loginfo("Shutdown complete.")

if __name__ == '__main__':
    main()