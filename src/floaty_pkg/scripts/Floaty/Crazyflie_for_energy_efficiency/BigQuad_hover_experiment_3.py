#!/usr/bin/env python
import time
import rospy
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

# Crazyflie URI
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7') # Adjust if needed

# --- TARGET HOVER CONFIGURATION ---
GOAL_POSITION = np.array([0.0, 0.0, 0.5])  # Target X, Y, Z in meters (OptiTrack frame)
GOAL_YAW_DEGREES = 0.0  # Target yaw in degrees

# --- BIGQUAD PHYSICAL & CONTROL PARAMETERS (CRITICAL - TUNE THESE!) ---
# 1. Hover Thrust (0-65535) - Thrust value for hovering. START LOW, INCREASE CAUTIOUSLY.
BIGQUAD_HOVER_THRUST_APPROX = 18000  # EXAMPLE! Find this for YOUR drone.

# 2. Thrust Limits (0-65535)
MIN_THRUST_BIGQUAD = 10000  # EXAMPLE! Min thrust for reliable motor spin.
MAX_THRUST_BIGQUAD = 35000  # EXAMPLE! Safety cap.

# 3. PID Gains (Start very low, especially P. I and D start at 0)
# Position PID (outputs desired roll/pitch angles in degrees)
KP_POS = 4.0     # EXAMPLE - Proportional gain for X/Y position
KI_POS = 0.0     # EXAMPLE - Integral gain for X/Y position (start at 0)
KD_POS = 1.5     # EXAMPLE - Derivative gain for X/Y position (start at 0)
POS_PID_OUTPUT_LIMIT = 15.0  # Max roll/pitch angle command (degrees)
POS_PID_INTEGRAL_LIMIT = 8.0 # Max accumulated integral error for X/Y

# Altitude PID (outputs thrust adjustment to add/subtract from hover thrust)
KP_ALT = 7000.0  # EXAMPLE - Proportional gain for Z position
KI_ALT = 200.0   # EXAMPLE - Integral gain for Z position (start at 0)
KD_ALT = 2000.0  # EXAMPLE - Derivative gain for Z position (start at 0)
ALT_PID_OUTPUT_LIMIT = 12000.0 # Max thrust adjustment (up or down)
ALT_PID_INTEGRAL_LIMIT = 8000.0 # Max accumulated integral error for Z

# Yaw PID (outputs desired yaw rate in degrees/second)
KP_YAW = 0.3     # EXAMPLE - Proportional gain for Yaw
KI_YAW = 0.0     # EXAMPLE - Integral gain for Yaw (start at 0)
KD_YAW = 0.05    # EXAMPLE - Derivative gain for Yaw (start at 0)
YAW_PID_OUTPUT_LIMIT = 60.0  # Max yaw rate command (degrees/s)
YAW_PID_INTEGRAL_LIMIT = 40.0  # Max accumulated integral error for Yaw

# --- Helper Function ---
def quaternion_to_yaw_degrees(qx, qy, qz, qw):
    """Convert quaternion to yaw in degrees."""
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limit, integral_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit) if integral_limit is not None else None
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def update(self, measurement, current_time):
        error = self.setpoint - measurement
        
        if self.last_time is None: # First call
            self.last_time = current_time
            self.prev_error = error
            # Avoid large derivative spike on first run, return P-term only or zero
            return np.clip(self.Kp * error, -self.output_limit, self.output_limit) 
            # return 0.0 

        dt = current_time - self.last_time
        if dt <= 1e-6: # Prevent division by zero or very small dt
            # Keep previous output or P-term if error changed significantly
            return np.clip(self.Kp * error + self.Ki * self.integral, -self.output_limit, self.output_limit)


        self.integral += error * dt
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error
        self.last_time = current_time
        return np.clip(output, -self.output_limit, self.output_limit)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def set_new_setpoint(self, new_setpoint):
        self.setpoint = new_setpoint
        # Optionally reset integral and prev_error if setpoint changes drastically
        # self.reset()


class BigQuadHoverController:
    def __init__(self, cf, goal_pos_xyz, goal_yaw_deg):
        self.cf = cf
        self.goal_pos_xyz = np.array(goal_pos_xyz)
        self.goal_yaw_deg = goal_yaw_deg

        self.current_pos_xyz = np.array([0.0, 0.0, 0.0])
        self.current_yaw_deg = 0.0
        
        self.is_flying = False
        self.last_pose_update_time = None # When OptiTrack data was last received

        # Initialize PID controllers
        self.pid_x = PID(KP_POS, KI_POS, KD_POS, self.goal_pos_xyz[0], POS_PID_OUTPUT_LIMIT, POS_PID_INTEGRAL_LIMIT)
        self.pid_y = PID(KP_POS, KI_POS, KD_POS, self.goal_pos_xyz[1], POS_PID_OUTPUT_LIMIT, POS_PID_INTEGRAL_LIMIT)
        self.pid_z = PID(KP_ALT, KI_ALT, KD_ALT, self.goal_pos_xyz[2], ALT_PID_OUTPUT_LIMIT, ALT_PID_INTEGRAL_LIMIT)
        self.pid_yaw = PID(KP_YAW, KI_YAW, KD_YAW, self.goal_yaw_deg, YAW_PID_OUTPUT_LIMIT, YAW_PID_INTEGRAL_LIMIT)
        
        # ROS Subscriber for pose from OptiTrack (or other motion capture)
        # IMPORTANT: Adjust the topic name to match your motion capture system's ROS publisher
        self.pose_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, self.pose_callback)
        rospy.loginfo("Subscribed to OptiTrack pose topic: /Optitrack/Floaty")

    def pose_callback(self, msg):
        self.last_pose_update_time = rospy.get_time() # Use ROS time
        self.current_pos_xyz[0] = msg.pose.position.x
        self.current_pos_xyz[1] = msg.pose.position.y
        self.current_pos_xyz[2] = msg.pose.position.z
        self.current_yaw_deg = quaternion_to_yaw_degrees(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        )
        rospy.loginfo_throttle(1.0, "PYTHON RX POSE: X={:.2f}, Y={:.2f}, Z={:.2f}, Yaw={:.1f}"
                             .format(self.current_pos_xyz[0], self.current_pos_xyz[1],
                                     self.current_pos_xyz[2], self.current_yaw_deg))

    def initialize_pids_after_first_pose(self):
        """Wait for the first pose and initialize PID prev_error and last_time."""
        rospy.loginfo("Waiting for first valid pose update to initialize PIDs...")
        while self.last_pose_update_time is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown(): return False
        
        current_time = rospy.get_time()
        self.pid_x.last_time = current_time
        self.pid_x.prev_error = self.pid_x.setpoint - self.current_pos_xyz[0]
        self.pid_y.last_time = current_time
        self.pid_y.prev_error = self.pid_y.setpoint - self.current_pos_xyz[1]
        self.pid_z.last_time = current_time
        self.pid_z.prev_error = self.pid_z.setpoint - self.current_pos_xyz[2]
        
        yaw_error_init = self.pid_yaw.setpoint - self.current_yaw_deg
        yaw_error_init = (yaw_error_init + 180) % 360 - 180
        self.pid_yaw.last_time = current_time
        self.pid_yaw.prev_error = yaw_error_init
        rospy.loginfo("PIDs initialized with first pose data.")
        return True

    def run_hover_control(self):
        if not self.initialize_pids_after_first_pose():
            rospy.logerr("Failed to initialize PIDs, exiting.")
            return

        rospy.loginfo("Setting Crazyflie stabilizer.controller to 1 (PID mode).")
        self.cf.param.set_value('stabilizer.controller', '1')
        time.sleep(0.1) # Allow param to be set

        rospy.loginfo("Unlocking motors by sending 0 thrust for a moment...")
        for _ in range(30): # Send for ~0.3 seconds
            self.cf.commander.send_setpoint(0.0, 0.0, 0, 0)
            time.sleep(0.01)
        
        self.is_flying = True
        rospy.loginfo("Starting BigQuad hover control loop.")

        loop_rate = rospy.Rate(100) # Control loop frequency (e.g., 100 Hz)

        try:
            while not rospy.is_shutdown() and self.is_flying:
                current_time = rospy.get_time()

                if self.last_pose_update_time is None or (current_time - self.last_pose_update_time > 0.5):
                    rospy.logwarn_throttle(1.0, "Stale OptiTrack data! Sending zero thrust for safety.")
                    self.cf.commander.send_setpoint(0, 0, 0, 0) # Safety action
                    loop_rate.sleep()
                    continue

                # --- PID Calculations ---
                # Note on coordinate systems and signs:
                # Assume OptiTrack: X-forward, Y-left, Z-up (ROS REP 103 ENU if OptiTrack is aligned)
                # Crazyflie send_setpoint:
                #   Roll: Positive makes drone roll RIGHT.
                #   Pitch: Positive makes drone pitch nose UP.

                # X-Position Control (generates pitch command)
                # If OptiTrack X_error > 0 (drone is BEHIND goal), we want to pitch FORWARD (nose DOWN).
                # PID_X output will be positive. send_setpoint pitch is + for nose UP. So, pitch_command = -pid_x.
                pitch_command = -self.pid_x.update(self.current_pos_xyz[0], current_time)

                # Y-Position Control (generates roll command)
                # If OptiTrack Y_error > 0 (drone is to the RIGHT of goal, as OptiY is Left), we want to roll RIGHT.
                # PID_Y output will be positive. send_setpoint roll is + for roll RIGHT. So, roll_command = +pid_y.
                # THIS SIGN MAY NEED INVERSION BASED ON YOUR EXACT OPTITRACK Y-AXIS DEFINITION
                # If OptiTrack Y is RIGHT, then error_y > 0 means drone is LEFT of goal, want to roll LEFT (-).
                roll_command = self.pid_y.update(self.current_pos_xyz[1], current_time) # Adjust sign if OptiY is not "left"
                
                # Z-Position (Altitude) Control (generates thrust adjustment)
                thrust_adjustment = self.pid_z.update(self.current_pos_xyz[2], current_time)
                thrust_command = int(BIGQUAD_HOVER_THRUST_APPROX + thrust_adjustment)
                thrust_command = np.clip(thrust_command, MIN_THRUST_BIGQUAD, MAX_THRUST_BIGQUAD)

                # Yaw Control (generates yaw rate command)
                current_yaw_error = self.pid_yaw.setpoint - self.current_yaw_deg
                current_yaw_error = (current_yaw_error + 180) % 360 - 180 # Normalize to [-180, 180]
                # Pass the error directly, as PID update calculates `setpoint - measurement`
                yaw_rate_command = self.pid_yaw.update(self.current_yaw_deg, current_time)


                rospy.loginfo_throttle(0.2, # Log every 0.2 seconds
                    "CTL: X_err={:.2f} Y_err={:.2f} Z_err={:.2f} Yaw_err={:.1f} | CMD: R={:.1f} P={:.1f} YR={:.1f} T={}"
                    .format(self.pid_x.prev_error, self.pid_y.prev_error, self.pid_z.prev_error, current_yaw_error,
                            roll_command, pitch_command, yaw_rate_command, thrust_command))

                self.cf.commander.send_setpoint(roll_command, pitch_command, yaw_rate_command, thrust_command)
                
                loop_rate.sleep()
        finally:
            self.perform_landing()

    def perform_landing(self):
        rospy.loginfo("Landing sequence initiated...")
        self.is_flying = False # Stop main loop if it was somehow still running
        
        # Gently reduce thrust over a few seconds
        landing_duration_s = 3.0
        start_landing_time = rospy.get_time()
        initial_thrust_for_landing = BIGQUAD_HOVER_THRUST_APPROX * 0.8 # Start a bit below hover
        if initial_thrust_for_landing < MIN_THRUST_BIGQUAD:
             initial_thrust_for_landing = MIN_THRUST_BIGQUAD + 1000

        while (rospy.get_time() - start_landing_time) < landing_duration_s:
            if rospy.is_shutdown(): break
            elapsed_ratio = (rospy.get_time() - start_landing_time) / landing_duration_s
            current_landing_thrust = initial_thrust_for_landing * (1.0 - elapsed_ratio)
            
            # Ensure thrust doesn't go below a very low value during ramp, then finally zero
            thrust_to_send = max(0, int(current_landing_thrust))
            if (rospy.get_time() - start_landing_time) > (landing_duration_s - 0.2): # Last 0.2s
                thrust_to_send = 0


            self.cf.commander.send_setpoint(0, 0, 0, thrust_to_send)
            rospy.loginfo_throttle(0.2, f"Landing thrust: {thrust_to_send}")
            time.sleep(0.02) # Send commands at ~50Hz during landing

        rospy.loginfo("Sending final stop setpoint to disarm motors.")
        self.cf.commander.send_stop_setpoint()
        time.sleep(0.1) # Allow command to be sent

def main():
    rospy.init_node('bigquad_hover_controller_node')
    cflib.crtp.init_drivers() # Initialize CRTP drivers

    rospy.loginfo(f"Connecting to Crazyflie at URI: {URI}")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf_instance = scf.cf
        rospy.loginfo("Successfully connected to Crazyflie.")

        # --- Pre-Flight Parameter Sanity Checks (Optional but good practice) ---
        # These parameters should have been set and SAVED via CFClient.
        # This is just to read and log them.
        # try:
            # Corrected parameter names based on our discussion
            # deck_enabled = cf_instance.param.get_value('deck.bcBigQuadEn')
            # esc_protocol = cf_instance.param.get_value('deck.bcBigQuadProto')
            # stab_controller = cf_instance.param.get_value('stabilizer.controller')
            # auto_arming_val = cf_instance.param.get_value('flightmode.stabModeYaw') # Placeholder, find actual auto_arming if exposed
                                                                               # Or infer from CONFIG_MOTORS_REQUIRE_ARMING
            
            # rospy.loginfo(f"Firmware Params: deck.bcBigQuadEn={deck_enabled}, deck.bcBigQuadProto={esc_protocol}, stabilizer.controller={stab_controller}")
            
            # if int(deck_enabled) != 1:
            #     rospy.logwarn("deck.bcBigQuadEn is NOT 1. Motors may not respond via BigQuad deck!")
            # if int(stab_controller) != 1:
            #     rospy.logwarn("stabilizer.controller is NOT 1. Offboard PID mode might not work as expected!")

        # except Exception as e:
        #     rospy.logwarn(f"Could not read all firmware parameters for verification: {e}")


        controller = BigQuadHoverController(cf_instance, GOAL_POSITION, GOAL_YAW_DEGREES)
        try:
            controller.run_hover_control()
        except KeyboardInterrupt:
            rospy.loginfo("KeyboardInterrupt received by main. Landing...")
            # controller.perform_landing() # run_hover_control's finally block will handle this
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred in main: {e}")
            # controller.perform_landing() # run_hover_control's finally block will handle this
        finally:
            rospy.loginfo("Shutting down main.")
            # Ensure motors are stopped if controller didn't land cleanly
            if hasattr(controller, 'is_flying') and controller.is_flying: # Check if controller and is_flying exist
                 rospy.logwarn("Main finally block: Controller was still marked as flying, attempting to land.")
                 controller.perform_landing() # Ensure landing is called
            elif hasattr(scf, 'cf'): # Check if scf and cf exist
                 rospy.logwarn("Main finally block: Sending stop setpoint as a final precaution.")
                 scf.cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    main()