import logging
import time
from scipy.spatial.transform import Rotation as R
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import rospy
from floaty_msgs.srv import set_cf_param_srv, set_flap_angle_srv, set_flaps_angles_srv, send_position_srv, send_orientation_srv
from std_srvs.srv import Empty

from floaty_msgs.msg import floaty_info_msg, floaty_flexible_msg
from geometry_msgs.msg import PoseStamped

import pandas as pd
import math

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

from cflib.utils.callbacks import Caller

import sys
import os
# This is to allow importing the quaternion.py file
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils.quaternions import quaternion_conjugate, quaternion_difference, quaternion_multiply, quaternion_to_euler, euler_to_quaternion


path_to_csv = "/home/floaty/Floaty/ws/src/floaty_pkg/data/floaty_estimator/estimation_data.csv"

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

floaty_info_publisher = None
floaty_flexible_info_publisher = None

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# sfloaty = None
sfloaty = SyncCrazyflie(uri)
connected = False
floaty_data_log_period = 20 # In ms
max_points_to_record = None
limit_reached = False

lg_stab = LogConfig(name='stateEstimate', period_in_ms=floaty_data_log_period)
lg_stab_vel = LogConfig(name='stateEstimate', period_in_ms=floaty_data_log_period)
lg_stab_orientation = LogConfig(name='stateEstimate', period_in_ms=floaty_data_log_period)
lg_control = LogConfig(name='motors_ctrp', period_in_ms=floaty_data_log_period)
lg_pwm = LogConfig(name='pwm', period_in_ms=100)
lg_motors_command = LogConfig(name='controller', period_in_ms=floaty_data_log_period)
lg_gyro = LogConfig(name='kalman', period_in_ms=floaty_data_log_period)
lg_gyro_filtered = LogConfig(name='kalman', period_in_ms=floaty_data_log_period)
lg_stab_rot = LogConfig(name='stateEstimate', period_in_ms=floaty_data_log_period)
lg_stab_rot_rate = LogConfig(name='stateEstimate', period_in_ms=floaty_data_log_period)
lg_stab_flaps = LogConfig(name='stateEstimate', period_in_ms=floaty_data_log_period)

lg_stab_uncertainty = LogConfig(name='Uncertainty', period_in_ms=floaty_data_log_period)

# Select data to send via flexible msg
flexible_msg_data_title = None
values_to_log = {}

# lg_int_pos_err = LogConfig(name='Error', period_in_ms=floaty_data_log_period)

sending_position_freq = 50.0
publish_data_freq = 50.0

# Frequency of dumping information about tasks
dump_tasks_freq = 0.0
send_yaw_freq = 10.0

# motor_log_freq = 1
# gyro_log_freq = 10

# estimation_log_freq = 10
# save_est_log_freq = 0.5

test_iter = 0.0

# Index
data_idx = 0

# Optitrack values
x_floaty = 0.0
y_floaty = 0.0
z_floaty = 0.0
qw_floaty = 1.0
qx_floaty = 0.0
qy_floaty = 0.0
qz_floaty = 0.0

# Log estimated position
values_to_log["est_x"] = 0.0
values_to_log["est_y"] = 0.0
values_to_log["est_z"] = 0.0
# Log estimated velocity
values_to_log["est_vx"] = 0.0
values_to_log["est_vy"] = 0.0
values_to_log["est_vz"] = 0.0
# Log estimated orientation
values_to_log["est_roll"] = 0.0
values_to_log["est_pitch"] = 0.0
values_to_log["est_yaw"] = 0.0
# Log estimated angular rates
values_to_log["est_roll_rate"] = 0.0
values_to_log["est_pitch_rate"] = 0.0
values_to_log["est_yaw_rate"] = 0.0
# Log estimated flap angle
values_to_log["est_f1"] = 0.0
values_to_log["est_f2"] = 0.0
values_to_log["est_f3"] = 0.0
values_to_log["est_f4"] = 0.0
# Log Optitrac position
values_to_log["Optitrack_x"] = 0.0
values_to_log["Optitrack_y"] = 0.0
values_to_log["Optitrack_z"] = 0.0
# Log Optitrac orientation
values_to_log["Optitrack_roll"] = 0.0
values_to_log["Optitrack_pitch"] = 0.0
values_to_log["Optitrack_yaw"] = 0.0
# Log gyro information
values_to_log["gyro_x"] = 0.0
values_to_log["gyro_y"] = 0.0
values_to_log["gyro_z"] = 0.0

# Log target position
values_to_log["target_x"] = 0.0
values_to_log["target_y"] = 0.0
values_to_log["target_z"] = 0.0
# Log target orientation
values_to_log["target_roll"] = 0.0
values_to_log["target_pitch"] = 0.0
values_to_log["target_yaw"] = 0.0
# Log estimated angular rates
# # Log estimated command values
# values_to_log["command_m1"] = 0.0
# values_to_log["command_m2"] = 0.0
# values_to_log["command_m3"] = 0.0
# values_to_log["command_m4"] = 0.0
# # Log estimated control values
# values_to_log["control_f1"] = 0.0
# values_to_log["control_f2"] = 0.0
# values_to_log["control_f3"] = 0.0
# values_to_log["control_f4"] = 0.0



# Traget values for floaty to track
err_x_floaty = 0.0
err_y_floaty = 0.0
err_z_floaty = 0.0
err_qw_floaty = 1.0
err_qx_floaty = 0.0
err_qy_floaty = 0.0
err_qz_floaty = 0.0

# Estimated values
est_x_floaty = 0.0
est_y_floaty = 0.0
est_z_floaty = 0.0

est_vx_floaty = 0.0
est_vy_floaty = 0.0
est_vz_floaty = 0.0

est_roll_floaty = 0.0
est_pitch_floaty = 0.0
est_yaw_floaty = 0.0

est_roll_rate_floaty = 0.0
est_pitch_rate_floaty = 0.0
est_yaw_rate_floaty = 0.0

est_f1 = 0.0
est_f2 = 0.0
est_f3 = 0.0
est_f4 = 0.0

est_qw_floaty = 1.0
est_qx_floaty = 0.0
est_qy_floaty = 0.0
est_qz_floaty = 0.0

control_f1 = 0.0
control_f2 = 0.0
control_f3 = 0.0
control_f4 = 0.0


# Other information
gyro_x = 0.0
gyro_y = 0.0
gyro_z = 0.0

gyro_filtered_x = 0.0
gyro_filtered_y = 0.0
gyro_filtered_z = 0.0

command_m1 = 0.0
command_m2 = 0.0
command_m3 = 0.0
command_m4 = 0.0

p_x_x = 0.0
p_x_vx = 0.0
p_vx_vx = 0.0

p_gx_gy = 0.0
p_gx_gz = 0.0
p_gy_gz = 0.0

goal_x = 0.0
goal_y = -0.0
# goal_z = 0.8
goal_z = 1.12
goal_z = 1.2

max_goal_targeting_rad = 0.1
min_goal_targeting_rad = 0.05
yaw_correction_range_size = max_goal_targeting_rad - min_goal_targeting_rad


target_yaw = 0

# # The frequency of the target yaw rotation
square_yaw = False
# square_yaw = True
# yaw_rotation_frequency = 0.0
yaw_rotation_frequency = 0.10
# yaw_rotation_amplitude = 1.55
yaw_rotation_amplitude = 0.0
override_yaw = False
override_yaw_val = 0 # -np.pi/2

# The frequency of the target x position
# square_x = False
square_x = True
x_frequency = 0.1
# x_amplitude = 0.25
x_amplitude = 0.0

# The frequency of the target y position
# square_y = False
square_y = True
y_frequency = 0.1
# y_amplitude = 0.16
y_amplitude = 0.0

# The frequency of the target z position
# square_z = True
square_z = False
z_frequency = 0.05
# z_amplitude = 0.2
z_amplitude = 0.0

# Boolian for sending position and orientation only when getting new data from Optitrack
gotNewData = False

# activate_goal_targeting = True
activate_goal_targeting = False

data_received_cb = Caller()

add_noise = True

# style.use('fivethirtyeight')

# fig = plt.figure()
# ax1 = fig.add_subplot(1,1,1)

# fig2 = plt.figure()
# ax2 = fig2.add_subplot(1,1,1)

def rotate_vector(x, y, z, roll, pitch, yaw, inverse=False):
    """
    Rotates a 3D vector (x, y, z) by the specified roll, pitch, and yaw angles.
    If inverse=True, it applies the inverse rotation to rotate back.
    
    Parameters:
    - x, y, z: float, the components of the vector in the original coordinate system.
    - roll: float, roll angle in radians.
    - pitch: float, pitch angle in radians.
    - yaw: float, yaw angle in radians.
    - inverse: bool, if True, applies the inverse rotation.
    
    Returns:
    - rotated_vector: numpy array, the rotated or inverse-rotated vector [x', y', z'].
    """
    # Original vector
    vector = np.array([x, y, z])

    # Create a rotation object from roll, pitch, and yaw using the 'xyz' order
    rotation = R.from_euler('xyz', [roll, pitch, yaw])

    # Apply the inverse rotation if specified
    if inverse:
        rotated_vector = rotation.inv().apply(vector)
    else:
        rotated_vector = rotation.apply(vector)

    return rotated_vector


def transform_euler_angles(original_angles, new_system_angles, sequence='xyz'):
    """
    Transforms a set of Euler angles from one coordinate system to another.
    
    Parameters:
    - original_angles: tuple/list of 3 floats, the roll, pitch, and yaw of the original rotation in radians.
    - new_system_angles: tuple/list of 3 floats, the roll, pitch, and yaw that define the new coordinate system in radians.
    - sequence: str, optional, the rotation order for Euler angles (e.g., 'xyz' or 'zyx').
    
    Returns:
    - transformed_angles: numpy array, Euler angles of the original rotation in the new coordinate system.
    """
    # Convert the original Euler angles to a rotation matrix
    R_orig = R.from_euler(sequence, original_angles)
    
    # Convert the new coordinate system's Euler angles to a rotation matrix
    R_new_basis = R.from_euler(sequence, new_system_angles)
    
    # Combine the rotations: new_basis * original
    R_transformed = R_new_basis * R_orig
    
    # Convert the result back to Euler angles
    transformed_angles = R_transformed.as_euler(sequence)
    
    return transformed_angles


# def animate(i):
#     est_xs = data_pd["Est_x"].to_list()
#     est_ys = data_pd["Est_y"].to_list()
#     est_zs = data_pd["Est_z"].to_list()

#     opt_xs = data_pd["Opt_x"].to_list()
#     opt_ys = data_pd["Opt_y"].to_list()
#     opt_zs = data_pd["Opt_z"].to_list()

#     x_axis = [i for i in range(len(est_xs))]

#     ax1.clear()
#     ax1.plot(x_axis,est_xs, 'g--')
#     ax1.plot(x_axis,opt_xs, 'g-')

#     ax1.plot(x_axis,est_ys, 'b--')
#     ax1.plot(x_axis,opt_ys, 'b-')

#     ax1.plot(x_axis,est_zs, 'r--')
#     ax1.plot(x_axis,opt_zs, 'r-')


# def animate2(i):
#     est_roll = data_pd["Est_roll"].to_list()
#     est_pitch = data_pd["Est_pitch"].to_list()
#     est_yaw = data_pd["Est_yaw"].to_list()

#     opt_roll = data_pd["Opt_roll"].to_list()
#     opt_ptich = data_pd["Opt_pitch"].to_list()
#     opt_yaw = data_pd["Opt_yaw"].to_list()

#     x_axis = [i for i in range(len(est_roll))]

#     ax2.clear()
#     ax2.plot(x_axis,est_roll, 'g--')
#     ax2.plot(x_axis,opt_roll, 'g-')

#     ax2.plot(x_axis,est_pitch, 'b--')
#     ax2.plot(x_axis,opt_ptich, 'b-')

#     ax2.plot(x_axis,est_yaw, 'r--')
#     ax2.plot(x_axis,opt_yaw, 'r-')

class Position:
  def __init__(self, x, y, z):
    self.x = x
    self.y = y
    self.z = z

class OrientationQuat:
  def __init__(self, qw, qx, qy, qz):
    self.qw = qw
    self.qx = qx
    self.qy = qy
    self.qz = qz

class OrientationAngs:
  def __init__(self, roll, pitch, yaw):
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw

class Pose:
  def __init__(self, position, orientation):
    self.position = position
    self.orientation = orientation

Goal_position = Position(goal_x, goal_y, goal_z)
# Goal_orientation = OrientationQuat(1, 0, 0, 0)
Goal_orientation = OrientationAngs(0, 0, 0)
Goal = Pose(Goal_position, Goal_orientation)


def move_angle_to_range(angle):
    rotated_ang = angle
    if angle > np.pi:
        rotated_ang = angle - 2*np.pi
    elif angle < -np.pi:
        rotated_ang = angle + 2*np.pi
    return rotated_ang

# def quaternion_to_euler(w, x, y, z):
    
#     # Calculate roll (x-axis rotation)
#     sinr_cosp = 2.0 * (w * x + y * z)
#     cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)

#     # Calculate pitch (y-axis rotation)
#     sinp = 2.0 * (w * y - z * x)
#     if np.abs(sinp) >= 1:
#         pitch = np.copysign(np.pi / 2, sinp)  # Use ±π/2 if out of range
#     else:
#         pitch = np.arcsin(sinp)

#     # Calculate yaw (z-axis rotation)
#     siny_cosp = 2.0 * (w * z + x * y)
#     cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

# def euler_to_quaternion(roll, pitch, yaw):
#     # Calculate quaternion components
#     cy = np.cos(yaw * 0.5)
#     sy = np.sin(yaw * 0.5)
#     cp = np.cos(pitch * 0.5)
#     sp = np.sin(pitch * 0.5)
#     cr = np.cos(roll * 0.5)
#     sr = np.sin(roll * 0.5)

#     # Calculate quaternion components
#     w = cr * cp * cy + sr * sp * sy
#     x = sr * cp * cy - cr * sp * sy
#     y = cr * sp * cy + sr * cp * sy
#     z = cr * cp * sy - sr * sp * cy

#     return w, x, y, z


def calc_target_pos():
    global err_x_floaty, err_y_floaty, err_z_floaty, err_qw_floaty, err_qx_floaty, err_qy_floaty, err_qz_floaty, target_yaw, goal_z
    global Goal
    t = time.time()
    # target_yaw = (t*math.pi*2*yaw_rotation_frequency)%(2*math.pi)
    if square_yaw:
        target_yaw = (np.sin(t*math.pi*2*yaw_rotation_frequency)>0)*yaw_rotation_amplitude
    else:
        target_yaw = np.sin(t*math.pi*2*yaw_rotation_frequency)*yaw_rotation_amplitude
    if override_yaw:
        target_yaw = override_yaw_val
    
    # target_yaw = -np.pi/2
    w, x, y, z = euler_to_quaternion(0, 0, target_yaw)
    target_q = np.array([w, x, y, z])
    current_q = np.array([qw_floaty, qx_floaty, qy_floaty, qz_floaty])
    err_q = quaternion_difference(target_q, current_q)
    err_qw_floaty, err_qx_floaty, err_qy_floaty, err_qz_floaty = err_q


    if square_x:
        Goal.position.x = goal_x + (np.sin(t*math.pi*2*x_frequency)>0)*x_amplitude -x_amplitude/2
    else:
        Goal.position.x = goal_x + np.sin(t*math.pi*2*x_frequency)*x_amplitude

    if square_y:
        Goal.position.y = goal_y + (np.sin(t*math.pi*2*y_frequency)>0)*y_amplitude - y_amplitude/2 # -(3*y_amplitude/4)
    else:
        Goal.position.y = goal_y + np.sin(t*math.pi*2*y_frequency)*y_amplitude

    if square_z:
        goal_z = 1.1
        Goal.position.z = goal_z + (np.sin(t*math.pi*2*z_frequency)>0)*z_amplitude #-z_amplitude/2
    else:
        Goal.position.z = goal_z + np.sin(t*math.pi*2*z_frequency)*z_amplitude
    # Goal.position.z = goal_z + np.sin(t*math.pi*2*z_frequency)*z_amplitude

    Goal.orientation.yaw = target_yaw
    
    values_to_log['target_x'] = Goal.position.x
    values_to_log['target_y'] = Goal.position.y
    values_to_log['target_z'] = Goal.position.z
    
    values_to_log['target_roll'] = Goal.orientation.roll
    values_to_log['target_pitch'] = Goal.orientation.pitch
    values_to_log['target_yaw'] = Goal.orientation.yaw



def simple_log(scf, logconf):

    with SyncLogger(scf, logconf) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]

            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

            break

def update_pose_callback(req):
    global x_floaty, y_floaty, z_floaty, qw_floaty, qx_floaty, qy_floaty, qz_floaty, err_x_floaty, err_y_floaty, err_z_floaty
    global target_yaw
    global gotNewData

    gotNewData = True
    qw_floaty = req.pose.orientation.w
    qx_floaty = req.pose.orientation.x
    qy_floaty = req.pose.orientation.y
    qz_floaty = req.pose.orientation.z

    # shifted_x = req.pose.position.x +0.01
    # shifted_y = req.pose.position.y -0.05
    # shifted_z = req.pose.position.z -0.56

    calc_target_pos()

    err_x = req.pose.position.x - Goal.position.x
    err_y = req.pose.position.y - Goal.position.y
    err_z = req.pose.position.z - Goal.position.z

    x_floaty = err_x
    y_floaty = err_y
    z_floaty = err_z

    roll, pitch, yaw = get_angs_from_quat(qw_floaty, qx_floaty, qy_floaty, qz_floaty)

    values_to_log['Optitrack_x'] = req.pose.position.x
    values_to_log['Optitrack_y'] = req.pose.position.y
    values_to_log['Optitrack_z'] = req.pose.position.z

    values_to_log['Optitrack_roll'] = roll
    values_to_log['Optitrack_pitch'] = pitch
    values_to_log['Optitrack_yaw'] = yaw

    # -------------------------------------------------------------------------------
    # # It is necessary to rotate the position error to face the target yaw 
    # # We move it to the target yaw and not the real yaw as the rest of the rotation 
    # # is calculated on floaty

    # rotate to goal frame  
    rotate_error = rotate_vector(err_x, err_y, err_z, Goal.orientation.roll, Goal.orientation.pitch, Goal.orientation.yaw, inverse=True)
    # err_x_floaty = err_x*np.cos(target_yaw) + err_y*np.sin(target_yaw)
    # err_y_floaty = err_y*np.cos(target_yaw) - err_x*np.sin(target_yaw)
    # err_z_floaty = err_z

    err_x_floaty = rotate_error[0]
    err_y_floaty = rotate_error[1]
    err_z_floaty = rotate_error[2]
    # -------------------------------------------------------------------------------


    # x_floaty = (np.abs(err_x)-0.03)**2*10*np.sign(err_x)
    # y_floaty = (np.abs(err_y)-0.03)**2*10*np.sign(err_y)


    # The idea is to have a small radius around the goal position (min_goal_targeting_rad), 
    # if floaty was inside this circle, no yaw correction is needed, if it was outside it, 
    # but still inside a larger circle, then the yaw correction is done gradually. If Floaty
    # is outside the larger circle, then the robot target yaw is towards the goal position
    
    if activate_goal_targeting:
        roll, pitch, yaw = quaternion_to_euler(qw_floaty, qx_floaty, qy_floaty, qz_floaty)
        distance_to_goal = np.sqrt(err_y**2+err_x**2)
        if distance_to_goal > min_goal_targeting_rad:
            error_distance = distance_to_goal - min_goal_targeting_rad
            yaw_to_goal = move_angle_to_range(np.arctan2(-err_y, -err_x) + np.pi/2)
            # If the yaw rotation towards the goal is larger than pi/2 then we flip the angle 
            if abs(yaw_to_goal - yaw) > np.pi/2:
                yaw_to_goal = move_angle_to_range(yaw_to_goal + np.pi)
            if error_distance <= max_goal_targeting_rad:
                yaw_correction_ratio = error_distance/yaw_correction_range_size
                change_in_target_yaw = (yaw_to_goal - yaw)*yaw_correction_ratio
                target_yaw = yaw + change_in_target_yaw
            else:
                target_yaw = yaw_to_goal


            
            # if abs(change_in_target_yaw) > np.pi/2
        # if np.sqrt(err_y**2+err_x**2) > max_goal_targeting_rad:
        #     target_yaw_1 = move_angle_to_range(np.arctan2(-err_y, -err_x))
        #     err_1 = np.abs(target_yaw - target_yaw_1)
        #     if err_1 > np.pi/2:
        #         target_yaw = move_angle_to_range(target_yaw_1 + np.pi)
        #     else:
        #         target_yaw = target_yaw_1


    # # Rotated frame
    # roll, pitch, yaw = quaternion_to_euler(qw_floaty, qx_floaty, qy_floaty, qz_floaty)

    # rotated_x = shifted_x*np.cos(yaw) + shifted_y*np.sin(yaw)
    # rotated_y = shifted_y*np.cos(yaw) - shifted_x*np.sin(yaw)

    # x_floaty = rotated_x
    # y_floaty = rotated_y
    # z_floaty = shifted_z
    

    # # 90 degrees
    # qw_floaty = 0.707
    # qx_floaty = 0
    # qy_floaty = 0
    # qz_floaty = 0.707

    # # 30 degrees
    # qw_floaty = 0.9659
    # qx_floaty = 0
    # qy_floaty = 0
    # qz_floaty = 0.2588


    # Yaw_shift = 2
    # roll, pitch, yaw = quaternion_to_euler(qw_floaty, qx_floaty, qy_floaty, qz_floaty)
    # yaw = (yaw+3.14 - Yaw_shift)%(2*3.14)-3.14
    # qw_floaty, qx_floaty, qy_floaty, qz_floaty = euler_to_quaternion(roll, pitch, yaw)
    # qw_floaty = 1
    # qx_floaty = 0
    # qy_floaty = 0
    # qz_floaty = 0

    return 1
    

def set_flap_angle_callback(req):
    flap_id = req.flap_id
    angle = req.angle


    if flap_id<1 or flap_id>4:
        rospy.logerr("Motor id not correct")
        return
    
    ns = rospy.get_namespace()
    # flaps_range = rospy.get_param(ns+"/flaps_range")
    flaps_range = rospy.get_param("floaty/flaps_range",default=120)
    angle_to_param = 65535/flaps_range
    param_val = angle_to_param*angle

    if param_val < 1 or param_val > 65534:
        rospy.logerr("Flap angle out of range")
        return
        
    group_name = "extModeControl"
    param_name = "motVal"+str(flap_id)
    set_param_value(sfloaty, group_name, param_name, param_val)
    param_name = "ctrType"+str(flap_id)
    set_param_value(sfloaty, group_name, param_name, 1)
    return 


def set_flaps_angles_callback(req):
    angles = req.angles

    ns = rospy.get_namespace()
    # flaps_range = rospy.get_param(ns+"/flaps_range")
    flaps_range = rospy.get_param("floaty/flaps_range",default=120)

    group_name = "extModeControl"

    for i in range(4):
        angle_to_param = 65535/flaps_range
        param_val = angle_to_param*angles[i]
        if param_val < 1 or param_val > 65534:
            rospy.logerr("Flap {id} angle out of range".format(i))
            return
        
        param_name = "motVal"+str(i+1)
        set_param_value(sfloaty, group_name, param_name, param_val)
        param_name = "ctrType"+str(i+1)
        set_param_value(sfloaty, group_name, param_name, 1)
    return True
        

def set_param_value(scf, groupstr, namestr, value):
    cf = scf.cf
    full_name = groupstr+ "." +namestr
    cf.param.set_value(full_name,value)
    return 1


def set_cf_param_callback(req):
    return set_param_value(sfloaty, req.group_name, req.param_name, req.value)
        

# def send_pose(scf, pos, quat):
#     cf = scf.cf
#     cf.extpos.send_extpose(pos.x, pos.y, pos.y, quat.x, quat.y, quat.z, quat.w)
#     return 1

def send_position_to_floaty(event):
    global err_x_floaty, err_y_floaty, err_z_floaty, err_qw_floaty, err_qx_floaty, err_qy_floaty, err_qz_floaty
    if not connected:
        print("Not connected yet!")
        return
    # cf = scf.cf
    global sfloaty
    global gotNewData

    if gotNewData:
        cf = sfloaty.cf
        cf.extpos.send_extpose(err_x_floaty, err_y_floaty, err_z_floaty, err_qx_floaty, err_qy_floaty, err_qz_floaty, err_qw_floaty)
        gotNewData = False
        # print(f"Data sent\nX : {err_x_floaty}\nY : {err_y_floaty}\nZ : {err_z_floaty}")
    else:
        print("Didn't recieve new data!")
    # print("sending pos: x: {}, y: {}, z: {}".format(x_floaty, y_floaty, z_floaty))
    # print("Sending")
    return

  
def send_pose_callback(req):
    # cf = scf.cf
    # cf.extpos.send_extpose(req.x, req.y, req.z, req.qx, req.qy, req.qz, req.qw)
    # # cf.extpos.send_extpos(req.x, req.y, req.z)
    # # send_pose(sfloaty, req.pose.position, req.pose.orientation)
    # rospy.loginfo("sent a position to Floaty")
    global err_x_floaty, err_y_floaty, err_z_floaty, err_qw_floaty, err_qx_floaty, err_qy_floaty, err_qz_floaty
    global gotNewData
    
    gotNewData = True
    
    err_x_floaty = req.x
    err_y_floaty = req.y
    err_z_floaty = req.z
    err_qw_floaty = req.qw
    err_qx_floaty = req.qx
    err_qy_floaty = req.qy
    err_qz_floaty = req.qz


    # if add_noise:
    #     x_floaty = x_floaty + np.random.normal(0,0.5)
    #     y_floaty = y_floaty + np.random.normal(0,0.5)
    #     z_floaty = z_floaty + np.random.normal(0,0.5)

    # qw_floaty = 1
    # qx_floaty = 0
    # qy_floaty = 0
    # qz_floaty = 0
    return 1


def send_orientation_callback(req):
    global err_qw_floaty, err_qx_floaty, err_qy_floaty, err_qz_floaty
    global gotNewData
    
    gotNewData = True

    roll = req.roll
    pitch = req.pitch
    yaw = req.yaw

    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    err_qw_floaty = cr * cp * cy + sr * sp * sy
    err_qx_floaty = sr * cp * cy - cr * sp * sy
    err_qy_floaty = cr * sp * cy + sr * cp * sy
    err_qz_floaty = cr * cp * sy - sr * sp * cy
    

    return 1


def send_yaw(event):
    set_param_value(sfloaty, 'extCtrl', 'target_yaw', target_yaw)
    # print("Target yaw changed: " + str(target_yaw))



# ---------------------------------------------------
# -------------- Log configs callbacks --------------
# ---------------------------------------------------
def lg_stab_cb(timestamp, data, logconf) -> None:
    global est_x_floaty, est_y_floaty, est_z_floaty
    est_x_floaty = data["stateEstimate.x"]
    est_y_floaty = data["stateEstimate.y"]
    est_z_floaty = data["stateEstimate.z"]
    #  data_received_cb.call([est_x_floaty, est_y_floaty, est_z_floaty])


def lg_stab_vel_cb(timestamp, data, logconf) -> None:
    global est_vx_floaty, est_vy_floaty, est_vz_floaty
    est_vx_floaty = data["stateEstimate.vx"]
    est_vy_floaty = data["stateEstimate.vy"]
    est_vz_floaty = data["stateEstimate.vz"]


# def lg_stab_uncertainty_cb(timestamp, data, logconf) -> None:
#     global p_x_x, p_x_vx, p_vx_vx
#     p_x_x = data["Uncertainty.P_x_x"]
#     p_x_vx = data["Uncertainty.P_x_vx"]
#     p_vx_vx = data["Uncertainty.P_vx_vx"]


def lg_stab_uncertainty_cb(timestamp, data, logconf) -> None:
    global p_gx_gy, p_gx_gz, p_gy_gz
    p_gx_gy = data["Uncertainty.P_gx_gy"]
    p_gx_gz = data["Uncertainty.P_gx_gz"]
    p_gy_gz = data["Uncertainty.P_gy_gz"]


def lg_stab_rot_cb(timestamp, data, logconf) -> None:
    global est_roll_floaty, est_pitch_floaty, est_yaw_floaty
    est_roll_floaty = data["stateEstimate.roll"]
    est_pitch_floaty = data["stateEstimate.pitch"]
    est_yaw_floaty = data["stateEstimate.yaw"]


def lg_stab_rot_rate_cb(timestamp, data, logconf) -> None:
    global est_roll_rate_floaty, est_pitch_rate_floaty, est_yaw_rate_floaty
    est_roll_rate_floaty = data["stateEstimate.roll_rate"]
    est_pitch_rate_floaty = data["stateEstimate.pitch_rate"]
    est_yaw_rate_floaty = data["stateEstimate.yaw_rate"]


def lg_stab_orientation_cb(timestamp, data, logconf) -> None:
    global est_qw_floaty, est_qx_floaty, est_qy_floaty, est_qz_floaty
    est_qw_floaty = data["stateEstimate.qw"]
    est_qx_floaty = data["stateEstimate.qx"]
    est_qy_floaty = data["stateEstimate.qy"]
    est_qz_floaty = data["stateEstimate.qz"]


def lg_control_cb(timestamp, data, logconf) -> None:
    global control_f1, control_f2, control_f3, control_f4
    control_f1 = data["controller.m1"]
    control_f2 = data["controller.m2"]
    control_f3 = data["controller.m3"]
    control_f4 = data["controller.m4"]


def lg_motors_command_cb(timestamp, data, logconf) -> None:
    global command_m1, command_m2, command_m3, command_m4
    command_m1 = data["motors_ctrp.m1"]
    command_m2 = data["motors_ctrp.m2"]
    command_m3 = data["motors_ctrp.m3"]
    command_m4 = data["motors_ctrp.m4"]


def lg_stab_flaps_cb(timestamp, data, logconf) -> None:
    global est_f1, est_f2, est_f3, est_f4
    est_f1 = data["stateEstimate.flap_1"]
    est_f2 = data["stateEstimate.flap_2"]
    est_f3 = data["stateEstimate.flap_3"]
    est_f4 = data["stateEstimate.flap_4"]


def lg_gyro_cb(timestamp, data, logconf) -> None:
    global gyro_x, gyro_y, gyro_z
    gyro_x = data["kalman.gyroMeasX"]
    gyro_y = data["kalman.gyroMeasY"]
    gyro_z = data["kalman.gyroMeasZ"]


def lg_gyro_filtered_cb(timestamp, data, logconf) -> None:
    global gyro_filtered_x, gyro_filtered_y, gyro_filtered_z
    gyro_filtered_x = data["kalman.gyroFilteredX"]
    gyro_filtered_y = data["kalman.gyroFilteredY"]
    gyro_filtered_z = data["kalman.gyroFilteredZ"]
    

# # ---------------------------------
# # For testing the error integration
# # Here I am using the position variables to store the error
# def lg_int_pos_err_cb(timestamp, data, logconf) -> None:
#     global est_x_floaty, est_y_floaty, est_z_floaty
#     est_x_floaty = data["Error.int_x_err"]
#     est_y_floaty = data["Error.int_y_err"]
#     est_z_floaty = data["Error.int_z_err"]

# ------------------------------------------------------------
# ---------------- Print data on the terminal ----------------
# ------------------------------------------------------------
def log_params_callback(event):

    with SyncLogger(sfloaty, lg_stab) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break

    rospy.sleep(0.005)
    with SyncLogger(sfloaty, lg_stab_vel) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break

    # with SyncLogger(sfloaty, lg_stab_orientation) as logger:
    with SyncLogger(sfloaty, lg_stab_rot) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break

    # with SyncLogger(sfloaty, lg_stab_orientation) as logger:
    with SyncLogger(sfloaty, lg_stab_rot_rate) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break

    with SyncLogger(sfloaty, lg_gyro) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break

    with SyncLogger(sfloaty, lg_gyro_filtered) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break

    with SyncLogger(sfloaty, lg_stab_flaps) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break

# -------------------------------------------------------------
# ------------- Writing recieved data onto a file -------------
# -------------------------------------------------------------
def log_data(event):
    # print("log_data function called")
    global data_pd
    keys = data_pd.keys()
    vals = []
    
    # ---------------------------------------------
    # Get position from the estimtator
    vals.append(est_x_floaty)
    vals.append(est_y_floaty)
    vals.append(est_z_floaty)
    
    # ---------------------------------------------
    # Getvelocity from the estimtator
    vals.append(est_vx_floaty)
    vals.append(est_vy_floaty)
    vals.append(est_vz_floaty)
    
    # ---------------------------------------------
    # Get angles from the estimtator
    vals.append(est_roll_floaty)
    vals.append(est_pitch_floaty)
    vals.append(est_yaw_floaty)

    # ---------------------------------------------
    # Get angluar rates from the estimtator
    vals.append(est_roll_rate_floaty)
    vals.append(est_pitch_rate_floaty)
    vals.append(est_yaw_rate_floaty)

    # ---------------------------------------------
    # Get flaps values from the estimtator
    vals.append(est_f1)
    vals.append(est_f2)
    vals.append(est_f3)
    vals.append(est_f4)

    # ---------------------------------------------
    # Get position and angles the Optitrack
    vals.append(x_floaty)
    vals.append(y_floaty)
    vals.append(z_floaty)
    roll, pitch, yaw = get_angs_from_quat(qw_floaty, qx_floaty, qy_floaty, qz_floaty)
    vals.append(roll)
    vals.append(pitch)
    vals.append(yaw)

    # ---------------------------------------------
    # Get control values from the estimtator
    vals.append(control_f1)
    vals.append(control_f2)
    vals.append(control_f3)
    vals.append(control_f4)

    dect_row = {}
    for i,val in enumerate(vals):
        dect_row[keys[i]]=vals[i]

    # data_pd_row = pd.DataFrame.from_dict(dect_row)
    data_pd = data_pd.append(dect_row, ignore_index=True)


# # ------------------------------------------------------------
# # ------------- Publish recieved data to a topic -------------
# # ------------------------------------------------------------
# def publish_data(event):
#     global data_idx, floaty_info_publisher
#     # print("publishing")
#     msg = floaty_info_msg()

#     msg.idx = data_idx

#     # ---------------------------------------------
#     # Get position from the estimtator
#     msg.Est_x = est_x_floaty
#     msg.Est_y = est_y_floaty
#     msg.Est_z = est_z_floaty

#     # ---------------------------------------------
#     # Getvelocity from the estimtator
#     msg.Est_vx = est_vx_floaty
#     msg.Est_vy = est_vy_floaty
#     msg.Est_vz = est_vz_floaty
    
#     # ---------------------------------------------
#     # Get angles from the estimtator
#     msg.Est_roll = est_roll_floaty
#     msg.Est_pitch = est_pitch_floaty
#     msg.Est_yaw = est_yaw_floaty

#     # ---------------------------------------------
#     # Get angluar rates from the estimtator
#     msg.Est_roll_rate = est_roll_rate_floaty
#     msg.Est_pitch_rate = est_pitch_rate_floaty
#     msg.Est_yaw_rate = est_yaw_rate_floaty

#     # msg.Est_roll_rate = gyro_filtered_x
#     # msg.Est_pitch_rate = gyro_filtered_y
#     # msg.Est_yaw_rate = gyro_filtered_z
    
#     # ---------------------------------------------
#     # Get flaps values from the estimtator
#     msg.Est_f1 = est_f1
#     msg.Est_f2 = est_f2
#     msg.Est_f3 = est_f3
#     msg.Est_f4 = est_f4

#     # ---------------------------------------------
#     # Get position and angles the Optitrack
#     msg.Opt_x = x_floaty
#     msg.Opt_y = y_floaty
#     msg.Opt_z = z_floaty
    
#     roll, pitch, yaw = get_angs_from_quat(qw_floaty, qx_floaty, qy_floaty, qz_floaty)
#     msg.Opt_roll = roll
#     msg.Opt_pitch = pitch
#     msg.Opt_yaw = yaw

#     # ---------------------------------------------
#     # Get gyro measurments
#     msg.gyro_x = gyro_x
#     msg.gyro_y = gyro_y
#     msg.gyro_z = gyro_z

#     # ---------------------------------------------
#     # Get commands that the motors get
#     msg.Command_m1 = command_m1
#     msg.Command_m2 = command_m2
#     msg.Command_m3 = command_m3
#     msg.Command_m4 = command_m4

#     # ---------------------------------------------
#     # Get control values from the estimtator
#     msg.Control_f1 = control_f1
#     msg.Control_f2 = control_f2
#     msg.Control_f3 = control_f3
#     msg.Control_f4 = control_f4


#     # # # ---------------------------------------------
#     # # # Get uncertainty from the estimtator (overwrite position)
#     # msg.Est_x = p_gx_gy
#     # msg.Est_y = p_gx_gz
#     # msg.Est_z = p_gy_gz


#     # # # ---------------------------------------------
#     # # # Get the target Yaw  and store it in the gyro message
#     # msg.gyro_x = p_gx_gy
#     # msg.gyro_x = p_gx_gz
#     msg.gyro_x = target_yaw


#     floaty_info_publisher.publish(msg)
    
#     # Increase the global index
#     data_idx = data_idx+1


# ------------------------------------------------------------
# ------------- Publish recieved data to a topic -------------
# ------------------------------------------------------------
def publish_flexible_data(event):
    global data_idx, floaty_flexible_info_publisher, max_points_to_record, limit_reached
    # print("publishing")
    
    if(data_idx>max_points_to_record):
        if limit_reached:
            print("Max data limit reached")
            limit_reached = True
        exit()
    msg = floaty_flexible_msg()

    msg.idx = data_idx
    msg.title = flexible_msg_data_title



    # values_to_log['est_x'] = est_x_floaty
    # values_to_log['est_y'] = est_y_floaty
    # values_to_log['est_z'] = est_z_floaty

    rotate_error = rotate_vector(est_x_floaty, est_y_floaty, est_z_floaty, Goal.orientation.roll, Goal.orientation.pitch, Goal.orientation.yaw)

    # This way, we rotate the estimation back to the absolute system
    values_to_log['est_x'] = rotate_error[0] + Goal.position.x
    values_to_log['est_y'] = rotate_error[1] + Goal.position.y
    values_to_log['est_z'] = rotate_error[2] + Goal.position.z

    # values_to_log['est_vx'] = est_vx_floaty
    # values_to_log['est_vy'] = est_vy_floaty
    # values_to_log['est_vz'] = est_vz_floaty

    rotate_error = rotate_vector(est_vx_floaty, est_vy_floaty, est_vz_floaty, Goal.orientation.roll, Goal.orientation.pitch, Goal.orientation.yaw)

    # This way, we rotate the estimation back to the absolute system
    values_to_log['est_vx'] = est_vx_floaty
    values_to_log['est_vy'] = est_vy_floaty
    values_to_log['est_vz'] = est_vz_floaty

    floaty_angles = (est_roll_floaty, est_pitch_floaty, est_yaw_floaty)
    target_angles = (Goal.orientation.roll, Goal.orientation.pitch, Goal.orientation.yaw)

    transformed_angles = transform_euler_angles(floaty_angles, target_angles, sequence='xyz')

    # This is without rotating back to target system
    # values_to_log['est_roll'] = est_roll_floaty
    # values_to_log['est_pitch'] = est_pitch_floaty
    # values_to_log['est_yaw'] = est_yaw_floaty

    values_to_log['est_roll'] = transformed_angles[0]
    values_to_log['est_pitch'] = transformed_angles[1]
    values_to_log['est_yaw'] = transformed_angles[2]

    values_to_log['est_roll_rate'] = est_roll_rate_floaty
    values_to_log['est_pitch_rate'] = est_pitch_rate_floaty
    values_to_log['est_yaw_rate'] = est_yaw_rate_floaty

    values_to_log["est_f1"] = est_f1
    values_to_log["est_f2"] = est_f2
    values_to_log["est_f3"] = est_f3
    values_to_log["est_f4"] = est_f4

    values_to_log['gyro_x'] = gyro_x
    values_to_log['gyro_y'] = gyro_y
    values_to_log['gyro_z'] = gyro_z

    values_to_log['command_f1'] = command_m1
    values_to_log['command_f2'] = command_m2
    values_to_log['command_f3'] = command_m3
    values_to_log['command_f4'] = command_m4

    values_to_log['control_f1'] = control_f1
    values_to_log['control_f2'] = control_f2
    values_to_log['control_f3'] = control_f3
    values_to_log['control_f4'] = control_f4

    msg.names = values_to_log.keys()
    msg.values = values_to_log.values()

    floaty_flexible_info_publisher.publish(msg)
    
    # Increase the global index
    data_idx = data_idx+1



def dump_tasks_info(event):
    set_param_value(sfloaty, 'system', 'taskDump', '1')


def save_est_log(event):
    data_pd.to_csv(path_to_csv)


# def log_angs(event):
#     roll, pitch, yaw = get_angs_from_quat(qw_floaty, qx_floaty, qy_floaty, qz_floaty)
#     print("Roll: {}, Pitch: {}, Yaw: {}".format(roll, pitch, yaw))


def get_angs_from_quat(qw, qx, qy, qz):
    roll = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
    pitch = math.asin(-2.0*(qx*qz - qw*qy))
    yaw = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
    return roll, pitch, yaw



def log_motor(event):
    if not connected:
        print("Not connected yet!")
        return
    with SyncLogger(sfloaty, lg_control) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break


def log_gyro(event):
    with SyncLogger(sfloaty, lg_gyro) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            # logconf_name = log_entry[2]

            print('[%d]: %s' % (timestamp, data))

            break


def flap_est_test_function(event):
    global test_iter
    val = str(math.sin(test_iter/40*2*3.14)*0.5)
    set_param_value(sfloaty, 'extCtrl', 'm1', val)
    test_iter = test_iter+1
    return

def test_fun(event):
    print("Timers running!")

def callback_console_incoming(console_text):
    print(console_text, end='')

def callback_connected(link_uri):
    """ This callback is called form the Crazyflie API when a Crazyflie
    has been connected and the TOCs have been downloaded."""
    print('Connected to %s' % (link_uri))

def callback_connection_failed(link_uri, msg):
    """Callback when connection initial connection fails (i.e no Crazyflie
    at the specified address)"""
    print('Connection to %s failed: %s' % (link_uri, msg))

def callback_connection_lost(link_uri, msg):
    """Callback when disconnected after a connection has been made (i.e
    Crazyflie moves out of range)"""
    print('Connection to %s lost: %s' % (link_uri, msg))

def callback_disconnected(link_uri):
    """Callback when the Crazyflie is disconnected (called in all cases)"""
    print('Disconnected from %s' % link_uri)


if __name__ == '__main__':
    # global sfloaty
    # Initialize the low-level drivers
    try:
        rospy.init_node("crazyradio_node")
        cflib.crtp.init_drivers()

        lg_stab.add_variable('stateEstimate.x', 'float')
        lg_stab.add_variable('stateEstimate.y', 'float')
        lg_stab.add_variable('stateEstimate.z', 'float')

        lg_stab_vel.add_variable('stateEstimate.vx', 'float')
        lg_stab_vel.add_variable('stateEstimate.vy', 'float')
        lg_stab_vel.add_variable('stateEstimate.vz', 'float')

        # lg_stab_uncertainty.add_variable('Uncertainty.P_x_x', 'float')
        # lg_stab_uncertainty.add_variable('Uncertainty.P_x_vx', 'float')
        # lg_stab_uncertainty.add_variable('Uncertainty.P_vx_vx', 'float')

        lg_stab_uncertainty.add_variable('Uncertainty.P_gx_gy', 'float')
        lg_stab_uncertainty.add_variable('Uncertainty.P_gx_gz', 'float')
        lg_stab_uncertainty.add_variable('Uncertainty.P_gy_gz', 'float')

        lg_stab_rot.add_variable('stateEstimate.roll', 'float')
        lg_stab_rot.add_variable('stateEstimate.pitch', 'float')
        lg_stab_rot.add_variable('stateEstimate.yaw', 'float')

        lg_stab_rot_rate.add_variable('stateEstimate.roll_rate', 'float')
        lg_stab_rot_rate.add_variable('stateEstimate.pitch_rate', 'float')
        lg_stab_rot_rate.add_variable('stateEstimate.yaw_rate', 'float')

        lg_stab_orientation.add_variable('stateEstimate.qx', 'float')
        lg_stab_orientation.add_variable('stateEstimate.qy', 'float')
        lg_stab_orientation.add_variable('stateEstimate.qz', 'float')
        lg_stab_orientation.add_variable('stateEstimate.qw', 'float')

        lg_stab_flaps.add_variable('stateEstimate.flap_1', 'float')
        lg_stab_flaps.add_variable('stateEstimate.flap_2', 'float')
        lg_stab_flaps.add_variable('stateEstimate.flap_3', 'float')
        lg_stab_flaps.add_variable('stateEstimate.flap_4', 'float')

        lg_control.add_variable('controller.m1', 'float')
        lg_control.add_variable('controller.m2', 'float')
        lg_control.add_variable('controller.m3', 'float')
        lg_control.add_variable('controller.m4', 'float')

        lg_motors_command.add_variable('motors_ctrp.m1', 'float')
        lg_motors_command.add_variable('motors_ctrp.m2', 'float')
        lg_motors_command.add_variable('motors_ctrp.m3', 'float')
        lg_motors_command.add_variable('motors_ctrp.m4', 'float')

        lg_pwm.add_variable('motor.m1', 'float')
        lg_pwm.add_variable('motor.m2', 'float')
        lg_pwm.add_variable('motor.m3', 'float')
        lg_pwm.add_variable('motor.m4', 'float')

        lg_gyro.add_variable('kalman.gyroMeasX', 'float')
        lg_gyro.add_variable('kalman.gyroMeasY', 'float')
        lg_gyro.add_variable('kalman.gyroMeasZ', 'float')

        lg_gyro_filtered.add_variable('kalman.gyroFilteredX', 'float')
        lg_gyro_filtered.add_variable('kalman.gyroFilteredY', 'float')
        lg_gyro_filtered.add_variable('kalman.gyroFilteredZ', 'float')


        # # ---------------------------------
        # # For testing the error integration
        # lg_int_pos_err.add_variable('Error.int_x_err', 'float')
        # lg_int_pos_err.add_variable('Error.int_y_err', 'float')
        # lg_int_pos_err.add_variable('Error.int_z_err', 'float')


        set_parameters_value_srv_name = rospy.get_param("/crazyradio/set_parameters_value_srv_name", "set_parameters_value")
        send_position_srv_name = rospy.get_param("/crazyradio/send_position_srv_name", "send_position")
        floaty_info_publish_topic_name = rospy.get_param("/crazyradio/floaty_info_publish_topic_name", "floaty_info")
        floaty_flexible_info_publish_topic_name = rospy.get_param("/crazyradio/floaty_flexible_info_publish_topic_name", "flexible_floaty_info")
        flexible_msg_data_title = rospy.get_param("/crazyradio/flexible_msg_data_title")
        max_points_to_record = rospy.get_param("/crazyradio/max_points_to_record")
        
        radio_srvice = rospy.Service(set_parameters_value_srv_name, set_cf_param_srv, set_cf_param_callback)
        set_flap_angle_srvice = rospy.Service('set_flap_angle', set_flap_angle_srv, set_flap_angle_callback)
        set_flaps_angles_srvice = rospy.Service('set_flaps_angles', set_flaps_angles_srv, set_flaps_angles_callback)
        send_srvice = rospy.Service(send_position_srv_name, send_position_srv, send_pose_callback)
        send_srvice = rospy.Service("send_orientation", send_orientation_srv, send_orientation_callback)
        send_srvice = rospy.Service("log_params", Empty, log_params_callback)

        pos_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, update_pose_callback)

        floaty_info_publisher = rospy.Publisher(floaty_info_publish_topic_name, floaty_info_msg, queue_size=10)
        floaty_flexible_info_publisher = rospy.Publisher(floaty_flexible_info_publish_topic_name, floaty_flexible_msg, queue_size=10)

    except rospy.ROSInterruptException:
        rospy.logerr("Error in radio code")


    cf=Crazyflie(rw_cache='./cache')
    
    cf.connected.add_callback(callback_connected)
    cf.disconnected.add_callback(callback_disconnected)
    cf.connection_failed.add_callback(callback_connection_failed)
    cf.connection_lost.add_callback(callback_connection_lost)
    cf.console.receivedChar.add_callback(callback_console_incoming)
    

    # with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    with SyncCrazyflie(uri, cf=cf) as scf:
        print("Running\n")
        print("scf:")
        print(scf)
        sfloaty = scf
        connected = True

        # # Position estimate
        # cf.log.add_config(lg_stab)
        # lg_stab.data_received_cb.add_callback(lg_stab_cb)
        # lg_stab.start()

        # # Velocity estimate
        # scf.cf.log.add_config(lg_stab_vel)
        # lg_stab_vel.data_received_cb.add_callback(lg_stab_vel_cb)
        # lg_stab_vel.start()

        # # Orientation estimate
        # scf.cf.log.add_config(lg_stab_rot)
        # lg_stab_rot.data_received_cb.add_callback(lg_stab_rot_cb)
        # lg_stab_rot.start()
        
        # Angular rates estimate
        scf.cf.log.add_config(lg_stab_rot_rate)
        lg_stab_rot_rate.data_received_cb.add_callback(lg_stab_rot_rate_cb)
        lg_stab_rot_rate.start()

        # scf.cf.log.add_config(lg_gyro)
        # lg_gyro.data_received_cb.add_callback(lg_gyro_cb)
        # lg_gyro.start()

        # Limited servo commands
        scf.cf.log.add_config(lg_motors_command)
        lg_motors_command.data_received_cb.add_callback(lg_motors_command_cb)
        lg_motors_command.start()
        
        # Servo commands before limiting
        scf.cf.log.add_config(lg_control)
        lg_control.data_received_cb.add_callback(lg_control_cb)
        lg_control.start()



        # scf.cf.log.add_config(lg_stab_uncertainty)
        # lg_stab_uncertainty.data_received_cb.add_callback(lg_stab_uncertainty_cb)
        # lg_stab_uncertainty.start()

        # scf.cf.log.add_config(lg_gyro_filtered)
        # lg_gyro_filtered.data_received_cb.add_callback(lg_gyro_filtered_cb)
        # lg_gyro_filtered.start()
        
        # scf.cf.log.add_config(lg_stab_orientation)
        # lg_stab_orientation.data_received_cb.add_callback(lg_stab_orientation_cb)
        # lg_stab_orientation.start()
        
        # scf.cf.log.add_config(lg_stab_flaps)
        # lg_stab_flaps.data_received_cb.add_callback(lg_stab_flaps_cb)
        # lg_stab_flaps.start()

        # # ---------------------------------
        # # For testing the error integration
        # scf.cf.log.add_config(lg_int_pos_err)
        # lg_int_pos_err.data_received_cb.add_callback(lg_int_pos_err_cb)
        # lg_int_pos_err.start()
        
        

        rospy.Timer(rospy.Duration(1.0/sending_position_freq), send_position_to_floaty)

        # rospy.Timer(rospy.Duration(1.0/publish_data_freq), publish_data)
        rospy.Timer(rospy.Duration(1.0/publish_data_freq), publish_flexible_data)

        if dump_tasks_freq>0:
            rospy.Timer(rospy.Duration(1.0/dump_tasks_freq), dump_tasks_info)

        if activate_goal_targeting:
            rospy.Timer(rospy.Duration(1.0/send_yaw_freq), send_yaw)


        # Test the flap estimation function
        # rospy.Timer(rospy.Duration(1.0/20.0), flap_est_test_function)
        
        # rospy.Timer(rospy.Duration(1.0/20), log_data)
        # rospy.Timer(rospy.Duration(1.0/save_est_log_freq), save_est_log)


        # ani = animation.FuncAnimation(fig, animate, interval=1000)
        # ani2 = animation.FuncAnimation(fig2, animate2, interval=1000)

        rospy.spin()
