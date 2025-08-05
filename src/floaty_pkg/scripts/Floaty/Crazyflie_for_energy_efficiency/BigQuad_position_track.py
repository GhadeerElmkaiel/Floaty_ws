import logging
import time
import math
import threading

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper
from cflib.crazyflie.commander import Commander

from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np
import math
from floaty_msgs.srv import empty_srv, int_srv, maestro_srv
from floaty_msgs.srv import empty_srvRequest, int_srvRequest, maestro_srvRequest

# --- External Motion Capture Data Provider ---
# You MUST integrate your MoCap system to update this object.
class ExternalMocapDataProvider:
    def __init__(self):
        self.position = [0.0, 0.0, 0.0]  # x, y, z in meters
        self.orientation_quat = [0.0, 0.0, 0.0, 1.0] # qx, qy, qz, qw
        self.lock = threading.Lock()
        self.data_available_once = False # Flag to indicate if data has ever been received
        self.pos_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, self.update_pose_from_external_source)

    def get_current_pose(self):
        """
        Returns the latest pose.
        This method will be called by the extpos sending thread.
        Ensure this method is thread-safe if updates happen concurrently.
        """
        with self.lock:
            # In a real system, you might also check a timestamp for data freshness
            if self.data_available_once:
                return list(self.position), list(self.orientation_quat)
            else:
                # print("Mocap data not yet available from provider.")
                return None, None # Indicates no data received yet or stale data

    # def pose_callback(self, msg):
    #     self.current_pos = np.array([
    #         msg.pose.position.x,
    #         msg.pose.position.y,
    #         msg.pose.position.z
    #     ])
    #     self.current_yaw = quaternion_to_yaw(
    #         msg.pose.orientation.x,
    #         msg.pose.orientation.y,
    #         msg.pose.orientation.z,
    #         msg.pose.orientation.w
    #     )

    def update_pose_from_external_source(self, msg):
        """
        !!! IMPORTANT !!!
        This method MUST be called by YOUR MoCap client/SDK
        whenever new pose data is received for the Crazyflie.
        Example:
            mocap_client_callback(mocap_data):
                # Assuming mocap_data contains x, y, z, qx, qy, qz, qw
                # Perform any necessary coordinate transformations here!
                g_mocap_data_provider.update_pose_from_external_source(
                    mocap_data.x, mocap_data.y, mocap_data.z,
                    mocap_data.qx, mocap_data.qy, mocap_data.qz, mocap_data.qw
                )
        """
        print("Getting data")
        with self.lock:
            self.position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
            ]
            self.orientation_quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
            ] # Optional, extpos primarily uses position
            if not self.data_available_once:
                print(f"First MoCap data received: Pos={self.position}")
            self.data_available_once = True
            # For debugging: print(f"ExternalMocapDataProvider Updated: Pos={self.position}")

# Global instance of the MoCap data provider. Your MoCap client will update this.
g_mocap_data_provider = ExternalMocapDataProvider()
# --- End External Motion Capture Data Provider ---


# --- Configuration ---
# URI to the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Target hover position (x, y, z in meters) and yaw (degrees)
TARGET_POSITION = [0.5, 0.5, 0.5]  # Target X, Y, Z
TARGET_YAW_DEG = 0.0                # Target Yaw

# Flight duration
FLIGHT_DURATION_S = 30 # Increased duration for testing

# MoCap update rate for sending extpos
EXTPOS_SEND_RATE_HZ = 50.0 # Send extpos at 50 Hz
SETPOS_SEND_RATE_HZ = 20.0 # Send position_setpoint at 20 Hz

# Set to True if you have a real MoCap system providing data to g_mocap_data_provider
# If False, the script will attempt to fly without external position updates
# (which will likely be unstable for precise hovering without onboard sensors like Flow deck)
MOCAP_SYSTEM_AVAILABLE = True # IMPORTANT: Set to True if your MoCap is feeding g_mocap_data_provider

# Global Crazyflie object and connection flag
g_cf = None
g_cf_is_connected = False

logging.basicConfig(level=logging.INFO)


def quaternion_to_yaw(qx, qy, qz, qw):
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')
    print("High-level commander activated")

def deactivate_high_level_commander(cf):
    if cf and g_cf_is_connected: # Check if cf is valid and connected
        try:
            cf.param.set_value('commander.enHighLevel', '0')
            print("High-level commander deactivated")
        except Exception as e:
            print(f"Error deactivating HLC: {e}")


def activate_mocap_estimator(cf):
    # Set the estimator to EKF (2) which can use external position
    cf.param.set_value('stabilizer.estimator', '2') # 2 for EKF
    print("Estimator set to EKF (for extpos)")

    # Set standard deviations for external position.
    # These values might need tuning based on your MoCap system's accuracy.
    # Lower values mean more trust in the MoCap data.
    # Note: extQuatStdDev is also available if you send quaternion data and want the EKF to use it.
    cf.param.set_value('locSrv.extPosStdDev', 0.03)  # Adjust as needed (meters)
    # cf.param.set_value('locSrv.extQuatStdDev', 0.05) # Uncomment if sending reliable orientation
    print("MoCap standard deviations configured for EKF.")

def send_extpos_thread_function(cf, mocap_data_source):
    """
    Thread function to continuously send external position data.
    """
    print("ExtPos sending thread started.")
    while g_cf_is_connected:
        if MOCAP_SYSTEM_AVAILABLE:
            pos, _ = mocap_data_source.get_current_pose() # We primarily need position for extpos
            if pos is not None:
                # print(f"Sending extpos: {pos}") # Uncomment for debugging
                try:
                    cf.extpos.send_extpos(pos[0], pos[1], pos[2])
                except Exception as e:
                    print(f"Error sending extpos: {e}")
                    break # Exit thread on error
            else:
                # This will print if MoCap data hasn't arrived yet or if get_current_pose returns None
                # print("MoCap data not available for extpos yet.")
                pass
        else:
            # This case should ideally not be reached if MOCAP_SYSTEM_AVAILABLE is false,
            # as the thread might not be started.
            pass
        time.sleep(1.0 / EXTPOS_SEND_RATE_HZ)
    print("ExtPos sending thread stopped.")


def control_loop_thread_function(cf, target_pos, target_yaw_deg):
    """
    Thread function to manage takeoff, hover at setpoint, and landing.
    """
    print("Control loop (position setpoint sending) thread started.")
    global g_cf_is_connected
    commander = cf.commander
    hlc = cf.high_level_commander

    try:
        print(f"Taking off to Z={target_pos[2]}m...")
        hlc.takeoff(target_pos[2], 2.5) # Target height (m), duration (s)
        time.sleep(3.0) # Give it time to reach height

        print(f"Moving to initial target: {target_pos} with yaw {target_yaw_deg} deg")
        hlc.go_to(target_pos[0], target_pos[1], target_pos[2], target_yaw_deg, 3.0, relative=False)
        time.sleep(3.5) # Give it time to reach the target

        print("Switching to continuous position_setpoint control for hovering.")
        start_time = time.time()
        while g_cf_is_connected and (time.time() - start_time < FLIGHT_DURATION_S):
            # print(f"Sending position_setpoint: X={target_pos[0]}, Y={target_pos[1]}, Z={target_pos[2]}, Yaw={target_yaw_deg}") # Uncomment for debugging
            try:
                commander.send_position_setpoint(target_pos[0], target_pos[1], target_pos[2], target_yaw_deg)
            except Exception as e:
                print(f"Error sending position_setpoint: {e}")
                break # Exit thread on error
            time.sleep(1.0 / SETPOS_SEND_RATE_HZ)

    except Exception as e:
        print(f"Error in control loop: {e}")
    finally:
        print("Control loop duration ended or error occurred. Initiating landing...")
        try:
            if g_cf_is_connected and hlc: # Check if hlc is still valid
                hlc.land(0.05, 3.0) # Target height (slightly above ground), duration
                time.sleep(3.5)
                hlc.stop() # Stop motors
        except Exception as e_land:
            print(f"Error during HLC landing: {e_land}")
            # Fallback: try to stop motors directly if HLC fails
            try:
                if g_cf_is_connected and commander:
                    commander.send_stop_setpoint()
                    print("Sent stop setpoint as fallback.")
            except Exception as e_stop:
                print(f"Error sending stop setpoint: {e_stop}")

        g_cf_is_connected = False # Signal other threads to stop
        print("Control loop (position setpoint sending) thread stopped.")


# --- Main Execution ---
if __name__ == '__main__':
    cflib.crtp.init_drivers()

    print("Crazyflie MoCap Hover Script")
    print("-----------------------------")
    print(f"Attempting to connect to URI: {URI}")
    print(f"Target hover position: {TARGET_POSITION}, Yaw: {TARGET_YAW_DEG} deg")
    print(f"Flight duration: {FLIGHT_DURATION_S} seconds")

    if MOCAP_SYSTEM_AVAILABLE:
        print("MOCAP_SYSTEM_AVAILABLE is True.")
        print("!!! IMPORTANT: Ensure your external MoCap client is running and")
        print("!!! calling 'g_mocap_data_provider.update_pose_from_external_source(...)'")
        print("!!! with the Crazyflie's pose data.")
    else:
        print("WARNING: MOCAP_SYSTEM_AVAILABLE is False.")
        print("The drone will attempt to fly using only onboard sensors if extpos is not sent.")
        print("Precise hovering at a fixed point will likely be unstable without MoCap.")

    extpos_thread = None
    control_thread = None

    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            g_cf = scf.cf
            g_cf_is_connected = True
            print(f"Successfully connected to {URI}")

            activate_high_level_commander(g_cf)
            if MOCAP_SYSTEM_AVAILABLE:
                activate_mocap_estimator(g_cf)
            else:
                print("Skipping MoCap estimator activation as MOCAP_SYSTEM_AVAILABLE is False.")

            # Reset the estimator - important for the EKF to initialize correctly
            g_cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            g_cf.param.set_value('kalman.resetEstimation', '0')
            print("Kalman estimator reset.")
            time.sleep(0.5) # Wait for estimator to settle

            # Start the thread that sends extpos data (if MoCap is enabled)
            if MOCAP_SYSTEM_AVAILABLE:
                extpos_thread = threading.Thread(target=send_extpos_thread_function,
                                                 args=(g_cf, g_mocap_data_provider))
                extpos_thread.daemon = True
                extpos_thread.start()
            else:
                print("Extpos thread not started as MOCAP_SYSTEM_AVAILABLE is False.")

            # Start the main control loop thread
            control_thread = threading.Thread(target=control_loop_thread_function,
                                               args=(g_cf, TARGET_POSITION, TARGET_YAW_DEG))
            control_thread.daemon = True
            control_thread.start()

            # Keep the main thread alive while the control thread is running
            control_thread.join() # Wait for the control thread to finish

            print("Main flight sequence finished in 'try' block.")

    except Exception as e:
        print(f"An error occurred in the main execution block: {e}")
        g_cf_is_connected = False # Ensure threads know to stop

    finally:
        print("Cleaning up...")
        g_cf_is_connected = False # Signal all threads to stop

        if control_thread and control_thread.is_alive():
            print("Waiting for control thread to finish...")
            control_thread.join(timeout=5.0) # Give some time for graceful shutdown
            if control_thread.is_alive():
                print("Control thread did not finish cleanly.")

        if extpos_thread and extpos_thread.is_alive():
            print("Waiting for extpos thread to finish...")
            extpos_thread.join(timeout=2.0)
            if extpos_thread.is_alive():
                print("Extpos thread did not finish cleanly.")

        if g_cf: # If connection was established
            deactivate_high_level_commander(g_cf)
            # The SyncCrazyflie context manager handles disconnection.
            print("Crazyflie should be disconnected by SyncCrazyflie context manager.")
        else:
            print("No Crazyflie connection was established or it was lost.")

        print("Script finished.")