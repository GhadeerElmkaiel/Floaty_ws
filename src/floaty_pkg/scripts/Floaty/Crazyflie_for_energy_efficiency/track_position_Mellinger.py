from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
import cflib.crtp
import time
from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

class PositionTracker:
    def __init__(self):
        # Target: [x, y, z, yaw_deg]
        self.GOAL = np.array([0.0, 0.0, 0.5, 0.0])  # x, y, z, yaw(degrees)
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.current_yaw = 0.0  # degrees
        
        # ROS Setup
        rospy.init_node("crazyflie_tracker")
        cflib.crtp.init_drivers()
        self.pos_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, self.pose_callback)
        
        # Crazyflie Connection
        self.cf = Crazyflie(rw_cache='./cache')
        self.scf = SyncCrazyflie(uri, cf=self.cf)

        # Mellinger controller parameters (tune as needed)
        self.mellinger_params = {
            'ctrlMel.kp_xy': 6.0,    # XY position P gain
            'ctrlMel.kd_xy': 3.0,    # XY velocity D gain
            'ctrlMel.kp_z': 10.0,    # Z position P gain
            'ctrlMel.kd_z': 5.0,     # Z velocity D gain
            'ctrlMel.ki_m_z': 2000,  # Mass-integral gain
            'stabilizer.controller': '2'  # Mellinger controller
        }
        
    def pose_callback(self, msg):
        # Extract position
        self.current_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # Extract yaw from quaternion
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        self.current_yaw = np.degrees(R.from_quat(q).as_euler('zyx')[0])

    def run(self):
        with self.scf as scf:
            cf = scf.cf
            
            # Set Mellinger controller parameters
            for param, value in self.mellinger_params.items():
                cf.param.set_value(param, str(value))
            
            # Enable high-level commander
            cf.param.set_value('commander.enHighLevel', '1')
            
            # Send initial position estimate
            for _ in range(20):
                cf.extpos.send_extpose(
                    self.current_pos[0],
                    self.current_pos[1],
                    self.current_pos[2],
                    0, 0, 0, 1  # Assuming no rotation (qw=1)
                )
                time.sleep(0.1)

            # Takeoff sequence
            cf.high_level_commander.takeoff(0.5, 2.0)
            time.sleep(3)

            # Position tracking loop
            last_time = time.time()
            exp_end_time = last_time + 5
            # while not rospy.is_shutdown():
            while last_time<exp_end_time:
                # Update external position estimate
                cf.extpos.send_extpose(
                    self.current_pos[0],
                    self.current_pos[1],
                    self.current_pos[2],
                    0, 0, 0, 1  # Replace with actual orientation if needed
                )

                # Send position setpoint to controller
                cf.commander.send_position_setpoint(
                    self.GOAL[0],
                    self.GOAL[1],
                    self.GOAL[2],
                    self.GOAL[3]
                )

                time.sleep(0.01)  # ~100 Hz
                last_time = time.time()

            # Landing sequence
            cf.high_level_commander.land(0.0, 2.0)
            time.sleep(2)
            cf.commander.send_stop_setpoint()

if __name__ == '__main__':
    tracker = PositionTracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        pass