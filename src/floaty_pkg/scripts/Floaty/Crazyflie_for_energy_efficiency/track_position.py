from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
import cflib.crtp
import time
from geometry_msgs.msg import PoseStamped
import rospy


uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

sfloaty = SyncCrazyflie(uri)

x=0
y=0
z=0
qx=0
qy=0
qz=0
qw=1
err_x=0
err_y=0
err_z=0


cf_pose_update_freq = 100
update_counter = 0
land_time = 10

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

Goal_position = Position(0, 0, 0.5)
# Goal_orientation = OrientationQuat(1, 0, 0, 0)
Goal_orientation = OrientationAngs(0, 0, 0)
Goal = Pose(Goal_position, Goal_orientation)

def update_pose_callback(req):
    global x, y, z, qw, qx, qy, qz, err_x, err_y, err_z

    qw = req.pose.orientation.w
    qx = req.pose.orientation.x
    qy = req.pose.orientation.y
    qz = req.pose.orientation.z

    x = req.pose.position.x
    y = req.pose.position.y
    z = req.pose.position.z

    err_x = req.pose.position.x - Goal.position.x
    err_y = req.pose.position.y - Goal.position.y
    err_z = req.pose.position.z - Goal.position.z


def send_position_to_floaty(event):
    global x, y, z, qw, qx, qy, qz, err_x, err_y, err_z
    global sfloaty

    cf = sfloaty.cf
    print("Sending position: ", x, ", ", y, ", ", z)
    cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
    print("Sending target: ", Goal.position.x, ", ", Goal.position.y, ", ", Goal.position.z)
    cf.commander.send_position_setpoint(Goal.position.x, Goal.position.y, Goal.position.z, 0)

def update_goal(event):
   global update_counter, cf_pose_update_freq
   global Goal

   update_counter = update_counter+1
   current_time = update_counter/cf_pose_update_freq
   if current_time> land_time:
      print("Landing")
      Goal.position.z = 0


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
    # Initialize the low-level drivers
    try:
        rospy.init_node("crazyflie_track_node")

        cflib.crtp.init_drivers()
        pos_sub = rospy.Subscriber("/Optitrack/Floaty", PoseStamped, update_pose_callback)


    except rospy.ROSInterruptException:
        rospy.logerr("Error in radio code")


    cf=Crazyflie(rw_cache='./cache')

    cf.connected.add_callback(callback_connected)
    cf.disconnected.add_callback(callback_disconnected)
    cf.connection_failed.add_callback(callback_connection_failed)
    cf.connection_lost.add_callback(callback_connection_lost)
    cf.console.receivedChar.add_callback(callback_console_incoming)


    with SyncCrazyflie(uri, cf=cf) as scf:
        print("Running\n")
        print("scf:")
        print(scf)
        sfloaty = scf
        connected = True
        print("connected")
        cf = scf.cf
        cf.param.set_value('commander.enHighLevel', '1')

        cf.param.set_value('stabilizer.controller', '1')   # PID
        cf.param.set_value('posCtlPid.zKp', '2.0')   # increase from 2
        # cf.param.set_value('posCtlPid.zKi', '1.0')   

        # cf.param.set_value('stabilizer.controller', '2')
        cf.extpos.send_extpose(0, 0, 0, 0, 0, 0, 1)
        input("Press any key when ready")
        cf.commander.send_position_setpoint(Goal.position.x, Goal.position.y, Goal.position.z, 0)

        rospy.Timer(rospy.Duration(1.0/cf_pose_update_freq), send_position_to_floaty)
        rospy.Timer(rospy.Duration(1.0/cf_pose_update_freq), update_goal)
        rospy.spin()
