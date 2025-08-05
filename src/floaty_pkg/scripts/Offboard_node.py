import rospy
from geometry_msgs.msg import PoseStamped
from floaty_msgs.srv import send_position_srv #, send_position_srvResponse

send_position_srv_name="send_position"

def got_pose_info_callback(data):

    send_position = rospy.ServiceProxy(send_position_srv_name, send_position_srv)
    rospy.loginfo("Offboard node is sending a position")
    # res : send_position_srvResponse = send_position(data.pose.position, data.pose.orientation)
    res = send_position(x= data.pose.position.x, y= data.pose.position.y, z= data.pose.position.z, qw= data.pose.orientation.w, qx= data.pose.orientation.x, qy= data.pose.orientation.y, qz= data.pose.orientation.z)
    return


if __name__ == '__main__':
    # global send_position_srv_name
    optitrack_topic = rospy.get_param('/NatNet/data_topic', "/Optitrack/Floaty")
    rospy.init_node("Offboard_node")
    rospy.loginfo("Offboard node started")
    # try:
    #     send_position_srv_name = "crazyradio/" + rospy.get_param("crazyradio/send_position_srv_name", "send_position")
    #     rospy.wait_for_service(send_position_srv_name)
        
    #     rospy.Subscriber(optitrack_topic, PoseStamped, got_pose_info_callback)
    #     rospy.loginfo("Offboard node initialized correctly")
    # except rospy.ROSInterruptException:
    #     rospy.logerr("Error in offboard code")

    send_position_srv_name = "/crazyradio/" + rospy.get_param("crazyradio/send_position_srv_name", "send_position")
    rospy.wait_for_service(send_position_srv_name)
    
    rospy.Subscriber(optitrack_topic, PoseStamped, got_pose_info_callback)
    rospy.loginfo("Offboard node initialized correctly")

    rospy.spin()

