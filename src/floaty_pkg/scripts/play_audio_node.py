import rospy
from floaty_msgs.srv import string_srv, string_srvResponse
from playsound import playsound


def play_audio_callback(req):
    # print("Playing audio file: %s" %req.data)
    playsound(req.data)
    return string_srvResponse(1)

def audio_service():
    rospy.init_node("play_audio_node")
    srvice = rospy.Service('play_audio_srv', string_srv, play_audio_callback)
    print("Audio node ready!")
    
    rospy.spin()


if __name__ == '__main__':
    try:
        audio_service()
    except rospy.ROSInterruptException:
        pass