import numpy as np
import rospy
from floaty_msgs.srv import string_srv
from floaty_msgs.srv import speed_update_srv, record_data_srv, record_data_srvRequest
from floaty_msgs.srv import maestro_srv, maestro_srvRequest
from floaty_msgs.srv import iter_update_srv
import os

pth_to_file = __file__
folders_to_pkg = pth_to_file.split("/")
pth_to_pkg = '/'.join(folders_to_pkg[:-2])
pth_to_sounds = '/'.join([pth_to_pkg, 'sounds', ''])
pth_to_recorded_data = '/'.join([pth_to_pkg, 'data', ''])


def play_sound(file_pth):
    rospy.wait_for_service('/play_audio_srv')
    try:
        play_audio = rospy.ServiceProxy('/play_audio_srv', string_srv)
        resp1 = play_audio(file_pth)
        return resp1.ok
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


# playing sounds
def swepping_done_audio():
    play_sound(pth_to_sounds+"swepping_done.mp3")


def update_speeds_audio():
    play_sound(pth_to_sounds+"updating_motors_speeds.mp3")


def start_swepping_audio():
    play_sound(pth_to_sounds+"start_swepping_process.mp3")


def start_gp_calculation(file_name, speeds):
    gp_srv_name = "/gp_process/calculate_gp_from_data"
    rospy.wait_for_service(gp_srv_name)
    try:
        gp_srv = rospy.ServiceProxy(gp_srv_name, speed_update_srv)
        result = gp_srv(file_name, speeds)
        speeds = []

        limit = 50
        for i in range(len(result.updated_speeds)):
            if result.updated_speeds[i]>limit:
                speeds.append(limit)
            else:
                speeds.append(result.updated_speeds[i])

        return speeds
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def start_collecting_data(file_name, num_of_points):
    collect_data_srv_name = "/data_recorder/collect_data_service"
    rospy.wait_for_service(collect_data_srv_name)
    try:
        collect_data_srv = rospy.ServiceProxy(collect_data_srv_name, record_data_srv)
        
        request = record_data_srvRequest()
        request.data_to_record = num_of_points
        request.file_name = file_name

        result = collect_data_srv(request)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def update_speeds(new_speeds_percent):
    update_speeds_srv_name = "/maestro_control/move_motors_service"
    rospy.wait_for_service(update_speeds_srv_name)
    try:
        update_speeds_srv = rospy.ServiceProxy(update_speeds_srv_name, maestro_srv)
        
        # TODO
        # I need to change the speeds to use percent 0-100 instead of the range 4000-8000
        new_speeds = [int(4000 + 40*new_speed_percent) for new_speed_percent  in new_speeds_percent]
        request = maestro_srvRequest()
        request.speeds = new_speeds
        print(new_speeds)
        result = update_speeds_srv(request)
        return True
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def iterative_speed_update_callback(req):
    
    # delay to get ready
    rospy.sleep(7)

    ns = rospy.get_namespace()
    alpha = rospy.get_param(ns+"/update_rate_decay")
    max_iter_num = rospy.get_param(ns+"/max_number_of_iterations")
    original_update_rate = rospy.get_param("/gp_process/motors_speed_update_rate")

    old_speeds = req.speeds
    num_of_points = req.num_of_points
    for i in range(max_iter_num):
        # Start Swepping process
        start_swepping_audio()
        file_name = pth_to_recorded_data+"iterative_learning_data_num_"+str(i)+".csv"

        if os.path.exists(file_name):
            os.remove(file_name)

        start_collecting_data(file_name, num_of_points)
        swepping_done_audio()

        # Calculate GP and new speeds
        new_speeds = start_gp_calculation(file_name, old_speeds)


        # Linear decay
        new_update_rate = original_update_rate/(1+1.3*i)

        if new_update_rate<0.05:
            new_update_rate = 0.05
        rospy.set_param("/gp_process/motors_speed_update_rate", new_update_rate)

        # Update the speeds
        update_speeds_audio()
        update_speeds(new_speeds)
        old_speeds = new_speeds
        rospy.sleep(2)
    start_swepping_audio()
    file_name = pth_to_recorded_data+"iterative_learning_data_num_"+str(max_iter_num)+".csv"
    start_collecting_data(file_name, num_of_points)
    swepping_done_audio()
    stop_speeds = [0, 0, 0, 0, 0, 0]
    update_speeds(stop_speeds)

    return 1



def iterative_update_service():
    rospy.init_node("iterative_speed_update_node")
    srvice = rospy.Service('iterative_speed_update_srv', iter_update_srv, iterative_speed_update_callback)
    print("Iterative Motors Speed Update Service Ready!")

    rospy.spin()


if __name__ == '__main__':
    try:
        iterative_update_service()
    except rospy.ROSInterruptException:
        pass