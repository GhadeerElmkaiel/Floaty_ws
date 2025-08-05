import numpy as np
import rospy
from floaty_msgs.srv import string_srv
from floaty_msgs.srv import speed_update_srv, record_data_srv, record_data_srvRequest
from floaty_msgs.srv import maestro_srv, maestro_srvRequest
from floaty_msgs.srv import iter_update_srv, motor_speed_change_srv
from netft_rdt_driver.srv import String_cmd, String_cmdRequest
import os
import json

pth_to_file = __file__
folders_to_pkg = pth_to_file.split("/")
pth_to_pkg = '/'.join(folders_to_pkg[:-2])
pth_to_sounds = '/'.join([pth_to_pkg, 'sounds', ''])
general_pth_to_data = ""
# pth_to_recorded_data = '/'.join([pth_to_pkg, 'data', ''])


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

def save_experiment_json(pth_to_json):
    json_dict = {}
    json_dict["max_number_of_iterations"] = rospy.get_param("/iterative_learning/max_number_of_iterations")
    json_dict["update_rate_decay"] = rospy.get_param("/iterative_learning/update_rate_decay")
    
    json_dict["gp_mesh_radius"] = rospy.get_param("/gp_process/gp_mesh_radius")
    json_dict["gp_mesh_gap"] = rospy.get_param("/gp_process/gp_mesh_gap")
    json_dict["motors_speed_update_rate"] = rospy.get_param("/gp_process/motors_speed_update_rate")
    json_dict["gp_uniform_value"] = rospy.get_param("/gp_process/gp_uniform_value")
    json_dict["motors_gaussian_sigma"] = rospy.get_param("/gp_process/motors_gaussian_sigma")
    json_dict["motors_exp_multiplier"] = rospy.get_param("/gp_process/motors_exp_multiplier")
    json_dict["max_motor_speed"] = rospy.get_param("/gp_process/max_motor_speed")
    json_dict["error_function"] = rospy.get_param("/gp_process/error_function")
    json_dict["kernel_l"] = rospy.get_param("/gp_process/kernel_l")
    json_dict["save_data_to_json"] = rospy.get_param("/gp_process/save_data_to_json")
    json_dict["data_to_use"] = rospy.get_param("/gp_process/data_to_use")
    json_dict["num_of_data_to_use"] = rospy.get_param("/gp_process/num_of_data_to_use")

    with open(pth_to_json, 'w') as outfile:
        json.dump(json_dict, outfile)


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
        
        # limit = 60
        limit = rospy.get_param("/gp_process/max_motor_speed")
        for i in range(len(result.updated_speeds)):
            if result.updated_speeds[i]>limit:
                speeds.append(limit)
            else:
                speeds.append(result.updated_speeds[i])

        return speeds
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def remove_bias():
    print("Removing sensor's bias")
    remove_bias_srv_name = "/ft_sensor/bias_cmd"
    rospy.wait_for_service(remove_bias_srv_name)
    try:
        remove_bias_srv = rospy.ServiceProxy(remove_bias_srv_name, String_cmd)
        
        request = String_cmdRequest()
        request.cmd = 'bias'

        result = remove_bias_srv(request)
        return True
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
    update_speeds_srv_name = "/maestro_control/ramp_motors_service"
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
    global general_pth_to_data

    try:

        # --------------------------------------------------------
        # This code is to create folders for each experiment
        # delay to get ready
        exp_num_file = rospy.get_param("/iterative_learning/exp_num_file")

        # Read the integer exp_num from the file
        with open(os.path.join(general_pth_to_data, exp_num_file), "r") as file:
            exp_num = int(file.readline().strip())

        # Increment exp_num by 1
        future_exp_num = exp_num + 1
        error_function = rospy.get_param("/gp_process/error_function")
        folder_name = "{:02d}".format(exp_num)+"-"+error_function
        
        # Replace the original exp_num with the updated value in the file
        with open(os.path.join(general_pth_to_data, exp_num_file), "w") as file:
            file.write(str(future_exp_num))

        current_exp_pth = os.path.join(general_pth_to_data, folder_name)
        os.makedirs(current_exp_pth, exist_ok=False)
        save_experiment_json(os.path.join(current_exp_pth, "experiment_parameters.json"))

        pth_to_json_file = os.path.join(current_exp_pth, "json/")
        os.makedirs(pth_to_json_file, exist_ok=False)
        # --------------------------------------------------------


        rospy.sleep(1)
        remove_bias()
        rospy.sleep(6)

        number_of_motors = rospy.get_param("/maestro_control/number_of_motors")
        alpha = rospy.get_param("/iterative_learning/update_rate_decay")
        max_iter_num = rospy.get_param("/iterative_learning/max_number_of_iterations")
        original_update_rate = rospy.get_param("/gp_process/motors_speed_update_rate")
        # pth_to_data = rospy.get_param("/data_recorder/data_file_path")
        # rospy.set_param("/data_recorder/data_file_path", current_exp_pth)

        old_speeds = req.speeds
        num_of_points = req.num_of_points
        speeds_history = [old_speeds]
        # Setting the initial motors' speeds
        print("Initializing the motors' speeds")
        update_speeds(old_speeds)
        rospy.sleep(2)
        
        for i in range(max_iter_num):
            # Start Swepping process
            start_swepping_audio()
            file_name = "iterative_learning_data_num_"+str(i)+".csv"
            # if os.path.exists(file_name):
            #     os.remove(file_name)

            pth_to_data_file = os.path.join(current_exp_pth, file_name)
            start_collecting_data(pth_to_data_file, num_of_points)
            swepping_done_audio()
            json_file_name = "iter_"+str(i)+".json"
            pth_to_json = os.path.join(pth_to_json_file, json_file_name)
            rospy.set_param("/gp_process/pth_to_json", pth_to_json)

            # Calculate GP and new speeds
            new_speeds = start_gp_calculation(pth_to_data_file, old_speeds)

            # Linear decay
            new_update_rate = original_update_rate/(1+alpha*i)

            if new_update_rate<0.05:
                new_update_rate = 0.05
            rospy.set_param("/gp_process/motors_speed_update_rate", new_update_rate)

            # Update the speeds
            update_speeds_audio()
            update_speeds(new_speeds)
            old_speeds = new_speeds
            speeds_history.append(old_speeds)
            rospy.sleep(7)

        start_swepping_audio()
        file_name = "iterative_learning_data_num_"+str(max_iter_num)+".csv"
        pth_to_data_file = os.path.join(current_exp_pth, file_name)
        start_collecting_data(pth_to_data_file, num_of_points)
        swepping_done_audio()
        # stop_speeds = [0, 0, 0, 0, 0, 0]
        stop_speeds = [0 for _ in range(number_of_motors)]
        update_speeds(stop_speeds)

        # Calculate the result for the last iteration
        json_file_name = "iter_"+str(max_iter_num)+".json"
        pth_to_json = os.path.join(pth_to_json_file, json_file_name)
        rospy.set_param("/gp_process/pth_to_json", pth_to_json)
        # Calculate GP and new speeds
        new_speeds = start_gp_calculation(pth_to_data_file, old_speeds)
        
        rospy.set_param("/gp_process/motors_speed_update_rate", original_update_rate)
        speeds_history_np = np.array(speeds_history)
        speeds_history_file = os.path.join(current_exp_pth, "speeds_history.csv")
        np.savetxt(speeds_history_file, speeds_history_np, delimiter=',', fmt='%.2f')

        print("speeds history")
        for i in range(max_iter_num):
            print("iteration {}".format(i))
            print(speeds_history[i])

        return 1

    
    # in case of an error
    except:
        rospy.ERROR("Something wrong happend in the iterative learnign node")
        # stop_speeds = [0, 0, 0, 0, 0, 0]
        stop_speeds = [0 for _ in range(number_of_motors)]
        update_speeds(stop_speeds)
        return 0
    


def change_one_motor_speed_callback(req):
    global general_pth_to_data

    try:

        # --------------------------------------------------------
        # This code is to create folders for each experiment
        # delay to get ready
        exp_num_file = rospy.get_param("/iterative_learning/exp_num_file")

        # Read the integer exp_num from the file
        with open(os.path.join(general_pth_to_data, exp_num_file), "r") as file:
            exp_num = int(file.readline().strip())

        # Increment exp_num by 1
        future_exp_num = exp_num + 1
        error_function = rospy.get_param("/gp_process/error_function")
        folder_name = "{:02d}".format(exp_num)+"-"+error_function
        
        # Replace the original exp_num with the updated value in the file
        with open(os.path.join(general_pth_to_data, exp_num_file), "w") as file:
            file.write(str(future_exp_num))

        current_exp_pth = os.path.join(general_pth_to_data, folder_name)
        os.makedirs(current_exp_pth, exist_ok=False)
        save_experiment_json(os.path.join(current_exp_pth, "experiment_parameters.json"))

        pth_to_json_file = os.path.join(current_exp_pth, "json/")
        os.makedirs(pth_to_json_file, exist_ok=False)
        # --------------------------------------------------------


        rospy.sleep(1)
        remove_bias()
        rospy.sleep(6)

        number_of_motors = rospy.get_param("/maestro_control/number_of_motors")
        alpha = rospy.get_param("/iterative_learning/update_rate_decay")
        max_iter_num = rospy.get_param("/iterative_learning/max_number_of_iterations")
        original_update_rate = rospy.get_param("/gp_process/motors_speed_update_rate")
        # pth_to_data = rospy.get_param("/data_recorder/data_file_path")
        # rospy.set_param("/data_recorder/data_file_path", current_exp_pth)

        old_speeds = req.speeds
        num_of_points = req.num_of_points
        motor_id = req.motor_id
        speed_step_change = req.speed_step_change

        speeds_history = [old_speeds]
        # Setting the initial motors' speeds
        print("Initializing the motors' speeds")
        update_speeds(old_speeds)
        rospy.sleep(2)
        
        for i in range(max_iter_num):
            # Start Swepping process
            start_swepping_audio()
            file_name = "iterative_learning_data_num_"+str(i)+".csv"
            # if os.path.exists(file_name):
            #     os.remove(file_name)

            pth_to_data_file = os.path.join(current_exp_pth, file_name)
            start_collecting_data(pth_to_data_file, num_of_points)
            swepping_done_audio()
            json_file_name = "iter_"+str(i)+".json"
            pth_to_json = os.path.join(pth_to_json_file, json_file_name)
            rospy.set_param("/gp_process/pth_to_json", pth_to_json)

            new_speeds = [x for x in old_speeds]

            new_speeds[motor_id] = new_speeds[motor_id] + speed_step_change

            # Update the speeds
            update_speeds_audio()
            update_speeds(new_speeds)
            old_speeds = new_speeds
            speeds_history.append(old_speeds)
            rospy.sleep(7)

        start_swepping_audio()
        file_name = "iterative_learning_data_num_"+str(max_iter_num)+".csv"
        pth_to_data_file = os.path.join(current_exp_pth, file_name)
        start_collecting_data(pth_to_data_file, num_of_points)
        swepping_done_audio()
        # stop_speeds = [0, 0, 0, 0, 0, 0]
        stop_speeds = [0 for _ in range(number_of_motors)]
        update_speeds(stop_speeds)

        # Calculate the result for the last iteration
        json_file_name = "iter_"+str(max_iter_num)+".json"
        pth_to_json = os.path.join(pth_to_json_file, json_file_name)
        rospy.set_param("/gp_process/pth_to_json", pth_to_json)
        # Calculate GP and new speeds
        new_speeds = start_gp_calculation(pth_to_data_file, old_speeds)
        
        rospy.set_param("/gp_process/motors_speed_update_rate", original_update_rate)
        speeds_history_np = np.array(speeds_history)
        speeds_history_file = os.path.join(current_exp_pth, "speeds_history.csv")
        np.savetxt(speeds_history_file, speeds_history_np, delimiter=',', fmt='%.2f')

        print("speeds history")
        for i in range(max_iter_num):
            print("iteration {}".format(i))
            print(speeds_history[i])

        return 1

    
    # in case of an error
    except:
        rospy.ERROR("Something wrong happend in the iterative learnign node")
        # stop_speeds = [0, 0, 0, 0, 0, 0]
        stop_speeds = [0 for _ in range(number_of_motors)]
        update_speeds(stop_speeds)
        return 0




def iterative_update_service():
    global general_pth_to_data
    rospy.init_node("iterative_speed_update_node")
    srvice = rospy.Service('iterative_speed_update_srv', iter_update_srv, iterative_speed_update_callback)
    srvice = rospy.Service('motor_speed_change_srv', motor_speed_change_srv, change_one_motor_speed_callback)
    
    print("Iterative Motors Speed Update Service Ready!")
    general_pth_to_data = rospy.get_param("/iterative_learning/pth_to_data")
    rospy.spin()


if __name__ == '__main__':
    try:
        iterative_update_service()
    except rospy.ROSInterruptException:
        pass