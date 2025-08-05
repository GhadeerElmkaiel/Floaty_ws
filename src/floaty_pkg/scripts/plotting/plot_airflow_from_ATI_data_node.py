import rospy
import os
from floaty_msgs.srv import create_gp_plots_srv, create_gp_plots_srvResponse


def create_gp_plots_callback(req):
    path_to_exp_folder = req.pth_to_data
    folder_path = path_to_exp_folder+"/scatter_plots/"
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

    service_name = "/gp_process/create_gp_plots_from_data"

    files_list = []
    for file_name in os.listdir(path_to_exp_folder):
        if "csv" in file_name:
            file_path = os.path.join(path_to_exp_folder, file_name)
            if os.path.isfile(file_path):
                files_list.append(file_name)
            
            gp_srv = rospy.ServiceProxy(service_name, create_gp_plots_srv)
            result = gp_srv(file_path)

    return create_gp_plots_srvResponse()

if __name__ == '__main__':
    # global sfloaty
    # Initialize the low-level drivers
    try:
        rospy.init_node("plot_airflow_from_ATI_data_node")
        rospy.wait_for_service("/gp_process/create_gp_plots_from_data")
        plot_gp_data_service = rospy.Service('plot_gp_data_service', create_gp_plots_srv, create_gp_plots_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("Error in plot_gp_data code")