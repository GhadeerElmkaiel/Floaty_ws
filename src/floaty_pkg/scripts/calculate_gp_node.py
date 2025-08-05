import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# from scipy import ndimage
import rospy
from floaty_msgs.srv import speed_update_srv, speed_update_srvResponse, create_gp_plots_srv, create_gp_plots_srvResponse
import math
import json
import os

mesh = []
# mesh_uniform_data = []

class SquaredExponentialKernel:
    def __init__(self, length: float = 1):
        self.length = length

    def __call__(self, arg_1: np.array, arg_2: np.array) -> float:
        return float(np.exp(-(np.linalg.norm(arg_1-arg_2)**2)/(2*self.length**2)))


class ExponentialKernel:
    def __init__(self, length: float = 1):
        self.length = length

    def __call__(self, arg_1: np.array, arg_2: np.array) -> float:
        return float(np.exp(-(np.linalg.norm(arg_1-arg_2))/(2*self.length)))


class BoxcarKernel:
    def __init__(self, length: float = 1):
        self.length = length

    def __call__(self, arg_1: np.array, arg_2: np.array) -> float:
        if np.linalg.norm(arg_1-arg_2)<self.length:
            return 0.5
        return 0


class TricubeKernel:
    def __init__(self, length: float = 1):
        self.length = length

    def __call__(self, arg_1: np.array, arg_2: np.array) -> float:
        if np.linalg.norm(arg_1-arg_2)<self.length:
            return float((70/81)*np.power(1-np.power(np.linalg.norm(arg_1-arg_2)/self.length, 3), 3))
        return 0


def cov_matrix(x1, x2, cov_function) -> np.array:
    return np.array([[cov_function(a,b) for a in x1] for b in x2])

class GPR:
    def __init__(self,
                data_x: np.array,
                data_y: np.array,
                white_noise_sigma: float = 0,
                covariance_function=SquaredExponentialKernel()):
        self.noise = white_noise_sigma
        self.data_x = data_x
        self.data_y = data_y
        self.covariance_function = covariance_function
            
        # Store the inverse of covariance matrix of input (+ machine epsilon on diagonal) since it is needed for every prediction
        self._inverse_of_covariance_matrix_of_input = np.linalg.inv(
            cov_matrix(data_x, data_x, covariance_function) +
            (3e-7 + self.noise) * np.identity(len(self.data_x)))
        # self._inverse_of_covariance_matrix_of_input = np.linalg.inv(
        #     cov_matrix(data_x, data_x, covariance_function))

        self._memory = None

    def predict(self, at_values: np.array) -> np.array:
        # The lower left part of the matrix
        # The covariance between the train and test x values
        k_star = cov_matrix(self.data_x, at_values, self.covariance_function)
        # The lower left part of the matrix
        # The covariance between the test x values and itself
        # k_star_star = cov_matrix(at_values, at_values, self.covariance_function)

        # Mean
        mean_at_values = np.dot(k_star, 
                                np.dot(self._inverse_of_covariance_matrix_of_input,
                                        self.data_y.T))
        return mean_at_values


def calc_circle_mesh(radius, mesh_size):
    mesh = []
    eps = 1e-7
    for i in np.arange(-radius, radius+eps, mesh_size):
        # for j in np.arange(-np.sqrt(radius**2 - i**2), np.sqrt(radius**2 - i**2)+eps, mesh_size):
        for j in np.arange(-radius, radius+eps, mesh_size):
            if j+eps>=-np.sqrt(abs(radius**2 - i**2)) and j-eps <= np.sqrt(abs(radius**2 - i**2)):
                mesh.append([i,j])
    return np.array(mesh)



def get_closest_points(mesh, data_pos):
    indices = []
    for x in mesh:
        distances = np.sqrt(np.sum((data_pos - x)**2, axis=1))
        idx = np.argmin(distances)
        indices.append(idx)
    return indices


def calc_motor_gaussian(motor_pos, thrust, points, motors_gaussian_sigma):
    res = []
    for point in points:
        res.append(float(thrust*np.exp(-(np.linalg.norm(point-motor_pos)**2)/(2*motors_gaussian_sigma**2))))
    return np.array([res])


# def calc_poly(poly_p, u):
#     return poly_p[0]*u**2 + poly_p[1]*u+poly_p[0]


def calc_error(prediction, mesh):
    delta = 0
    error_function = rospy.get_param("/gp_process/error_function")
    gp_uniform_value = rospy.get_param("/gp_process/gp_uniform_value")  # 2

    delta = prediction - gp_uniform_value
    if error_function == "uniform":
        delta = prediction - gp_uniform_value
    if error_function == "step":
        delta = []
        for (i, x) in enumerate(prediction):
            if i < prediction.shape[0]/2:
                delta.append(x-gp_uniform_value/2)
            else:
                delta.append(x-gp_uniform_value)
        delta = np.array(delta)
    if error_function == "atan":
        delta = []
        for (i, x) in enumerate(prediction):
            val = math.atan(10*mesh[i][0])/(2*np.pi) + gp_uniform_value
            delta.append(x-val)
        delta = np.array(delta)
    if error_function == "linear":
        delta = []
        for (i, x) in enumerate(prediction):
            val = mesh[i][0]/0.8 + gp_uniform_value
            delta.append(x-val)
        delta = np.array(delta)
    if error_function == "Gaussian":
        delta = []
        t_poly = [0.00042106, 0.0053486, -0.03799]
        speed = 40
        thrust = t_poly[0]*speed**2 + t_poly[1]*speed + t_poly[2]
        pos = [0,0.26]
        # pos = [np.cos(np.pi/6)*0.26,np.sin(np.pi/6)*0.26]
        motors_gaussian_sigma = rospy.get_param("/gp_process/motors_gaussian_sigma") # good value is 0.15
        motors_exp_multiplier = rospy.get_param("/gp_process/motors_exp_multiplier") # A multiplier used for the exponential function
        y = calc_motor_gaussian(pos, motors_exp_multiplier*thrust, mesh, motors_gaussian_sigma)
        for (i, x) in enumerate(prediction):
            val = y[0][i]
            delta.append(x-val)
        delta = np.array(delta)

    return delta


def calc_poly_der(poly_p, u):
    return 2*poly_p[0]*u + poly_p[1]


def calc_speed_update(prediction, mesh, motors_speeds, ax=None):
    update_rate = rospy.get_param("/gp_process/motors_speed_update_rate") # 0.5
    print("Update rate is : ", update_rate)
    # gp_uniform_value = rospy.get_param(ns+"/gp_uniform_value")  # 2


    # The error F_measured (prediction) - F_des
    # delta = prediction - gp_uniform_value

    delta = calc_error(prediction, mesh)
    # total_err = np.sqrt(np.average(delta))
    total_err = np.linalg.norm(delta)

    # print("GP error is : ", np.linalg.norm(delta))
    print("GP error is : ", total_err)

    motors_gaussian_sigma = rospy.get_param("/gp_process/motors_gaussian_sigma") # good value is 0.15
    motors_exp_multiplier = rospy.get_param("/gp_process/motors_exp_multiplier") # A multiplier used for the exponential function
 
    # # Old measurements
    # t_poly = [0.0003135, 0.0177, -0.0637]
    # b_poly = [0.0003965, 0.00688, -0.042]

    # # Old measurements before central prop
    # t_poly = [0.00042106, 0.0053486, -0.03799]
    # b_poly = [0.00042106, 0.0053486, -0.03799]
    # # b_poly = [0.0004198, 0.0018, -0.0145]

    # New measurments
    t_poly = [0.000846, 0.002829, -0.019436]
    b_poly = [0.000846, 0.002829, -0.019436]
    c_poly = [0.000448, 0.003509, -0.022448]



    motor_hexa_radius = 0.2648
    # motors_angles = [-np.pi/6, np.pi/2, -(5/6)*np.pi, np.pi/6, (5/6)*np.pi, -np.pi/2]
    motors_angles = [0, (2/3)*np.pi, -(2/3)*np.pi, (1/3)*np.pi, np.pi, -(1/3)*np.pi] # New Optitrack alignemnet
    motors_positions = []
    # I use the correct angle of each motor directly instead of calculating then changing the position
    for i in range(6):
        x_motor = motor_hexa_radius*np.cos(motors_angles[i])
        y_motor = motor_hexa_radius*np.sin(motors_angles[i])
        motors_positions.append([x_motor, y_motor])

    motors_positions.append([0, 0]) # Add the motor in the center

    # motors_thrust_der = [calc_poly_der(b_poly, motors_speeds[0]),
    #                      calc_poly_der(b_poly, motors_speeds[1]),
    #                      calc_poly_der(b_poly, motors_speeds[2]),
    #                      calc_poly_der(t_poly, motors_speeds[3]),
    #                      calc_poly_der(t_poly, motors_speeds[4]),
    #                      calc_poly_der(t_poly, motors_speeds[5])]

    motors_thrust_der = [calc_poly_der(b_poly, motors_speeds[0]),
                         calc_poly_der(b_poly, motors_speeds[1]),
                         calc_poly_der(b_poly, motors_speeds[2]),
                         calc_poly_der(t_poly, motors_speeds[3]),
                         calc_poly_der(t_poly, motors_speeds[4]),
                         calc_poly_der(t_poly, motors_speeds[5]),
                         calc_poly_der(c_poly, motors_speeds[6])]


    # I use the correct angle of each motor directly instead of calculating then changing the position
    # for i in range(6):
    #     x_motor = motor_hexa_radius*np.cos(np.pi/3*i)
    #     y_motor = motor_hexa_radius*np.sin(np.pi/3*i)
    #     motors_positions.append([x_motor, y_motor])

    # # motors numbering sequance 1 4 2 5 3 6
    # motors_thrust_der = [calc_poly_der(b_poly, motors_speeds[0]), calc_poly_der(t_poly, motors_speeds[3]),
    #                      calc_poly_der(b_poly, motors_speeds[1]), calc_poly_der(t_poly, motors_speeds[4]),
    #                      calc_poly_der(b_poly, motors_speeds[2]), calc_poly_der(t_poly, motors_speeds[5])]

    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  

    # ax.scatter(mesh[:,0], mesh[:,1], delta)

    # Calculate the derivative of the thrutle value with respect to first motor's speed
    y = calc_motor_gaussian(motors_positions[0], motors_exp_multiplier*motors_thrust_der[0], mesh, motors_gaussian_sigma)
    # ax.scatter(mesh[:,0], mesh[:,1], y[0])
    # ax.scatter(mesh[:,0], mesh[:,1], prediction)

    # for i in range(1,6):
    for i in range(1,7):
        # Calculate the derivative of the thrutle value with respect to i_st motor's speed
        res_temp = calc_motor_gaussian(motors_positions[i], motors_exp_multiplier*motors_thrust_der[i], mesh, motors_gaussian_sigma)
        # ax.scatter(mesh[:,0], mesh[:,1], res_temp[0])
        y=np.append(y, res_temp, axis=0)

    
    # ax.set_xlabel("x")
    # ax.set_ylabel("y")
    # ax.set_zlabel("airflow speed")
    # plt.show()

    y = np.transpose(y)
    inv = np.linalg.pinv(y)


    max_motor_speed = rospy.get_param("/gp_process/max_motor_speed")


    update_speeds = -update_rate *np.matmul(inv,delta)
    print("calc_gp_node: Update speeds:")
    print(update_speeds)
    new_speeds = np.add(motors_speeds, update_speeds)
    for i in range(len(new_speeds)):
        if new_speeds[i]>max_motor_speed:
            new_speeds[i]=max_motor_speed
        if new_speeds[i]<0:
            new_speeds[i]=0

    save_data_to_json = rospy.get_param("/gp_process/save_data_to_json")
    if save_data_to_json:
        pth_to_json = rospy.get_param("/gp_process/pth_to_json")
        error_function = rospy.get_param("/gp_process/error_function")
        # total_err = np.sqrt(np.average(delta))
        total_err = np.linalg.norm(delta)

        json_dict = {"original_speeds":list(motors_speeds),"new_speeds": new_speeds.tolist(), "update_rate": update_rate, "error": total_err, "error_function": error_function, "error_array": delta.tolist()}
        # with open(os.path.join(pth_to_json, json_file_name), 'w') as outfile:
        with open(pth_to_json, 'w') as outfile:
            json.dump(json_dict, outfile)

    return new_speeds.tolist()


def create_plots_callback(req):
    global mesh

    print("reading data from %s" %req.pth_to_data)
    # data = pd.read_csv(req.pth_to_data, header=None)
    data = pd.read_csv(req.pth_to_data)
    path_to_plot_file = os.path.dirname(req.pth_to_data)+"/scatter_plots/"
    file_name, _ = os.path.splitext(os.path.basename(req.pth_to_data))
    path_to_plot = path_to_plot_file + file_name +".png"

    data_to_use = rospy.get_param("/gp_process/data_to_use")
    
    # Use a uniformly distributed data over the same mesh we want to evaluate at
    if data_to_use.lower() == "uniform":
        uniform_mesh_radius = rospy.get_param("/gp_process/uniform_mesh_radius")
        uniform_mesh_gap = rospy.get_param("/gp_process/uniform_mesh_gap")
        uniform_mesh = calc_circle_mesh(uniform_mesh_radius, uniform_mesh_gap)

        X_all = np.array(data.iloc[:,[0,1]])
        indices = get_closest_points(uniform_mesh, X_all)
        X = np.array(data.iloc[indices,[0,1]])
        y = -1*np.array(data.iloc[indices,5])
    # Use a random sample of the data
    elif data_to_use.lower() == "random":
        all_indices = range(data.shape[0])
        num_of_data_to_use = rospy.get_param("/gp_process/num_of_data_to_use")
        indices = np.random.choice(all_indices, size=num_of_data_to_use, replace=False)
        X = np.array(data.iloc[indices.tolist(),[0,1]])
        y = -1*np.array(data.iloc[indices.tolist(),5])
    # Use all the data
    else:
        X = np.array(data.iloc[:,[0,1]])
        y = -1*np.array(data.iloc[:,5])


    noise_sigma = 0.1


    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  
    # ax.scatter(X[:,0], X[:,1], y)
    # plt.show()
    kernel_l = rospy.get_param("/gp_process/kernel_l")  # 0.09
    kernel_function = rospy.get_param("/gp_process/kernel_function")  # Gaussian

    cov_function=SquaredExponentialKernel(length=kernel_l) # default kernel function

    if kernel_function == "Exponential":
        cov_function=ExponentialKernel(length=kernel_l)
    elif kernel_function == "Gaussian":
        cov_function=SquaredExponentialKernel(length=kernel_l)
    elif kernel_function == "Boxcar":
        cov_function=BoxcarKernel(length=kernel_l)
    elif kernel_function == "Tricube":
        cov_function=TricubeKernel(length=kernel_l)


    model = GPR(X, y, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    prediction = model.predict(mesh)

    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')  
    # ax.scatter(X[:,0], X[:,1], y_median)
    

    ax.scatter(mesh[:,0], mesh[:,1], prediction)
    plt.savefig(path_to_plot)
    # ax.scatter(X_orig[:,0], X_orig[:,1], y_median_orig)

    return create_gp_plots_srvResponse()



def update_speed_callback(req):
    global mesh

    print("reading data from %s" %req.pth_to_data)
    # data = pd.read_csv(req.pth_to_data, header=None)
    data = pd.read_csv(req.pth_to_data)

    data_to_use = rospy.get_param("/gp_process/data_to_use")
    
    # Use a uniformly distributed data over the same mesh we want to evaluate at
    if data_to_use.lower() == "uniform":
        uniform_mesh_radius = rospy.get_param("/gp_process/uniform_mesh_radius")
        uniform_mesh_gap = rospy.get_param("/gp_process/uniform_mesh_gap")
        uniform_mesh = calc_circle_mesh(uniform_mesh_radius, uniform_mesh_gap)

        X_all = np.array(data.iloc[:,[0,1]])
        indices = get_closest_points(uniform_mesh, X_all)
        X = np.array(data.iloc[indices,[0,1]])
        y = -1*np.array(data.iloc[indices,5])
    # Use a random sample of the data
    elif data_to_use.lower() == "random":
        all_indices = range(data.shape[0])
        num_of_data_to_use = rospy.get_param("/gp_process/num_of_data_to_use")
        indices = np.random.choice(all_indices, size=num_of_data_to_use, replace=False)
        X = np.array(data.iloc[indices.tolist(),[0,1]])
        y = -1*np.array(data.iloc[indices.tolist(),5])
    # Use all the data
    else:
        X = np.array(data.iloc[:,[0,1]])
        y = -1*np.array(data.iloc[:,5])


    noise_sigma = 0.1


    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  
    # ax.scatter(X[:,0], X[:,1], y)
    # plt.show()
    kernel_l = rospy.get_param("/gp_process/kernel_l")  # 0.09
    kernel_function = rospy.get_param("/gp_process/kernel_function")  # Gaussian

    cov_function=SquaredExponentialKernel(length=kernel_l) # default kernel function

    if kernel_function == "Exponential":
        cov_function=ExponentialKernel(length=kernel_l)
    elif kernel_function == "Gaussian":
        cov_function=SquaredExponentialKernel(length=kernel_l)
    elif kernel_function == "Boxcar":
        cov_function=BoxcarKernel(length=kernel_l)
    elif kernel_function == "Tricube":
        cov_function=TricubeKernel(length=kernel_l)


    model = GPR(X, y, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    prediction = model.predict(mesh)
    new_speeds = calc_speed_update(prediction, mesh, req.speeds)

    # # No need to do that anymore
    # # Change the ordering to fit Windy's motor ordering
    # motors_oredering = [0, 3, 1, 4, 2, 5]
    # ordered_new_speeds = [new_speeds[i] for i in motors_oredering]
    return speed_update_srvResponse(new_speeds)

    
# class Request:
#     def __init__(self, file) -> None:
#         self.data = file


def gp_service():
    global mesh
    # global mesh_uniform_data

    rospy.init_node("calculate_gp_node")

    radius = rospy.get_param("/gp_process/gp_mesh_radius")  # 0.3
    gap = rospy.get_param("/gp_process/gp_mesh_gap")        # 0.02

    # print("Radius:%f",radius)
    # radius = 0.3
    # gap = 0.02


    data_to_use = rospy.get_param("/gp_process/data_to_use")
    
    # Use a uniformly distributed data over the same mesh we want to evaluate at
    if data_to_use.lower() == "uniform":
        uniform_mesh_radius = rospy.get_param("/gp_process/uniform_mesh_radius")
        uniform_mesh_gap = rospy.get_param("/gp_process/uniform_mesh_gap")
        mesh = calc_circle_mesh(uniform_mesh_radius, uniform_mesh_gap)
    else:
        mesh = calc_circle_mesh(radius, gap)
    # mesh_uniform_data = calc_circle_mesh(0.37, gap)

    srvice = rospy.Service('calculate_gp_from_data', speed_update_srv, update_speed_callback)
    srvice = rospy.Service('create_gp_plots_from_data', create_gp_plots_srv, create_plots_callback)
    print("Gaussian Process Service Ready!")


    rospy.spin()


if __name__ == '__main__':
    try:
        gp_service()
    except rospy.ROSInterruptException:
        pass