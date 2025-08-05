import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import ndimage
import rospy
from floaty_msgs.srv import speed_update_srv, speed_update_srvResponse
import math


class SquaredExponentialKernel:
    def __init__(self, sigma_f: float = 1, length: float = 1):
        self.sigma_f = sigma_f
        self.length = length

    def __call__(self, arg_1: np.array, arg_2: np.array) -> float:
        return float(self.sigma_f * np.exp(-(np.linalg.norm(arg_1-arg_2)**2)/(2*self.length**2)))


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

        self._memory = None

    def predict(self, at_values: np.array) -> np.array:
        # The lower left part of the matrix
        # The covariance between the train and test x values
        k_star = cov_matrix(self.data_x, at_values, self.covariance_function)
        # The lower left part of the matrix
        # The covariance between the test x values and itself
        k_star_star = cov_matrix(at_values, at_values, self.covariance_function)

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


def calc_motor_gaussian(motor_pos, thrust, points, motors_gaussian_sigma):
    res = []
    for point in points:
        res.append(float(thrust*np.exp(-(np.linalg.norm(point-motor_pos)**2)/(2*motors_gaussian_sigma**2))))
    return np.array([res])


# def calc_poly(poly_p, u):
#     return poly_p[0]*u**2 + poly_p[1]*u+poly_p[0]


def calc_error(prediction, mesh):
    delta = 0
    ns = rospy.get_namespace()
    error_function = rospy.get_param(ns+"/error_function")
    gp_uniform_value = rospy.get_param(ns+"/gp_uniform_value")  # 2

    if error_function == "uniform":
        delta = prediction - gp_uniform_value
    if error_function == "step":
        delta = []
        for (i, x) in enumerate(prediction):
            if i < prediction.shape[0]/2:
                delta.append(x-gp_uniform_value/2)
            else:
                delta.append(x-gp_uniform_value)
    if error_function == "atan":
        delta = []
        for (i, x) in enumerate(prediction):
            val = math.atan(10*mesh[i][0])/(2*np.pi) + 1
            delta.append(x-val)
    if error_function == "linear":
        delta = []
        for (i, x) in enumerate(prediction):
            val = mesh[i][0]/0.8 + gp_uniform_value
            delta.append(x-val)

    return delta


def calc_poly_der(poly_p, u):
    return 2*poly_p[0]*u + poly_p[1]


def calc_speed_update(prediction, mesh, motors_speeds, ax=None):
    # Get the namespace
    ns = rospy.get_namespace()

    update_rate = rospy.get_param(ns+"/motors_speed_update_rate") # 0.5
    print("Update rate is : %f", update_rate)
    # gp_uniform_value = rospy.get_param(ns+"/gp_uniform_value")  # 2


    # The error F_measured (prediction) - F_des
    # delta = prediction - gp_uniform_value

    delta = calc_error(prediction, mesh)
    print("GP error is %f: ", np.linalg.norm(delta))

    motors_gaussian_sigma = rospy.get_param(ns+"/motors_gaussian_sigma") # good value is 0.15
 
    t_poly = [0.0003135, 0.0177, -0.0637]
    b_poly = [0.0003965, 0.00688, -0.042]


    motor_hexa_radius = 0.2648
    motors_positions = []
    for i in range(6):
        x_motor = motor_hexa_radius*np.cos(np.pi/3*i)
        y_motor = motor_hexa_radius*np.sin(np.pi/3*i)
        motors_positions.append([x_motor, y_motor])

    # motors numbering sequance 1 4 2 5 3 6
    motors_thrust_der = [calc_poly_der(b_poly, motors_speeds[0]), calc_poly_der(t_poly, motors_speeds[3]),
                         calc_poly_der(b_poly, motors_speeds[1]), calc_poly_der(t_poly, motors_speeds[4]),
                         calc_poly_der(b_poly, motors_speeds[2]), calc_poly_der(t_poly, motors_speeds[5])]

    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  

    # ax.scatter(mesh[:,0], mesh[:,1], delta)

    # Calculate the derivative of the thrutle value with respect to first motor's speed
    y = calc_motor_gaussian(motors_positions[0], motors_thrust_der[0], mesh, motors_gaussian_sigma)
    # ax.scatter(mesh[:,0], mesh[:,1], y[0])

    for i in range(1,6):
        # Calculate the derivative of the thrutle value with respect to i_st motor's speed
        res_temp = calc_motor_gaussian(motors_positions[i], motors_thrust_der[i], mesh, motors_gaussian_sigma)
        # ax.scatter(mesh[:,0], mesh[:,1], res_temp[0])
        y=np.append(y, res_temp, axis=0)

    
    # ax.set_xlabel("x")
    # ax.set_ylabel("y")
    # ax.set_zlabel("z")
    # plt.show()

    y = np.transpose(y)
    inv = np.linalg.pinv(y)



    update_speeds = -update_rate *np.matmul(inv,delta)
    new_speeds = np.add(motors_speeds, update_speeds)

    return new_speeds.tolist()


def update_speed_callback(req):
    print("reading data from %s" %req.pth_to_data)
    data = pd.read_csv(req.pth_to_data, header=None)
    
    X = np.array(data.iloc[:,[0,1]])
    y = -1*np.array(data.iloc[:,2])
    y_median = ndimage.median_filter(y, size=3)
    noise_sigma = 0.1


    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  
    # ax.scatter(X[:,0], X[:,1], y_median)

    kernal_l=0.2
    cov_function=SquaredExponentialKernel(length=kernal_l)

    ns = rospy.get_namespace()
    radius = rospy.get_param(ns+"/gp_mesh_radius")  # 0.3
    gap = rospy.get_param(ns+"/gp_mesh_gap")        # 0.02

    # print("Radius:%f",radius)
    # radius = 0.3
    # gap = 0.02

    mesh = calc_circle_mesh(radius, gap)

    model = GPR(X, y_median, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    prediction = model.predict(mesh)
    new_speeds = calc_speed_update(prediction, mesh, req.speeds)

    # Change the ordering to fit Windy's motor ordering
    motors_oredering = [0, 3, 1, 4, 2, 5]
    ordered_new_speeds = [new_speeds[i] for i in motors_oredering]
    return speed_update_srvResponse(ordered_new_speeds)

    
# class Request:
#     def __init__(self, file) -> None:
#         self.data = file


def gp_service():
    rospy.init_node("calculate_gp_node")
    srvice = rospy.Service('calculate_gp_from_data', speed_update_srv, update_speed_callback)
    print("Gaussian Process Service Ready!")


    rospy.spin()


if __name__ == '__main__':
    try:
        gp_service()
    except rospy.ROSInterruptException:
        pass