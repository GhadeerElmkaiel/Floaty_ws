import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import ndimage
from floaty_msgs.srv import speed_update_srv, speed_update_srvResponse
import os

# Get the home directory of the current user
home_directory = os.path.expanduser('~')

# Get the username from the home directory path
username = os.path.basename(home_directory)

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


def calc_poly_der(poly_p, u):
    return 2*poly_p[0]*u + poly_p[1]



def calc_error(prediction, gp_uniform_value, error_function):
    delta = 0
    # ns = rospy.get_namespace()
    # error_function = rospy.get_param(ns+"/error_function")
    # gp_uniform_value = rospy.get_param(ns+"/gp_uniform_value")  # 2

    if error_function == "uniform":
        delta = prediction - gp_uniform_value
    if error_function == "step":
        delta = []
        for (i, x) in enumerate(prediction):
            if i < prediction.shape[0]/2:
                delta.append(x-gp_uniform_value/2)
            else:
                delta.append(x-gp_uniform_value)
    return delta


def calc_speed_update(prediction, mesh, motors_speeds, ax=None):
    # Get the namespace

    update_rate = 0.5
    gp_uniform_value = 0.7
    error_function = "uniform"

    # The error F_measured (prediction) - F_des
    delta = calc_error(prediction, gp_uniform_value, error_function)

    motors_gaussian_sigma = 0.15
 
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


    # ax.scatter(mesh[:,0], mesh[:,1], delta)


    des = calc_error(np.zeros_like(delta), -gp_uniform_value, error_function)
    ax.scatter(mesh[:,0], mesh[:,1], prediction)
    ax.scatter(mesh[:,0], mesh[:,1], des)

    # Calculate the derivative of the thrutle value with respect to first motor's speed
    y = calc_motor_gaussian(motors_positions[0], motors_thrust_der[0], mesh, motors_gaussian_sigma)
    # ax.scatter(mesh[:,0], mesh[:,1], y[0])

    for i in range(1,6):
        # Calculate the derivative of the thrutle value with respect to i_st motor's speed
        res_temp = calc_motor_gaussian(motors_positions[i], motors_thrust_der[i], mesh, motors_gaussian_sigma)
        # ax.scatter(mesh[:,0], mesh[:,1], res_temp[0])
        y=np.append(y, res_temp, axis=0)

    
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("force (N)")
    # ax.set_zlabel("force error (N)")

    import tikzplotlib
    tikzplotlib.save(f"/home/{username}/Floaty/ws/src/floaty_pkg/data/uniform_0.6_radius_0.2/uniform_4.tex")

    plt.show()

    y = np.transpose(y)
    inv = np.linalg.pinv(y)



    update_speeds = -update_rate *np.matmul(inv,delta)
    new_speeds = np.add(motors_speeds, update_speeds)

    return new_speeds.tolist()


def update_speed_callback(req):
    plt.rcParams.update({'font.size': 12})
    fig = plt.figure(figsize=(10,10))

    kernal_l=0.2
    cov_function=SquaredExponentialKernel(length=kernal_l)

    radius = 0.3
    gap = 0.02

    print("reading data from %s" %req.pth_to_data)
    data = pd.read_csv(req.pth_to_data, header=None)
    
    X = np.array(data.iloc[:,[0,1]])
    y = -1*np.array(data.iloc[:,2])
    y_median = ndimage.median_filter(y, size=3)
    noise_sigma = 0.1

    speeds_vals = np.sqrt(2*y_median/(1.225*0.04*1.17))

    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X[:,0], X[:,1], y_median)
    # ax.scatter(X[:,0], X[:,1], speeds_vals)

    # ax.set_xlabel("x [m]")
    # ax.set_ylabel("y [m]")
    # ax.set_zlabel("speed [m/s]")
    # plt.show()


    mesh = calc_circle_mesh(radius, gap)

    model = GPR(X, y_median, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    prediction = model.predict(mesh)
    new_speeds = calc_speed_update(prediction, mesh, req.speeds, ax)
    

    # ---------------------------------------------
    # new_prediction = prediction
    # for (i, x) in enumerate(prediction):
    #     new_prediction[i]=i

    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  
    # ax.scatter(X[:,0], X[:,1], new_prediction)
    # plt.show()
    # ----------------------------------------------



    return speed_update_srvResponse(new_speeds)

    
class Request:
    def __init__(self, file, speeds=[50, 50, 50, 50, 50, 50]) -> None:
        self.pth_to_data = file
        self.speeds = speeds


def gp_service():
    
    # --------------------------------------------
    mesh = calc_circle_mesh(0.5, 0.02)
    # fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  
    

    motor_hexa_radius = 0.267
    motors_positions = []

    y = calc_motor_gaussian([motor_hexa_radius, 0], 3, mesh, 0.15)
    for i in range(1,6):
        x_motor = motor_hexa_radius*np.cos(np.pi/3*i)
        y_motor = motor_hexa_radius*np.sin(np.pi/3*i)
        motors_positions.append([x_motor, y_motor])

        res_temp = calc_motor_gaussian([x_motor, y_motor], 3, mesh, 0.15)
        y=np.add(y, res_temp)


    # y = calc_motor_gaussian([0, 0], 3, mesh, 0.15)
    # fig = plt.figure(figsize=(12,12))
    # plt.scatter(mesh[:,0], mesh[:,1], s=15)
    # plt.show()
    # ax = fig.add_subplot(111, projection='3d')  

    # ax.scatter(mesh[:,0], mesh[:,1], y)

    # plt.show()

    file = f"/home/{username}/Floaty/ws/src/floaty_pkg/data/test_onno_2/iterative_learning_data_num_7.csv"
    request = Request(file)
    update_speed_callback(request)
    print("done")



if __name__ == '__main__':
    gp_service()