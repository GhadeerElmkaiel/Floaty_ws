import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import ndimage
from floaty_msgs.srv import speed_update_srv, speed_update_srvResponse
import math


mesh = []

def exp_function(center, point, sigma):
    return math.exp(-((point[0]-center[0])**2 + (point[1]-center[1])**2)/(sigma**2))

def eval_exp_error(mesh, values, sigma, center, exp_vale_at_center):
    error = 0
    for i, point in enumerate(mesh):
        error += (exp_vale_at_center*exp_function(center, point, sigma) - values[i])**2
    error = math.sqrt(error/len(mesh))
    return error

class SquaredExponentialKernel:
    def __init__(self, sigma_f: float = 1, length: float = 1):
        self.sigma_f = sigma_f
        self.length = length

    def __call__(self, arg_1: np.array, arg_2: np.array) -> float:
        return float((self.sigma_f**2) * np.exp(-(np.linalg.norm(arg_1-arg_2)**2)/(2*self.length**2)))


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


def calc_circle_mesh(radius, mesh_size, shift_in_center=[0,0]):
    mesh = []
    eps = 1e-7
    shift_i = shift_in_center[0]
    shift_j = shift_in_center[1]
    for i in np.arange(-radius, radius+eps, mesh_size):
        # for j in np.arange(-np.sqrt(radius**2 - i**2), np.sqrt(radius**2 - i**2)+eps, mesh_size):
        for j in np.arange(-radius, radius+eps, mesh_size):
            if j+eps>=-np.sqrt(abs(radius**2 - i**2)) and j-eps <= np.sqrt(abs(radius**2 - i**2)):
                mesh.append([i+shift_i, j+shift_j])
    return np.array(mesh)



def update_speed_callback():

    radius = 0.25
    gap = 0.025

    kernal_l=0.2
    cov_function=SquaredExponentialKernel(length=kernal_l)

    pth = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/airflow_shape_motor_5.csv"
    
    mesh_center = [-0.48/2, 0.26/2]         # here I divide by two because the position measurments where scaled by two
    mesh = calc_circle_mesh(radius, gap, mesh_center)     # motor 5

    data = pd.read_csv(pth)
    
    X = np.array(data.iloc[:,[0,1]])/2      # here I divide by two because the position measurments where scaled by two
    y = -1*np.array(data.iloc[:,5])
    y_median = ndimage.median_filter(y, size=3)
    noise_sigma = 0.1


    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')  
    # ax.scatter(X[:,0], X[:,1], y_median)

    model = GPR(X, y_median, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    prediction = model.predict(mesh)
    ax.scatter(mesh[:,0], mesh[:,1], prediction)

    base_exp_value = 1.28206
    min_err = 1e9
    sigmas_to_test = np.linspace(0.1,2,20)
    exps_values_multiplier = np.linspace(0.8,1.8,25)
    for sigma in sigmas_to_test:
        for exp_multiplier in exps_values_multiplier:
            error = eval_exp_error(mesh, prediction, sigma, mesh_center, exp_multiplier*base_exp_value)
            if error < min_err:
                min_err = error
                best_sigma = sigma
                best_exp_multiplier = exp_multiplier

    print("best_sigma = ", best_sigma)
    print("best_exp_multiplier = ", best_exp_multiplier)

    exp_function_res = []
    for pos in mesh:
        exp_function_res.append(best_exp_multiplier*base_exp_value*exp_function(mesh_center, pos, best_sigma))
    exp_function_res = np.array(exp_function_res)
    
    ax.scatter(mesh[:,0], mesh[:,1], exp_function_res)


    # mesh2_center = [-0.47/2, -0.27/2]           # here I divide by two because the position measurments where scaled by two
    # mesh2 = calc_circle_mesh(radius, gap, mesh2_center)    # motor 3
    # pth = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/airflow_shape_motor_3.csv"
    # data2 = pd.read_csv(pth)
    
    # X = np.array(data2.iloc[:,[0,1]])/2         # here I divide by two because the position measurments where scaled by two
    # y = -1*np.array(data2.iloc[:,5])
    # y_median = ndimage.median_filter(y, size=3)
    # noise_sigma = 0.1

    # ax.scatter(X[:,0], X[:,1], y_median)
    # model = GPR(X, y_median, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    # prediction = model.predict(mesh2)

    # ax.scatter(mesh2[:,0], mesh2[:,1], prediction)

    # base_exp_value = 1.125
    # min_err = 1e9
    # # sigmas_to_test = np.linspace(0.1,2,20)
    # # exps_values_multiplier = np.linspace(0.8,1.8,25)
    # for sigma in sigmas_to_test:
    #     for exp_multiplier in exps_values_multiplier:
    #         error = eval_exp_error(mesh2, prediction, sigma, mesh2_center, exp_multiplier*base_exp_value)
    #         if error < min_err:
    #             min_err = error
    #             best_sigma = sigma
    #             best_exp_multiplier = exp_multiplier



    # print("best_sigma = ", best_sigma)
    # print("best_exp_multiplier = ", best_exp_multiplier)

    # exp_function_res = []
    # for pos in mesh2:
    #     exp_function_res.append(best_exp_multiplier*base_exp_value*exp_function(mesh2_center, pos, best_sigma))
    # exp_function_res = np.array(exp_function_res)
    
    # # ax.scatter(mesh2[:,0], mesh2[:,1], exp_function_res)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("airflow speed")
    ax.legend(["GP model", "Fitted exponential"])
    plt.show()

    return 0




if __name__ == '__main__':
    update_speed_callback()
