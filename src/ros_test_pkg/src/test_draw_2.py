import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import ndimage
from floaty_msgs.srv import speed_update_srv, speed_update_srvResponse
import math


mesh = []

def exp_function(center, point, sigma):
    return math.exp(-((point[0]-center[0])**2 + (point[1]-center[1])**2)/(2*sigma**2))

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


def calc_motor_gaussian(motor_pos, thrust, points, motors_gaussian_sigma):
    res = []
    for point in points:
        res.append(float(thrust*np.exp(-(np.linalg.norm(point-motor_pos)**2)/(2*motors_gaussian_sigma**2))))
    return np.array([res])


# def calc_poly(poly_p, u):
#     return poly_p[0]*u**2 + poly_p[1]*u+poly_p[0]


def update_speed_callback(pth_to_data):
    global mesh

    print("reading data from %s" %pth_to_data)
    original_data = pd.read_csv(pth_to_data)
    # data = original_data.sample(n=200)
    
    X_orig = np.array(original_data.iloc[:,[0,1]])
    y_orig = -1*np.array(original_data.iloc[:,5])
    y_median_orig = ndimage.median_filter(y_orig, size=3)

    subset_indices = np.random.choice(X_orig.shape[0], size=200, replace=False)

    X = X_orig[subset_indices,:]
    y_median = y_median_orig[subset_indices]
    noise_sigma = 0.1


    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(111, projection='3d')  
    ax.scatter(X[:,0], X[:,1], y_median)

    kernal_l=0.2
    cov_function=SquaredExponentialKernel(length=kernal_l)

    model = GPR(X, y_median, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    prediction = model.predict(mesh)
    ax.scatter(mesh[:,0], mesh[:,1], prediction)
    ax.scatter(X_orig[:,0], X_orig[:,1], y_median_orig)


    # mesh2 = calc_circle_mesh(0.5, 0.05, [-0.47, -0.27])    # motor 3
    # pth = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/airflow_shape_motor_3.csv"
    # data2 = pd.read_csv(pth)
    
    # X = np.array(data2.iloc[:,[0,1]])
    # y = -1*np.array(data2.iloc[:,5])
    # y_median = ndimage.median_filter(y, size=3)
    # noise_sigma = 0.1

    # ax.scatter(X[:,0], X[:,1], y_median)
    # model = GPR(X, y_median, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    # prediction = model.predict(mesh2)

    # ax.scatter(mesh2[:,0], mesh2[:,1], prediction)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("airflow speed")
    plt.show()

    return 0
    # # Change the ordering to fit Windy's motor ordering
    # motors_oredering = [0, 3, 1, 4, 2, 5]
    # ordered_new_speeds = [new_speeds[i] for i in motors_oredering]
    # return speed_update_srvResponse(ordered_new_speeds)

    
# class Request:
#     def __init__(self, file) -> None:
#         self.data = file


def gp_service():
    global mesh

    radius = 0.5
    gap = 0.05

    # print("Radius:%f",radius)
    # radius = 0.3
    # gap = 0.02

    mesh = calc_circle_mesh(radius, gap)     # center
    # mesh = calc_circle_mesh(radius, gap, [-0.48, 0.26])     # motor 5
    # mesh = calc_circle_mesh(radius, gap, [-0.47, -0.27])    # motor 3

    pth = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/iterative_learning_data_num_0.csv"
    # pth = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/airflow_shape_motor_5.csv"
    # pth = "/home/gelmkaiel/Floaty/ws/src/floaty_pkg/data/Windys_motors_fit/airflow_shape_motor_3.csv"
    update_speed_callback(pth)




if __name__ == '__main__':
    gp_service()
