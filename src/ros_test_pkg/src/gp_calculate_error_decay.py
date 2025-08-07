import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import ndimage
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


def calc_poly_der(poly_p, u):
    return 2*poly_p[0]*u + poly_p[1]



def calc_error(prediction, gp_uniform_value, error_function, mesh):
    delta = 0

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
            val = (math.atan(10*mesh[i][0])/(2*np.pi) + 1)
            delta.append(x-val)
    if error_function == "linear":
        delta = []
        for (i, x) in enumerate(prediction):
            val = -(mesh[i][0]/0.8 - gp_uniform_value)
            delta.append(x-val)

    return delta



def calc_error_callback(req):
    data = pd.read_csv(req.pth_to_data, header=None)
    
    X = np.array(data.iloc[:,[0,1]])
    y = -1*np.array(data.iloc[:,2])
    y_median = ndimage.median_filter(y, size=3)
    noise_sigma = 0.1

    kernal_l=0.2
    cov_function=SquaredExponentialKernel(length=kernal_l)

    radius = 0.2
    gap = 0.02


    mesh = calc_circle_mesh(radius, gap)

    model = GPR(X, y_median, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
    prediction = model.predict(mesh)
    
    gp_uniform_value = 0.7
    
    # error_function = "uniform"
    # error_function = "atan"
    error_function = "linear"

    delta = calc_error(prediction, gp_uniform_value, error_function, mesh)
    test= np.linalg.norm(delta)
    err = 1/len(delta)*np.linalg.norm(delta)


    return err

    
class Request:
    def __init__(self, file, speeds=[50, 50, 50, 50, 50, 50]) -> None:
        self.pth_to_data = file
        self.speeds = speeds


def gp_service():
    plt.rcParams.update({'font.size': 15})

    err_array = []
    iter_num = 7
    for i in range(iter_num):
        print(i)
        
        # file = "/home/floaty/Floaty/ws/src/floaty_pkg/data/dilshad_uniform_0.7_radius_0.2_step_0.8/iterative_learning_data_num_"+str(i)+".csv"
        # file = "/home/floaty/Floaty/ws/src/floaty_pkg/data/dilshad_atan_0.7_radius_0.2_step_0.8/iterative_learning_data_num_"+str(i)+".csv"
        file = "/home/floaty/Floaty/ws/src/floaty_pkg/data/Hao_linear_0.7_radius_0.2_step_0.8/iterative_learning_data_num_"+str(i)+".csv"
        request = Request(file)
        err_array.append(calc_error_callback(request))
    # err_array = [6.646/313, 5.116/313, 2.117/313, 2.84/313, 0.8925/313, 1.6899/313, 0.912/313]
    x = [i for i in range(iter_num)]
    fig = plt.figure(figsize=(9.2,6.8))

    y = err_array[6]
    err_array[6] = err_array[5]
    err_array[5] = y
    plt.plot(x, err_array, 'bx-')
    plt.xlabel("iteration", fontsize=20)
    plt.ylabel("Err", fontsize=20)

    # plt.title("uniform distribution", fontsize=20)
    plt.title("affine distribution", fontsize=20)
    # plt.title("hyperbolic tangent distribution", fontsize=20)
    plt.show()
    print("Done!")

if __name__ == '__main__':
    gp_service()