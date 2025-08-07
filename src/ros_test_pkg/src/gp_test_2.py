import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from itertools import product
from scipy import ndimage

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

        # Covariance.
        cov_at_values = k_star_star - \
            np.dot(k_star, np.dot(
                self._inverse_of_covariance_matrix_of_input, k_star.T))

        # Adding value larger than machine epsilon to ensure positive semi definite
        cov_at_values = cov_at_values + 3e-7 * np.ones(
            np.shape(cov_at_values)[0])

        var_at_values = np.diag(cov_at_values)

        self._memory = {
            'mean': mean_at_values,
            'covariance_matrix': cov_at_values,
            'variance': var_at_values
        }
        return mean_at_values

# -----------------------------------------------------------------
# -----------------------------------------------------------------

# train_size = 250
# range = 100
# noise_sigma = 0.02
# range_x = np.arange(-range/2, range/2, 0.25)
# noise_for_train_coff = 1

# x_values = np.random.random(size=train_size)*range-range/2
# y_values = np.arctan(x_values) + np.random.normal(0, noise_sigma, size=train_size)*noise_for_train_coff
# function_vals = np.arctan(range_x)


data = pd.read_csv("/home/floaty/Floaty/ws/src/ros_test_pkg/src/data/recordings_all_motors_6000_fixed.csv", header=None)

X = np.array(data.iloc[:,[0,1]])
y = -1*np.array(data.iloc[:,2])
y_median = ndimage.median_filter(y, size=3)
axis_margins = 0
noise_sigma = 0.1

x1 = np.linspace(X[:,0].min() - axis_margins, X[:,0].max() + axis_margins) #p
x2 = np.linspace(X[:,1].min() - axis_margins, X[:,1].max() + axis_margins) #q
x = (np.array([x1, x2])).T


x1x2 = np.array(list(product(x1, x2)))
# X0p, X1p = x1x2[:,0].reshape(x1.size, x2.size), x1x2[:,1].reshape(x1.size, x2.size)


kernal_l=0.2
cov_function=SquaredExponentialKernel(length=kernal_l)

model = GPR(X, y, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
prediction = model.predict(x1x2)
prediction = prediction.reshape(x1.size, x2.size)

fig = plt.figure(figsize=(10,10))

ax = fig.add_subplot(111, projection='3d')            
# surf = ax.plot_surface(X0p, X1p, prediction, rstride=1, cstride=1, cmap='jet', linewidth=0, antialiased=False)
ax.scatter3D(X[:,0], X[:,1], y)
ax.scatter3D(X[:,0], X[:,1], y_median)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")


plt.show()


# -------------------------------------------------------------------
# -------------------------------------------------------------------


data = pd.read_csv("/home/floaty/Floaty/ws/src/ros_test_pkg/src/data/recordings_upper_motor_5000.csv", header=None)

X = np.array(data.iloc[:,[0,1]])
y = -1*np.array(data.iloc[:,2])


x1 = np.linspace(X[:,0].min() - axis_margins, X[:,0].max() + axis_margins) #p
x2 = np.linspace(X[:,1].min() - axis_margins, X[:,1].max() + axis_margins) #q
x = (np.array([x1, x2])).T


x1x2 = np.array(list(product(x1, x2)))
X0p, X1p = x1x2[:,0].reshape(x1.size, x2.size), x1x2[:,1].reshape(x1.size, x2.size)

cov_function=SquaredExponentialKernel(length=kernal_l)

model = GPR(X, y, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
prediction = model.predict(x1x2)
prediction = prediction.reshape(x1.size, x2.size)

fig = plt.figure(figsize=(10,10))

ax = fig.add_subplot(111, projection='3d')            
surf = ax.plot_surface(X0p, X1p, prediction, rstride=1, cstride=1, cmap='jet', linewidth=0, antialiased=False)
ax.scatter3D(X[:,0], X[:,1], y)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")


plt.show()


# -------------------------------------------------------------------
# -------------------------------------------------------------------


data = pd.read_csv("/home/floaty/Floaty/ws/src/ros_test_pkg/src/data/recordings_bottom_motor_5000.csv", header=None)

X = np.array(data.iloc[:,[0,1]])
y = -1*np.array(data.iloc[:,2])


x1 = np.linspace(X[:,0].min() - axis_margins, X[:,0].max() + axis_margins) #p
x2 = np.linspace(X[:,1].min() - axis_margins, X[:,1].max() + axis_margins) #q
x = (np.array([x1, x2])).T


x1x2 = np.array(list(product(x1, x2)))
X0p, X1p = x1x2[:,0].reshape(x1.size, x2.size), x1x2[:,1].reshape(x1.size, x2.size)

cov_function=SquaredExponentialKernel(length=kernal_l)

model = GPR(X, y, white_noise_sigma=noise_sigma*2, covariance_function=cov_function)
prediction = model.predict(x1x2)
prediction = prediction.reshape(x1.size, x2.size)

fig = plt.figure(figsize=(10,10))

ax = fig.add_subplot(111, projection='3d')            
surf = ax.plot_surface(X0p, X1p, prediction, rstride=1, cstride=1, cmap='jet', linewidth=0, antialiased=False)
ax.scatter3D(X[:,0], X[:,1], y)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")

plt.show()

print("end")

