import numpy as np
from itertools import product
import pandas as pd
import os

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import matplotlib.pyplot as plt


# Get the home directory of the current user
home_directory = os.path.expanduser('~')

# Get the username from the home directory path
username = os.path.basename(home_directory)

# import matplotlib.pyplot as plt
# from sklearn.gaussian_process import GaussianProcessRegressor
# from sklearn.gaussian_process.kernels import RBF


# X = np.linspace(start=0, stop=10, num=1_000).reshape(-1,1)
# y = np.squeeze(X*np.sin(X))

# plt.plot(X, y, label=r"$f(x) = y$", linestyle="dotted")
# plt.legend()
# plt.xlabel("$x$")
# plt.ylabel("f(x)")
# plt.show()

# rng = np.random.RandomState(21)
# training_indices = rng.choice(np.arange(y.size), size=6, replace=False)
# X_train, y_train = X[training_indices], y[training_indices]

# kernel = 1 * RBF(length_scale=1.0, length_scale_bounds=(1e-2, 1e2))
# gaussian_process = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9)
# gaussian_process.fit(X_train, y_train)
# gaussian_process.kernel_

# mean_prediction, std_prediction = gaussian_process.predict(X, return_std=True)

# plt.plot(X, y, label=r"$f(x) = x \sin(x)$", linestyle="dotted")
# plt.scatter(X_train, y_train, label="Observations")
# plt.plot(X, mean_prediction, label="Mean prediction")
# plt.fill_between(
#     X.ravel(),
#     mean_prediction - 1.96 * std_prediction,
#     mean_prediction + 1.96 * std_prediction,
#     alpha=0.5,
#     label=r"95% confidence interval",
# )
# plt.legend()
# plt.xlabel("$x$")
# plt.ylabel("$f(x)$")
# _ = plt.title("Gaussian process regression on noise-free dataset")
# plt.show()

# l = input("enter:")
# pth = os.path.abspath(f"/home/{username}/Floaty/ws/src/ros_test_pkg/src/recordings_1.csv")
# print(pth)


# -----------------------------------------------------------------
# -----------------------------------------------------------------


# Test data
n = 50
Xtest = np.linspace(-5, 5, n).reshape(-1,1)

# Define the kernel function
def kernel(a, b, param):
    sqdist = np.sum(a**2,1).reshape(-1,1) + np.sum(b**2,1) - 2*np.dot(a, b.T)
    return np.exp(-.5 * (1/param) * sqdist)

param = 0.1
K_ss = kernel(Xtest, Xtest, param)

fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111)

x1x2 = np.array(list(product(Xtest, Xtest)))
X0p, X1p = x1x2[:,0].reshape(50,50), x1x2[:,1].reshape(50,50)
ax.pcolormesh(X0p, X1p, K_ss)
plt.show()
# Get cholesky decomposition (square root) of the
# covariance matrix
L = np.linalg.cholesky(K_ss + 1e-15*np.eye(n))

test_val = np.random.normal(size=(n,3))
# Sample 3 sets of standard normals for our test points,
# multiply them by the square root of the covariance matrix

f_prior = np.dot(L, np.random.normal(size=(n,3)))

# Now let's plot the 3 sampled functions.
plt.plot(Xtest, f_prior)
plt.axis([-5, 5, -3, 3])
plt.title('Three samples from the GP prior')
plt.show()

# -----------------------------------------------------------------
# -----------------------------------------------------------------




data = pd.read_csv(f"/home/{username}/Floaty/ws/src/ros_test_pkg/src/data/recordings_test_6000.csv", header=None)
print(data.head)

X = np.array(data.iloc[:,[0,1]])
y = np.array(data.iloc[:,2])

# kernel = 1 * RBF(length_scale=1.0, length_scale_bounds=(1e-2, 1e2))
# kernel = 1 * RBF(length_scale=1.0, length_scale_bounds=(1e-1, 5e-1))
kernel = 1 * RBF(length_scale=1.0, length_scale_bounds=(1e-1, 2e-1))
gaussian_process = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9)
gaussian_process.fit(X, y)
gaussian_process.kernel_

axis_margins = -0.13

x1 = np.linspace(X[:,0].min() - axis_margins, X[:,0].max() + axis_margins) #p
x2 = np.linspace(X[:,1].min() - axis_margins, X[:,1].max() + axis_margins) #q
x = (np.array([x1, x2])).T


x1x2 = np.array(list(product(x1, x2)))

y_pred, MSE = gaussian_process.predict(x1x2, return_std=True)


X0p, X1p = x1x2[:,0].reshape(50,50), x1x2[:,1].reshape(50,50)
Zp = np.reshape(y_pred,(50,50))

fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111)
ax.pcolormesh(X0p, X1p, Zp)
plt.scatter(X[:,0], X[:,1], c='r')
plt.show()

ax = fig.add_subplot(111, projection='3d')            
surf = ax.plot_surface(X0p, X1p, Zp, rstride=1, cstride=1, cmap='jet', linewidth=0, antialiased=False)
ax.scatter3D(X[:,0], X[:,1], y)
print("end")