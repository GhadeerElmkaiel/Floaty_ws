import numpy as np
import matplotlib.pyplot as plt
import time
import pandas as pd

def calc_circle_mesh(radius, mesh_size):
    mesh = []
    eps = 1e-7
    for i in np.arange(-radius, radius+eps, mesh_size):
        # for j in np.arange(-np.sqrt(radius**2 - i**2), np.sqrt(radius**2 - i**2)+eps, mesh_size):
        for j in np.arange(-radius, radius+eps, mesh_size):
            if j+eps>=-np.sqrt(abs(radius**2 - i**2)) and j-eps <= np.sqrt(abs(radius**2 - i**2)):
                mesh.append([i,j])
    return np.array(mesh)


def get_closest_points_no_rep(X, Y):
    remaining_points = Y.copy()  # create a copy of Y to keep track of remaining points
    closest_points = []
    for x in X:
        distances = np.sqrt(np.sum((remaining_points - x)**2, axis=1))
        idx = np.argmin(distances)
        closest_point = remaining_points[idx]
        closest_points.append(closest_point)
        remaining_points = np.delete(remaining_points, idx, axis=0)  # remove the closest point from the remaining points
    return np.array(closest_points)


def get_closest_points(X, Y):
    closest_points = []
    for x in X:
        distances = np.sqrt(np.sum((Y - x)**2, axis=1))
        closest_point = Y[np.argmin(distances)]
        closest_points.append(closest_point)
    return np.array(closest_points)

gap = 0.04
radius = 0.37

# pth_to_data = "/home/floaty/Floaty/ws/src/floaty_pkg/data/iterative_learning_algorithm/52_48_43_40_50_45.csv"
pth_to_data = "/home/floaty/Floaty/ws/src/floaty_pkg/data/iterative_learning_algorithm/47-Gaussian/iterative_learning_data_num_0.csv"

s = "{:02d}".format(1)
print(s)
s = "{:02d}".format(10)
print(s)
s = "{:03d}".format(1)
print(s)
s = "{:002d}".format(1)
print(s)

data = pd.read_csv(pth_to_data)
arr_1 = np.array(data.iloc[:,[0,1]])

# arr_1 = np.random.random((1500,2))
# arr_1 = np.array([[v[0]-0.5, v[1]-0.5] for v in arr_1])
arr_2 = calc_circle_mesh(radius, gap)
# arr_2 = np.random.random((400,2))

fig = plt.figure()
plt.scatter(arr_1[:,0], arr_1[:,1])
plt.scatter(arr_2[:,0], arr_2[:,1])

# plt.show()

ts = time.time_ns()
arr_3 = get_closest_points(arr_2, arr_1)
tend = time.time_ns()

print((tend-ts)*1e-9)

ts = time.time_ns()
arr_4 = get_closest_points_no_rep(arr_2, arr_1)
tend = time.time_ns()

print("Unique with rep: ",len(np.unique(arr_3)))
print("Unique without rep: ",len(np.unique(arr_4)))

print((tend-ts)*1e-9)



fig = plt.figure()
# plt.scatter(arr_2[:,0], arr_2[:,1])
plt.scatter(arr_3[:,0], arr_3[:,1])

fig = plt.figure()
# plt.scatter(arr_2[:,0], arr_2[:,1])
plt.scatter(arr_4[:,0], arr_4[:,1])

plt.show()
print("end")

