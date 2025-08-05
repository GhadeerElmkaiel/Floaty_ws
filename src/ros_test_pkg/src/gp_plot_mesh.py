import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import ndimage
from floaty_msgs.srv import speed_update_srv, speed_update_srvResponse
import math

def calc_circle_mesh(radius, mesh_size):
    mesh = []
    eps = 1e-7
    for i in np.arange(-radius, radius+eps, mesh_size):
        # for j in np.arange(-np.sqrt(radius**2 - i**2), np.sqrt(radius**2 - i**2)+eps, mesh_size):
        for j in np.arange(-radius, radius+eps, mesh_size):
            if j+eps>=-np.sqrt(abs(radius**2 - i**2)) and j-eps <= np.sqrt(abs(radius**2 - i**2)):
                mesh.append([i,j])
    return np.array(mesh)



def gp_service():
    
    # --------------------------------------------
    mesh = calc_circle_mesh(0.2, 0.02)
    fig = plt.figure(figsize=(10,10))
    # ax = fig.add_subplot(111, projection='3d')  
    



    # y = calc_motor_gaussian([0, 0], 3, mesh, 0.15)
    # fig = plt.figure(figsize=(12,12))
    plt.scatter(mesh[:,0], mesh[:,1], s=25, c="red")
    plt.show()
    # ax = fig.add_subplot(111, projection='3d')  

    # ax.scatter(mesh[:,0], mesh[:,1], y)

    # plt.show()
    print("done")



if __name__ == '__main__':
    gp_service()