from xml.etree.ElementTree import PI
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R



kst_range = 123
emax_range = 105
dpwoer_range = 113

signal_range = dpwoer_range
df = pd.read_csv("/home/gelmkaiel/Floaty/ws/src/ros_test_pkg/src/Motors_response_tests/KST/angles_test_2.csv")
np_arr = df.to_numpy()
np_arr[:,0] = (np_arr[:,0]-4000)*signal_range/4000

# max_val = max(np_arr[:,1])
# min_val = min(np_arr[:,1])



quaternions = np_arr[:,1:]

rotations = []

for i in range(quaternions.shape[0]):
    r = R.from_quat(quaternions[i])
    rotations.append(r)


initial_rot_matrix = np.array(rotations[0].as_matrix())

relative_rotations = []
for i in range(quaternions.shape[0]):
    rot_matrix = np.array(rotations[i].as_matrix())
    new_rot_matrix = np.matmul(rot_matrix.transpose(), initial_rot_matrix)
    new_rotation_val = R.from_matrix(new_rot_matrix)
    relative_rotations.append(new_rotation_val.magnitude())

rotations_in_deg = [x*180/np.pi for x in relative_rotations]

all_same_levels=[]
same=False
new_level=[]
for i in range(2,len(rotations_in_deg)):
    x = rotations_in_deg[i]
    x1 = rotations_in_deg[i-1]
    x2 = rotations_in_deg[i-2]
    if abs(x-x1)<1 and abs(x-x2)<1:
        new_level.append(x)
        if not same:
            same=True

    else:
        if same == True:
            same = False
            all_same_levels.append(new_level)
        new_level = []

avarage_levels = [np.average(x) for x in all_same_levels]
length = np_arr.shape[0]
freq = 1000
x = [i/freq for i in range(length)]
plt.plot(x, np_arr[:,0], label = "Motor command")
plt.scatter(x, rotations_in_deg, s= 2, c="r", label = "Optitrack")
plt.legend(loc='lower right')
plt.xlabel("Time 'second'")
plt.ylabel("Angle 'degrees'")
plt.show()
print("done!")