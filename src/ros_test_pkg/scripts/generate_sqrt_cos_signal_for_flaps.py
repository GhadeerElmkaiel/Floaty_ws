import math
import numpy as np


s = ""
for i in range(200):
    s= s+ "{:.2f}".format(math.sqrt(math.sqrt(math.sin(i/200.0*(2*3.14))*np.sign(math.sin(i/200.0*(2*3.14)))))*np.sign(math.sin(i/200.0*(2*3.14)))) +", "

print(s)