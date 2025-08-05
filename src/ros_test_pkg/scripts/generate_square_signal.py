import math
import numpy as np


len_signal = 200
slop_percentage = 15

# First value is zero

num_slop_points = int(slop_percentage*len_signal/200)
slop_points = [2*i/(num_slop_points+1)-1 for i in range(1,num_slop_points+1)]

num_top_edge = int(len_signal/2-num_slop_points)
num_bottom_edge = len_signal-2*num_slop_points-num_top_edge

s = ""

starting_point = int(num_slop_points/2)
for i in range (starting_point, num_slop_points):
    s = s+ "{:.2f}, ".format(slop_points[i])

for i in range(num_top_edge):
    s = s+ "1.00, "

for i in range (num_slop_points):
    s = s+ "{:.2f}, ".format(slop_points[-i-1])

for i in range(num_bottom_edge):
    s = s+ "-1.00, "

for i in range (starting_point):
    s = s+ "{:.2f}, ".format(slop_points[i])


print(s)
