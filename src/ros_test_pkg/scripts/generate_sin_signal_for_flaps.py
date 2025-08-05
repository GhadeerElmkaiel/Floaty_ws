import math


s = ""
for i in range(200):
    s= s+ "{:.2f}".format(math.sin(i/200.0*(2*3.14))*0.5) +", "

print(s)