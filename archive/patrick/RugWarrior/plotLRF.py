import numpy as np
import matplotlib.pyplot as plt
import sys

index, length = np.loadtxt("SLAM/sensorData/lrf" + sys.argv[1] + ".dat",unpack=True, skiprows  =1)

length[length ==0] = 5700

plt.plot(index, length)
plt.show()