import numpy as np
import matplotlib.pyplot as plt
import sys

data = np.loadtxt("SLAM/sensorData/dataIMU.dat",unpack=False)
index = data[:,0]
aX = data[:,1]
aY = data[:,2]
aZ = data[:,3]
vYaw = data[:,6]

data = np.loadtxt("SLAM/sensorData/rw.dat", unpack = False)
indexV = data[:,0]
posX = data[:,1]
posY = data[:,2]
posZ = data[:,3]
vX = data[:,4]
vY = data[:,5]
vZ = data[:,6]

print np.mean(vYaw)


# aX[np.abs(aX) < 0.3] = 0
vYCalc = np.zeros(len(aX))
vYCalc[0] = vY[0]
dt = 1./256.
for i in range(1,len(aY)):
    vYCalc[i] = vYCalc[i-1] + 9.81 * dt * aY[i]#np.sqrt(aY[i]**2 + aX[i]**2)

fig = plt.figure()
ax =fig.add_subplot('111')
# axT = ax.twinx()

# ax.plot(index,aX, label="aX", color = "red")
# ax.plot(index,vYCalc, label="vYCalc", color = "red")
# ax.plot(index,aY, label="aY")
# plt.plot(index, vYaw, label="vYaw")
plt.plot(indexV,vY, label="vY", color = "green")
# ax.plot(index,aZ, label="aZ")
ax.legend()
# axT.legend()
plt.show()