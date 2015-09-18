import numpy as np
import matplotlib.pyplot as plt
import sys

filename = "sensorData/sensorData.dat"
data = np.loadtxt(filename,unpack=False)
index = data[:,0]
posX = data[:,1]
posY = data[:,2]
yaw = data[:,3]
vX = data[:,4]
vY = data[:,5]
vYaw = data[:,6]
aX = data[:,7]
aY = data[:,8]

n = len(index)

startIndex = 10
vXCalc = np.zeros(n)
vXCalc[startIndex] = vX[startIndex]
vYCalc = np.zeros(n)
vYCalc[startIndex] = vY[startIndex]
posXCalc = np.zeros(n)
posYCalc = np.zeros(n)
dt = 0.2

for i in range(startIndex+1,n):
    vXCalc[i] = vXCalc[i-1] + aX[i-1] * dt
    posXCalc[i] = posXCalc[i-1] + vXCalc[i-1] * dt + 0.5 * aX[i-1] * dt**2
    vYCalc[i] = vYCalc[i-1] + aY[i-1] * dt
    posYCalc[i] = posYCalc[i-1] + vYCalc[i-1] * dt + 0.5 * aY[i-1] * dt**2

fig = plt.figure()
ax = fig.add_subplot('111')
arg = str(sys.argv[1])
if arg == "acc":
    ax.plot(index, aX, label = "aX")
    ax.plot(index, aY, label = "aY")
elif arg == "vel": 
    ax.plot(index[startIndex:], vX[startIndex:], label = "vX")
#     ax.plot(index, vY,label = "vY")
    ax.plot(index[startIndex:], vXCalc[startIndex:], label = "vXCalc")
#     ax.plot(index, vYCalc, label = "vYCalc")
#     ax.plot(index, vYaw, label = "vYaw")
elif arg == "pos":
    ax.scatter(posX, posY)
elif arg == "yaw":
    yaw *= 180/np.pi
#     yaw += 90
    ax.plot(index, yaw, label = "yaw")
else:
    print "Wrong argument"
ax.legend()
plt.show()