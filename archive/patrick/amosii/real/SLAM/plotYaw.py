import numpy as np
import matplotlib.pyplot as plt

dataDirectory = "data/"
indexParticle, posXParticle, posYParticle, yaw = np.loadtxt(dataDirectory + "particles.dat", unpack=True)

yaw = ( yaw + np.pi) % (2 * np.pi ) - np.pi
yaw *= 180/np.pi

plt.plot(indexParticle, yaw)
plt.show()