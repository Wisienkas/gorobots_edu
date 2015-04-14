import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import sys

index, posX, posY, sizeX, sizeY, occupied = np.loadtxt(str(sys.argv[1]), unpack=True)
index = index.astype(int)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_xlim([np.min(posX)-1,np.max(posX)+1])
ax.set_ylim([np.min(posY)-1,np.max(posY)+1])

for i in range(index[-1]+1):
    box = matplotlib.patches.Rectangle((posX[i],posY[i]), sizeX[i], sizeY[i], color = str(1 - occupied[i]), zorder = 0)
    ax.add_patch(box)

ax.scatter(0,0)
plt.show()
