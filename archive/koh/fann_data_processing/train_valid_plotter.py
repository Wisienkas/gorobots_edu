import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("train_vali.dat", delimiter="\t")

# plt.rcParams.update({'font.size': 20})
plt.gcf().subplots_adjust(bottom=0.12)
#plt.style.use('seaborn-bright') # seaborn-bright
plt.plot(data[:,0], data[:,1], label="Training MSE training data", linewidth=4)
plt.plot(data[:,0], data[:,2], label="Training MSE validation data", linewidth=4)
# plt.plot(epoch, label="Validation MSE", linewidth=3)

plt.ylabel('MSE')
plt.xlabel('EPOCH')
plt.legend( loc='center right', numpoints = 1 )
plt.grid(True)
plt.savefig("train_valid.png")
plt.show()

print "\n"
