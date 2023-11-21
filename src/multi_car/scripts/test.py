import matplotlib.pyplot as plt

import numpy as np

x = np.linspace(0,1,1000)
y = 16*x**3-8*x**2-18*x+10
plt.plot(x,y)
plt.show()