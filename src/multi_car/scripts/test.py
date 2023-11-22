import matplotlib.pyplot as plt

import numpy as np
def fun1(x):
    A = 50.0048886846377

    B = 1.33041641228442

    C = 0.295627839582315

    D = -9.8828764268101
    return (A-D)/(1+np.power(x/C,B))+D

x = np.linspace(0,1,1000)
print(x[-2:-1])
y = fun1(x)
plt.plot(x,y)
plt.show()