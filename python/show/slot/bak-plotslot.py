import matplotlib.pyplot as plt
import numpy as np

# p0x:0.855854 p0y:-0.368011 p1x:-2.331193 p1y:0.559654 p2x:-8.239580 p2y:-3.492483 p3x:-4.913500 p3y:-4.425000
slotx = np.array([0.855854, -2.331193,-8.239580,-4.913500,0.855854])
sloty = np.array([-0.368011,0.559654,-3.492483,-4.425000,-0.368011])
plt.figure('slot')
plt.plot(slotx, sloty)
plt.axis('equal')       #x,y轴等比
plt.show()
