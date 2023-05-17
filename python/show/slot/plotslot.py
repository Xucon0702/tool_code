import matplotlib.pyplot as plt
import numpy as np

# p0x:0.855854 p0y:-0.368011 p1x:-2.331193 p1y:0.559654 p2x:-8.239580 p2y:-3.492483 p3x:-4.913500 p3y:-4.425000
slot1x = np.array([0.855854, -2.331193,-8.239580,-4.913500,0.855854])
slot1y = np.array([-0.368011,0.559654,-3.492483,-4.425000,-0.368011])

# slot2x = np.array([1, 3,5,2,1])
# slot2y = np.array([2,3,6,9,2])

slot_hpa_x = np.array([-0.143890,-1.930719,-8.113486,-6.279337,-0.143890])
slot_hpa_y = np.array([-2.833926,-0.495329,-1.777503,-4.084842,-2.833926])

slot_psd_x = np.array([-1.354902,-1.928298])
slot_psd_y = np.array([-2.526098,0.032128])

plt.figure('slot')
# plt.plot(slot1x, slot1y)

plt.plot(slot_hpa_x, slot_hpa_y)
plt.plot(slot_psd_x, slot_psd_y)

plt.axis('equal')       #x,y轴等比
plt.show()
