from remote.coppelia import Coppelia
import numpy as np
import time

if __name__ == '__main__':
    c = Coppelia()
    x, y, z = np.array([200 - 25, 100, 90 + 35])
    T06 = [[1,  0,  0,  x],
           [0,  -1, 0,  y],
           [0,  0,  -1, z],
           [0,  0, 0,  1]]

    q = c.inverse_kinematics(np.array(T06, dtype=np.float64).squeeze())
    c.move_joints(q)
    time.sleep(2)
