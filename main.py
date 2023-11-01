from remote.coppelia import Coppelia
import numpy as np
import time

if __name__ == '__main__':
    c = Coppelia()
    qo = [65, -45, 45, 0, 0, 0]
    T06 = c.foward_kinematics(qo)
    q = c.inverse_kinematics(np.array(T06, dtype=np.float64).squeeze())
    c.move_joints(q)
    time.sleep(2)
