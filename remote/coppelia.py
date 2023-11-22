from IPython.display import display
from remote import sim
import numpy as np
import sympy as sp
import time


class Coppelia:
    def __init__(self, port=19990):
        self.client_id = self.connect(port)
        self.__init_handles()
        self.__init_blocks(quantity=0)

        # Medidas del robot - Ajustadas según el Coppelia
        self.L1 = 29.55
        self.L2 = 124.59
        self.L3 = 108
        self.L4 = 20.27
        self.L5 = 168.67
        self.E = 24.28 #58.85  # Sin gripper 24.28

    def connect(self, port):
        """ 
        Connect to coppelia local server
        param: connectionport
        return: client id 
        """
        sim.simxFinish(-1)
        client_id = sim.simxStart("127.0.0.1", port, True, True, 2000, 5)

        if client_id == 0:
            print("Contectado a {}".format(port))
        else:
            print("No se pudo conectar")
        return client_id

    def __init_blocks(self, quantity=1):
        """ 
        Init simulation blocks positions and handles 
        """
        # Pickup Parts
        self.blocks = [sim.simxGetObjectHandle(
            self.client_id, f"./pickupPart{i+1}/pickPos", sim.simx_opmode_blocking)[1] for i in range(quantity)]

        # Block Positions
        self.blocks_pos = [np.around(np.array(sim.simxGetObjectPosition(
            self.client_id, self.blocks[i], -1, sim.simx_opmode_blocking)[1]), 5) for i in range(quantity)]

    def __init_handles(self):
        """
        Initializes handles for joints and other 
        parts of the robots
        """
        # Gripper
        self.gripper = sim.simxGetObjectHandle(
            self.client_id, "./Mirobot/vacuumGripper", sim.simx_opmode_blocking)[1]

        # Final Tip (efector Final)
        self.tip = sim.simxGetObjectHandle(
            self.client_id, "./Mirobot/tip", sim.simx_opmode_blocking)[1]

        # Joints
        self.joints = [sim.simxGetObjectHandle(
            self.client_id, f"./Mirobot/joint{i+1}", sim.simx_opmode_blocking)[1] for i in range(6)]

        #

    def move_joints(self, q):
        """ 
         Move robot joins to specific angles. Prints
         tip position and calc DH model
         param: q -> Array of angles in degrees
        """
        # Calc model using DH
        T06 = self.foward_kinematics(q)
        print(f"Matriz T06 calculada con DH utilizando: {q}\n")
        sp.pprint(T06)
        time.sleep(0.5)

        # Executes for each join, each move
        for qval, qi in zip(q, self.joints):
            sim.simxSetJointTargetPosition(
                self.client_id, qi, np.deg2rad(qval), sim.simx_opmode_oneshot)

        time.sleep(0.5)

        # Calc final position
        tip_position = np.around(np.array(sim.simxGetObjectPosition(
            self.client_id, self.tip, -1, sim.simx_opmode_blocking)[1]), 5)

        # Print final tip position
        print(f"\nPosición del Efector según Coppelia: {tip_position*1000}\n")

        # Return T06
        return np.array(T06, dtype=np.float64).squeeze()

    def Tdh(self, d, theta, a, alpha):
        """ 
        Function that calculates the result TH matrix 
        for DH method
        params: DH parameters
        return: Homogeneus Transform Matrix
        """
        T = sp.Matrix([[sp.cos(theta), -sp.cos(alpha)*sp.sin(theta), sp.sin(alpha)*sp.sin(theta), a*sp.cos(theta)],
                       [sp.sin(theta), sp.cos(alpha)*sp.cos(theta), -
                        sp.sin(alpha)*sp.cos(theta), a*sp.sin(theta)],
                       [0,            sp.sin(alpha),               sp.cos(
                           alpha),              d],
                       [0,            0,                           0,                          1]])
        return T

    def foward_kinematics(self, q):
        """ 
        Calculates de final tip position with
        the angles given
        params: q -> Array of angles in degrees
        return: Homogeneus Transform Matrix
        """
        q1, q2, q3, q4, q5, q6 = q
        ds = np.array([self.L2, 0, 0, self.L5, 0, self.E])
        thetas = np.array([q1, 90+q2, q3, 180+q4, 90+q5, q6])*(np.pi/180)
        a_s = np.array([self.L1, self.L3, self.L4, 0, 0, 0])
        alphas = np.array([90, 0, 90, 90, 90, 0])*(np.pi/180)

        # Transformaciones homogéneas
        T01, T12, T23, T34, T45, T56 = [
            self.Tdh(ds[i], thetas[i], a_s[i], alphas[i]) for i in range(6)]

        # Transformación homogénea final
        return sp.simplify(T01*T12*T23*T34*T45*T56).applyfunc(lambda x: '{:.4f}'.format(x))

    def inverse_kinematics(self, T06):
        """ 
        Calculates the angles required to move to T06
        params: T06 -> Transformation Matrix in meters
        return: Array of angles in degrees
        """
        pw = T06[:3, 3] - T06[:3, 2]*self.E
        wx, wy, wz = pw

        # Obtenemos q123 mediante el método geométrico
        q1 = np.rad2deg(np.arctan(wy/wx))

        # Obtenemos q2
        w = np.sqrt(wx**2 + wy**2)
        beta = np.arctan((wz - self.L2)/(w - self.L1))
        Lq2w = np.sqrt((w - self.L1)**2 + (wz - self.L2)**2)  # Hipotenusa
        Lq3w = np.sqrt(self.L4**2 + self.L5**2)  # Distancia oblicuo
        alpha = np.arccos(
            (self.L3**2 + Lq2w**2 - (Lq3w)**2)/(2*self.L3*Lq2w))
        q2 = np.rad2deg((beta + alpha) - (np.pi/2))

        # Obtenemos q3
        theta = np.arctan(self.L5/self.L4)
        gamma = np.arccos(
            (self.L3**2 + Lq3w**2 - (Lq2w)**2)/(2*self.L3*Lq3w))
        q3 = np.rad2deg((gamma + theta) - (np.pi))

        # Ontenemos R03
        # Defino mis parametros DH hasta q3
        ds = np.array([self.L2, 0, 0])
        thetas = np.array([q1, 90+q2, q3])*(np.pi/180)
        a_s = np.array([self.L1, self.L3, self.L4])
        alphas = np.array([90, 0, 90])*(np.pi/180)

        # Obtengo las transformaciones hasta 03
        T01, T12, T23 = [self.Tdh(ds[i], thetas[i], a_s[i], alphas[i])
                         for i in range(3)]
        T03 = sp.simplify(T01*T12*T23).applyfunc(lambda x: '{:.4f}'.format(x))
        T03 = np.array(T03, dtype=np.float64).squeeze()

        # Obtenemos q4,q5,q6
        R03 = T03[:3, :3]  # Matriz rotacional R03
        R30 = R03.T
        R06 = T06[:3, :3]  # Matriz rotacional R06
        R36 = np.dot(R30, R06)

        # Obtenemos los angulos de euler
        # q5 = np.rad2deg(np.arctan2(R36[2, 2], np.sqrt(1 - R36[2, 2]**2)))
        # q4 = np.rad2deg(np.arctan2(-R36[0, 2], -R36[1, 2]))
        # q6 = np.rad2deg(np.arctan2(R36[2, 0], -R36[2, 1]))
        # return np.around(np.array([q1, q2, q3, 90 - q4, q5, 90 - q6]), 2)

        # Obtenemos los angulos de euler
        q5 = np.rad2deg(np.arcsin(R36[2, 2]))
        q4 = np.rad2deg(np.arctan2(-R36[1, 2], -R36[0, 2]))
        q6 = np.rad2deg(np.arctan2(-R36[2, 1], R36[2, 0]))

        return np.around(np.array([q1, q2, q3, q4, q5, q6]), 2)

    def set_gripper(self, state=False):
        """ 
        Void Method to activate gripper 
        """
        pass
