from remote import sim
import numpy as np
import sympy as sp
import cv2
import time


class Coppelia:
    def __init__(self, port=19990):
        self.client_id = self.__connect(port)
        self.__init_handles()
        self.__init_cups()

        # Medidas del robot - Ajustadas según el Coppelia
        self.L1 = 29.55
        self.L2 = 124.59
        self.L3 = 108
        self.L4 = 20.27
        self.L5 = 168.67
        self.E = 58.85  # Sin gripper 24.28

    def __connect(self, port):
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

    def __init_cups(self):
        """ 
        Init simulation blocks positions and handles 
        """
        cups_colors = ["Blue", "Red", "White", "Green"]
        quantity = len(cups_colors)
        self.cup_handlers = [sim.simxGetObjectHandle(
            self.client_id, f"./Cup{cups_colors[i]}/pickPos", sim.simx_opmode_blocking)[1] for i in range(quantity)]

        self.cups_pos = {cups_colors[i]: np.around(np.array(sim.simxGetObjectPosition(
            self.client_id, self.cup_handlers[i], -1, sim.simx_opmode_blocking)[1]), 5) for i in range(quantity)}

    def __init_handles(self):
        """
        Initializes handles for joints and other 
        parts of the robots
        """
        # Gripper handler
        self.gripper = sim.simxGetObjectHandle(
            self.client_id, "./Mirobot/vacuumGripper", sim.simx_opmode_blocking)[1]

        # Final Tip (efector Final) hanler
        self.tip = sim.simxGetObjectHandle(
            self.client_id, "./Mirobot/tip", sim.simx_opmode_blocking)[1]

        # Joints handler
        self.joints = [sim.simxGetObjectHandle(
            self.client_id, f"./Mirobot/joint{i+1}", sim.simx_opmode_blocking)[1] for i in range(6)]

        # Camera Handler
        self.camera = sim.simxGetObjectHandle(
            self.client_id, "./camera", sim.simx_opmode_oneshot_wait)[1]

        # Camera init
        _, _, _ = sim.simxGetVisionSensorImage(
            self.client_id, self.camera, 0, sim.simx_opmode_streaming)

        # Proximity Sensor handler
        self.proximity_sensor = sim.simxGetObjectHandle(
            self.client_id, "./proximity_sensor", sim.simx_opmode_blocking)[1]

    def __Tdh(self, d, theta, a, alpha):
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
            self.__Tdh(ds[i], thetas[i], a_s[i], alphas[i]) for i in range(6)]

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
        T01, T12, T23 = [self.__Tdh(ds[i], thetas[i], a_s[i], alphas[i])
                         for i in range(3)]
        T03 = sp.simplify(T01*T12*T23).applyfunc(lambda x: '{:.4f}'.format(x))
        T03 = np.array(T03, dtype=np.float64).squeeze()

        # Obtenemos q4,q5,q6
        R03 = T03[:3, :3]  # Matriz rotacional R03
        R30 = R03.T
        R06 = T06[:3, :3]  # Matriz rotacional R06
        R36 = np.dot(R30, R06)

        # Obtenemos los angulos de euler
        q5 = np.rad2deg(np.arcsin(R36[2, 2]))
        q4 = np.rad2deg(np.arctan2(-R36[1, 2], -R36[0, 2]))
        q6 = np.rad2deg(np.arctan2(-R36[2, 1], R36[2, 0]))

        return np.around(np.array([q1, q2, q3, q4, q5, q6]), 2)

    def move_robot(self, position):
        x, y, z = np.array(position)*1000
        T06 = np.array(
            [[1, 0, 0, x], [0, -1, 0, y], [0, 0, -1, z], [0, 0, 0, 1]])
        joints = self.inverse_kinematics(T06)
        self.move_joints(joints)
        time.sleep(1)

    def move_joints(self, q):
        """ 
         Move robot joins to specific angles. Prints
         tip position and calc DH model
         param: q -> Array of angles in degrees
        """
        # Executes for each join, each move
        for qval, qi in zip(q, self.joints):
            sim.simxSetJointTargetPosition(
                self.client_id, qi, np.deg2rad(qval), sim.simx_opmode_oneshot)

    def set_gripper(self, state):
        """ 
        Void Method to activate gripper 
        """
        sim.simxCallScriptFunction(self.client_id, './Mirobot', sim.sim_scripttype_childscript,
                                   'setGripperOn', [state], [], [], bytearray(), sim.simx_opmode_blocking)
        time.sleep(0.5)

    def set_conveyor(self, velocity=0):
        """ 
        Void Method to activate conveyor
        """
        sim.simxCallScriptFunction(self.client_id, './Mirobot', sim.sim_scripttype_childscript,
                                   'setConveyor', [], [velocity], [], bytearray(), sim.simx_opmode_blocking)

    def read_proximity_sensor(self):
        """ 
        Read Proximity Sensor values
        Return: Sensor state, obj_coordinates,obj_handle
        """
        state = sim.simxReadProximitySensor(
            self.client_id, self.proximity_sensor, sim.simx_opmode_blocking)[1]
        return state

    def classify_blocks(self):
        """ 
        This method shows Coppelia Camera Vision Sensor
        and Classifies White, Blue, Red, Green Colors
        """
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            self.client_id, self.camera, 0, sim.simx_opmode_buffer)
        color = ""
        if returnCode == 0 and image:
            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[0], resolution[1], 3])
            img = cv2.resize(img, (512, 512))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            HSV = {
                "Blue": ([110, 50, 50], [130, 255, 255]),
                "Red": ([0, 100, 100], [10, 255, 255]),
                "Green": ([25, 52, 72], [102, 255, 255]),
                "White": ([0, 0, 255], [0, 0, 255])
            }

            images = ["Blue", "Red", "Green", "White"]
            for img_name in images:
                img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower = np.array(HSV[img_name][0], np.uint8)
                upper = np.array(HSV[img_name][1], np.uint8)
                mask = cv2.inRange(img_hsv, lower, upper)

                kernel = np.ones((5, 5), "uint8")
                mask = cv2.dilate(mask, kernel)
                res = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
                gray_image = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
                _, hierarchy = cv2.findContours(
                    gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if hierarchy is not None:
                    color = img_name
            return color
