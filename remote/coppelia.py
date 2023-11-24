from remote import sim
import numpy as np
import sympy as sp
import cv2
import time


class Client:
    def __init__(self, port: int = 19990) -> None:
        sim.simxFinish(-1)
        self.id = sim.simxStart("127.0.0.1", port, True, True, 2000, 5)

        if self.id == 0:
            print("Contectado a {}".format(port))
        else:
            print("No se pudo conectar")


class Object:
    def __init__(self, client_id: int, path: str, position=False):
        self.__path = path
        self.__client_id = client_id

        self.handler = sim.simxGetObjectHandle(
            self.__client_id, self.__path, sim.simx_opmode_blocking)[1]

        if position:
            self.position = np.around(np.array(sim.simxGetObjectPosition(
                self.__client_id, self.handler, -1, sim.simx_opmode_blocking)[1]), 5)


class Mirobot():
    def __init__(self, client_id):
        """ 
        Init class. Initializes handlers, objects
        and attributes 
        """
        self.client_id = client_id
        self.joints = [
            Object(self.client_id, f"./Mirobot/joint{i+1}") for i in range(6)]

        # Robot measurements
        self.__L1 = 29.55
        self.__L2 = 124.59
        self.__L3 = 108
        self.__L4 = 20.27
        self.__L5 = 168.67
        self.__E = 58.85  # Without gripper 24.28

    def __Tdh(self, d: float, theta: float, a: float, alpha: float) -> list:
        """ 
        Function that calculates the result TH matrix 
        for Denavit Hartenberg (DH) method

        args: 
            DH variables (d,theta,a,alpha)
        returns: 
            T: Homogeneus Transform Matrix
        """
        T = sp.Matrix([[sp.cos(theta), -sp.cos(alpha)*sp.sin(theta), sp.sin(alpha)*sp.sin(theta), a*sp.cos(theta)],
                       [sp.sin(theta), sp.cos(alpha)*sp.cos(theta), -
                        sp.sin(alpha)*sp.cos(theta), a*sp.sin(theta)],
                       [0,            sp.sin(alpha),               sp.cos(
                           alpha),              d],
                       [0,            0,                           0,                          1]])
        return T

    def __set_joints(self, desired_angles: list) -> None:
        """ 
         Void method to move robot joins to 
         desired angles

         args: 
            desired_angles: Array of desired angles per join in degrees
        """
        for angle, joint in zip(desired_angles, self.joints):
            sim.simxSetJointTargetPosition(
                self.client_id, joint.handler, np.deg2rad(angle), sim.simx_opmode_oneshot)

    def foward_kinematics(self, angles_per_joint: list) -> list:
        """ 
        Calculates de final tip position with 
        joint angles given

        args: 
            angles_per_joint: Array of 6 angles in degrees
        returns: 
            T06: Homogeneus Transform Matrix
        """
        q1, q2, q3, q4, q5, q6 = angles_per_joint
        ds = np.array([self.__L2, 0, 0, self.__L5, 0, self.__E])
        thetas = np.array([q1, 90+q2, q3, 180+q4, 90+q5, q6])*(np.pi/180)
        a_s = np.array([self.__L1, self.__L3, self.__L4, 0, 0, 0])
        alphas = np.array([90, 0, 90, 90, 90, 0])*(np.pi/180)

        # Homogeneous Transformations (T)
        T01, T12, T23, T34, T45, T56 = [
            self.__Tdh(ds[i], thetas[i], a_s[i], alphas[i]) for i in range(6)]

        # Homogeneous Transform T06
        T06 = sp.simplify(T01*T12*T23*T34*T45 *
                          T56).applyfunc(lambda x: '{:.4f}'.format(x))
        return T06

    def inverse_kinematics(self, T06) -> None:
        """ 
        Calculates the angles required to the robot
        to the desired orientation and position provided
        by the T06 matrix using Kinematic decouple method

        args: 
            T06: Transformation Matrix in Meters
        return: 
            calc_joints: Joints array of angles in degrees
        """
        # Wrist position
        pw = T06[:3, 3] - T06[:3, 2]*self.__E
        wx, wy, wz = pw

        # Obtain q1
        q1 = np.rad2deg(np.arctan(wy/wx))

        # Obtain q2
        w = np.sqrt(wx**2 + wy**2)
        beta = np.arctan((wz - self.__L2)/(w - self.__L1))
        Lq2w = np.sqrt((w - self.__L1)**2 + (wz - self.__L2)**2)  # Hipotenusa
        Lq3w = np.sqrt(self.__L4**2 + self.__L5**2)  # Distancia oblicuo
        alpha = np.arccos(
            (self.__L3**2 + Lq2w**2 - (Lq3w)**2)/(2*self.__L3*Lq2w))
        q2 = np.rad2deg((beta + alpha) - (np.pi/2))

        # Obtain q3
        theta = np.arctan(self.__L5/self.__L4)
        gamma = np.arccos(
            (self.__L3**2 + Lq3w**2 - (Lq2w)**2)/(2*self.__L3*Lq3w))
        q3 = np.rad2deg((gamma + theta) - (np.pi))

        # DH parameteres to obtain R03
        ds = np.array([self.__L2, 0, 0])
        thetas = np.array([q1, 90+q2, q3])*(np.pi/180)
        a_s = np.array([self.__L1, self.__L3, self.__L4])
        alphas = np.array([90, 0, 90])*(np.pi/180)

        # Get Transformations to T03
        T01, T12, T23 = [self.__Tdh(ds[i], thetas[i], a_s[i], alphas[i])
                         for i in range(3)]
        T03 = sp.simplify(T01*T12*T23).applyfunc(lambda x: '{:.4f}'.format(x))
        T03 = np.array(T03, dtype=np.float64).squeeze()

        # Obtain q4,q5,q6
        R03 = T03[:3, :3]
        R30 = R03.T
        R06 = T06[:3, :3]
        R36 = np.dot(R30, R06)

        q5 = np.rad2deg(np.arcsin(R36[2, 2]))
        q4 = np.rad2deg(np.arctan2(-R36[1, 2], -R36[0, 2]))
        q6 = np.rad2deg(np.arctan2(-R36[2, 1], R36[2, 0]))

        # Calculated Joints
        calc_joints = np.around(np.array([q1, q2, q3, q4, q5, q6]), 2)

        return calc_joints

    def move(self, position: list, orientation: list = [[1, 0, 0], [0, -1, 0], [0, 0, -1], [0, 0, 0]]) -> None:
        """ 
        Void method to move robot to desired position considering
        a vertical tip orientation as default. This method is used to
        move objets. Waits 1 second per execution

        args:
            position: Array with x,y,z positions in milimeters
            orientation: Rotational Matrix. Default is 90 deg
        """

        x, y, z = np.array(position)*1000  # mm to m
        orientation[0].append(x)
        orientation[1].append(y)
        orientation[2].append(z)
        orientation[3].append(1)

        # Homogeneous Transformation Matrix
        T06 = np.array(orientation, dtype=np.float64)

        # Calc. Necessary Angles
        joints = self.inverse_kinematics(T06)

        # Set Joints Angles
        self.__set_joints(joints)
        time.sleep(1)

    def set_gripper(self, state: int) -> None:
        """ 
        Void Method to activate gripper 
        waits 0.5 seconds each time

        args:
            state (int): 1 means True | 0 means False
        """
        sim.simxCallScriptFunction(self.client_id, './Mirobot', sim.sim_scripttype_childscript,
                                   'setGripperOn', [state], [], [], bytearray(), sim.simx_opmode_blocking)
        time.sleep(0.5)


class Camera():
    def __init__(self, client_id):
        self.__client_id = client_id
        self.__path = "./camera"
        self.__camera = sim.simxGetObjectHandle(
            self.__client_id, self.__path, sim.simx_opmode_blocking)[1]

        sim.simxGetVisionSensorImage(
            self.__client_id, self.__camera, 0, sim.simx_opmode_streaming)

    def color_classification(self):
        """ 
        This method shows Coppelia Camera Vision Sensor
        and Classifies White, Blue, Red, Green Colors

        returns:
            color: String value with color or None
        """
        returnCode, resolution, image = sim.simxGetVisionSensorImage(
            self.__client_id, self.__camera, 0, sim.simx_opmode_buffer)
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
            for color in images:
                img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower = np.array(HSV[color][0], np.uint8)
                upper = np.array(HSV[color][1], np.uint8)
                mask = cv2.inRange(img_hsv, lower, upper)

                kernel = np.ones((5, 5), "uint8")
                mask = cv2.dilate(mask, kernel)
                res = cv2.bitwise_and(img_hsv, img_hsv, mask=mask)
                gray_image = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
                _, hierarchy = cv2.findContours(
                    gray_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if hierarchy is not None:
                    return color


class Conveyor():
    def __init__(self, client_id: int):
        self.__client_id = client_id

    def set_velocity(self, velocity) -> None:
        """ 
        Void Method to set conveyor velocity.
        It's importanto to add a Dummy as a Parent
        to the original conveyor

        args:
            velocity (float): Velocity in m/s
        """
        sim.simxCallScriptFunction(self.__client_id, './Conveyor', sim.sim_scripttype_childscript,
                                   'setConveyor', [], [velocity], [], bytearray(), sim.simx_opmode_blocking)


class ProximitySensor():
    def __init__(self, client_id: int):
        self.__client_id = client_id
        self.sensor = Object(self.__client_id,"./proximity_sensor")

    def read(self) -> bool:
        """ 
        Read Proximity Sensor value

        returns: 
            state (Bool): True if sensor detects an obj. 
                          false if not
        """
        state = sim.simxReadProximitySensor(
            self.__client_id, self.sensor.handler, sim.simx_opmode_blocking)[1]
        return state
