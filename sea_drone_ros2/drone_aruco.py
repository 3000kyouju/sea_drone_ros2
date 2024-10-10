import math
import time
import cv2
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String

class DroneControl(Node):
    def __init__(self, img_size):
        super().__init__('controller')
        self.img_size = img_size
        self.drone_p = self.target_p = np.array([-1,-1])
        self.drone_v = self.target_v = np.array([-1,-1])
        self.movingForward, self.movingAngle = 0.0, 0.0
        self.target_route = self.makeTargetRoute()
        self.target_route_number = 0
        self.my_recognize = False
        self.tg_recognize = False
        self.prev_time = time.time()
        self.sending_count = 0
        self.pub = self.create_publisher(String, 'controller', 10)
        dictionaly = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(dictionaly)
        self.id = 0
        self.corners, self.ids = None, None

    def makeTargetRoute(self):
        #target_route = np.array([[0.9,0.1], [0.9,0.9], [0.1,0.9], [0.1,0.1], [-1,-1]])           
        target_route = np.array([2,1,1,2,1,2])       
        return target_route
    
    def detectMarkers(self, frame):
        self.corners, self.ids, _ = self.detector.detectMarkers(frame)

    def getDronePoint(self):
        self.my_recognize = False
        if self.ids is not None:
            for id, corner in zip(self.ids, self.corners):
                if id[0] == self.id:
                    self.drone_p = (corner[0][0] + corner[0][1] + corner[0][2] + corner[0][3]) / 4 / self.img_size
                    self.drone_v = ((corner[0][0] + corner[0][1])/2 - (corner[0][2] + corner[0][3])/ 2) / self.img_size
                    self.my_recognize = True

    def getNextTarget(self,frame):
        self.tg_recognize = True
        if self.target_route_number == self.target_route.size:
            print("goal!")
            self.tg_recognize = False
            return
        self.target_p = self.getAruco(frame)
        if self.target_p[0] == -1:
            print("lost target")
            self.tg_recognize = False
            return
        self.target_v = self.target_p - self.drone_p
        target_l = np.linalg.norm(self.target_v, ord=2)
        if target_l < 0.13 and self.my_recognize:
            print("reach target!")
            self.target_route_number += 1
        
    def getAruco(self,frame):
        target_id = self.target_route[self.target_route_number]
        if self.ids is not None:
            for id, corner in zip(self.ids, self.corners):
                if id[0] == target_id:
                    return (corner[0][0] + corner[0][1] + corner[0][2] + corner[0][3]) / 4 / self.img_size
        return np.array([-1,-1])
            
    def get_drec(self, theta):
        if theta > 90:
            angle = theta / 40
            forward = 0
        elif theta > 10:
            angle = theta / 30
            forward = 0.03
        else:
            angle = 0
            forward = 0.08
        return forward, angle
            
    def getMovingDerection(self):
        if self.tg_recognize:
            cross = np.cross(self.drone_v, self.target_v)
            inner = np.inner(self.drone_v, self.target_v)
            #outer = np.outer(drone_v, target_v)
            drone_l = np.linalg.norm(self.drone_v, ord=2)
            target_l = np.linalg.norm(self.target_v, ord=2)
            vector_cos = inner / (drone_l * target_l)
            #theta = math.degrees(np.arccos(vector_cos))
            theta = np.rad2deg(np.arccos(np.clip(vector_cos, -1.0, 1.0)))
            forward, angle = self.get_drec(theta)
            if cross > 0:
                angle *= -1
            self.movingForward, self.movingAngle = forward, angle
            return True
        else:
            self.movingForward = 0.0
            self.movingAngle = 0.0
            return False
        
    def sendCommand(self):
        duration = time.time() - self.prev_time
        msg = String()
        if 0.1 < duration <= 2 and self.my_recognize:
            msg.data = "{},{}".format(self.movingForward, self.movingAngle)
            self.sending_count += 1
        else:
            self.movingForward, self.movingAngle = 0.0, 0.0
            msg.data = "{},{}".format(self.movingForward, self.movingAngle)
        self.prev_time = time.time()
        self.get_logger().info(msg.data)
        self.pub.publish(msg)

    def OverImage(self, img):
        yellow, green, blue= (0,255,255), (255,0,255), (255,0,0)
        cyan, magenta, white = (255,255,0), (255,0,255), (255, 255, 255)

        target_p = (self.target_p * self.img_size).astype(int)
        drone_p = (self.drone_p * self.img_size).astype(int)
        cv2.circle(img, tuple(target_p), 30, blue, thickness=-1) #target point
        if self.my_recognize and self.tg_recognize:
            if self.drone_v[0] != 0:
                drone_v = (self.drone_v * self.img_size).astype(int)
                drone_v = (drone_v / np.linalg.norm(drone_v) *100).astype(int)
                cv2.arrowedLine(img, tuple(drone_p), tuple(drone_v+drone_p), white, thickness=5)#drone direction  
            if self.target_v[0] != 0:
                target_v = (self.target_v * self.img_size).astype(int)
                target_v = (target_v / np.linalg.norm(target_v) * 100).astype(int)
                cv2.arrowedLine(img, tuple(drone_p), tuple(target_v+drone_p), magenta, thickness=5)#target direction
        img = cv2.aruco.drawDetectedMarkers(img, self.corners, self.ids)
        return img