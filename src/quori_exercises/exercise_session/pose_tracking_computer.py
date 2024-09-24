#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from pytz import timezone
from datetime import datetime
import numpy as np
from sensor_msgs.msg import Image
from config_computer import *
import cv2

import warnings
warnings.filterwarnings("ignore")

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

class PoseTracking:

    def __init__(self):
        self.sub = rospy.Subscriber("/astra_ros/devices/default/color/image_color", Image, self.callback)
        
        self.landmark_points = ['nose', 'left_eye_inner', 'left_eye', 'left_eye_outer', 'right_eye_inner', 'right_eye', 'right_eye_outer', 'left_ear', 'right_ear', 'mouth_left', 'mouth_right', 'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow', 'left_wrist', 'right_wrist', 'left_pinky', 'right_pinky', 'left_index', 'right_index', 'left_thumb', 'right_thumb', 'left_hip', 'right_hip', 'left_knee', 'right_knee', 'left_ankle', 'right_ankle', 'left_heel', 'right_heel', 'left_foot_index', 'right_foot_index']
        base_options = python.BaseOptions(model_asset_path='/home/roshni/quori_files/quori_ros/src/quori_exercises/exercise_session/pose_landmarker.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=False)
        self.pose_detector = vision.PoseLandmarker.create_from_options(options)
        
        # self.pose_detector = mp.solutions.pose.Pose(
        #             min_detection_confidence=0.5,  # have some confidence baseline
        #             min_tracking_confidence=0.5,
        #             model_complexity=0,)
        self.flag = False

    def calc_angle(self, vec_0, vec_1, angle_type):
        if angle_type == 'xy':
            angle = np.arctan2(vec_1[1], vec_1[1]) - \
                np.arctan2(-vec_0[1], vec_0[0])
        elif angle_type == 'yz':
            angle = np.arctan2(vec_1[1], vec_1[2]) - \
                np.arctan2(-vec_0[1], -vec_0[2])
        elif angle_type == 'xz':
            angle = np.arctan2(vec_1[2], vec_1[0]) - \
                np.arctan2(-vec_0[2], -vec_0[0])
        
        angle = np.abs(angle*180.0/np.pi)
        if angle > 180:
            angle = 360-angle

        return 180 - angle

    def callback(self, data):
        if not self.flag:
            return

        image = np.frombuffer(data.data, dtype=np.uint8).reshape(
            data.height, data.width, -1)
        cv2.imwrite('test.jpg', image) 
        image = mp.Image.create_from_file('test.jpg')

        results = self.pose_detector.detect(image)
        if results.pose_landmarks:
            ct = datetime.now(tz)

            landmarks = []
            for i, landmark in enumerate(results.pose_landmarks[0]):
                landmarks.append([landmark.x, landmark.y, landmark.z])

            angle_msg = Float64MultiArray()
            data = []
            
            for joint_group, angle_description in ANGLE_INFO:
                indices = [self.landmark_points.index(angle_description[i]) for i in range(3)]
                points = [np.array(landmarks[i]) for i in indices]
                vec_0 = points[0] - points[1]
                vec_1 = points[2] - points[1]

                for plane in ['xy', 'yz', 'xz']:
                    angle = self.calc_angle(vec_0, vec_1, plane)
                    data.append(angle)
            data.append(rospy.get_time())
            angle_msg.data = data
            angle_pub.publish(angle_msg)


if __name__ == '__main__':
    
    rospy.init_node('pose_tracking', anonymous=True)
    
    angle_pub = rospy.Publisher('joint_angles', Float64MultiArray, queue_size=10)
    face_pub = rospy.Publisher('facial_features', Float64MultiArray, queue_size=10)
    tz = timezone('EST')

    #Start with exercise 1, set 1
    pose_tracking = PoseTracking()
    inittime = datetime.now(tz)

    pose_tracking.flag = True
    rospy.spin()

    
    





