#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

names = [
    'nose',
    'left_eye_inner',
    'left_eye',
    'left_eye_outer',
    'right_eye_inner',
    'right_eye',
    'right_eye_outer',
    'left_ear',
    'right_ear',
    'mouth_left',
    'mouth_right',
    'left_shoulder',
    'right_shoulder',
    'left_elbow',
    'right_elbow',
    'left_wrist',
    'right_wrist',
    'left_pinky',
    'right_pinky',
    'left_index',
    'right_index',
    'left_thumb',
    'right_thumb',
    'left_hip',
    'right_hip',
    'left_knee',
    'right_knee',
    'left_ankle',
    'right_ankle',
    'left_heel',
    'right_heel',
    'left_foot_index',
    'right_foot_index',
]

# landmark_to_index = {key:landmarks[i] for i,key in enumerate(index_to_landmark)}
landmarkDict = {}
counter = 0
stage = "down"
formType = "bad form"
exercise = "bicep curls"
view = "no view"

# POSE PREDICTIONS PRELIMINARY PUNCTIONS
def getPtCoords(lmName):
      if landmarkDict:
        point = landmarkDict[lmName]
        return point.x, point.y, point.z
def getPtVis(lmName):
      if landmarkDict:
          point = landmarkDict[lmName]
          return point.visibility
def calculate_angle(point1,point2,point3):
    x1, y1, z1 = getPtCoords(point1)
    x2, y2, z2 = getPtCoords(point2)
    x3, y3, z3 = getPtCoords(point3)
    radians = np.arctan2(y3-y2, x3-x2) - np.arctan2(y1-y2, x1-x2)
    angle = np.abs(radians*180.0/np.pi)
    if angle > 180.0:
        angle = 360-angle
    return angle

# DOING POSES
def bicepCurlAngles():
      left_sew_angle = calculate_angle('left_shoulder', 'left_elbow', 'left_wrist')
      right_sew_angle = calculate_angle('right_shoulder', 'right_elbow', 'right_wrist')
      return left_sew_angle, right_sew_angle
def dumbbellsidelateralraise():
      left_sew_angle = calculate_angle('left_hip', 'left_shoulder', 'left_wrist')
      right_sew_angle = calculate_angle('right_hip', 'right_shoulder', 'right_wrist')
      return left_sew_angle, right_sew_angle

def checkBicepCurlAngles(): #pass in variables instead of using global, return variables
    global formType, stage, counter, view
    left_sew_angle, right_sew_angle = bicepCurlAngles()
    isLeftVis = getPtVis('left_wrist') > 0.8
    isRightVis = getPtVis('right_wrist') > 0.7
    if isLeftVis and not isRightVis:
          # only check left side angles
          # mark view as leftsideview
          view = "left view"
          if (left_sew_angle > 175 or left_sew_angle < 10): # hyperextension
                formType = "bad form"
          else:
                formType = "good form"
                if left_sew_angle > 160:
                      stage = "down"
                if left_sew_angle < 30 and stage == "down":
                      stage = "up"
                      counter += 1
    elif isRightVis and not isLeftVis:
          # only check right side angles
          # mark view as rightsideview
          view = "right view"
          if (right_sew_angle > 175 or right_sew_angle < 10): # hyperextension
                formType = "bad form"
          else:
                formType = "good form"
                if right_sew_angle > 160:
                      stage = "down"
                if right_sew_angle < 30 and stage == "down":
                      stage = "up"
                      counter += 1
    elif isRightVis and isLeftVis:
          # check both angles
          # mark view as frontview
          view = "front view"
          if (left_sew_angle > 175 or right_sew_angle > 175 or left_sew_angle < 10 or
              right_sew_angle < 10): # hyperextension
                formType = "bad form"
          else:
                formType = "good form"
                if left_sew_angle > 160 or right_sew_angle > 160:
                      stage = "down"
                if (left_sew_angle < 30 or right_sew_angle < 30) and stage == "down":
                      stage = "up"
                      counter += 1
    else:
          # no view -> hands are not visible
          view = "no view"
    
    
def get_distance(p1, p2):
    x1, y1, z1 = p1.x, p1.y, p1.z
    x2, y2, z2 = p2.x, p2.y, p2.z
    
    return ((x1**2 - x2**2) + (y1**2 - y2**2) + (z1**2 - z2**2))**0.5

class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        self.pose = mp_pose.Pose(
                        static_image_mode=False,
                        model_complexity=0,
                        enable_segmentation=False,
                        smooth_landmarks=True,
                        min_detection_confidence=0.5)
        
        # subscribe to /astra_ros/devices/default/color/image_color sensor topic
        rospy.Subscriber("/astra_ros/devices/default/color/image_color",Image,self.callback)
        

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        results = self.pose.process(self.image)
        
        if not results.pose_landmarks:
            cv2.imshow('Frame',cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR))

            if cv2.waitKey(25) & 0xFF == ord('q'):
                exit()
        else:
            for i, landmark in enumerate(results.pose_landmarks.landmark):
                landmarkDict[names[i]] = landmark
            if exercise == "bicep curls":
                checkBicepCurlAngles()
            print("#####-----FEEDBACK-----######")
            if view == "no view":
                print("Please center yourself in the frame")
            else:
                print("View: ", view)
                print("Form: ", formType)
                print("Counter: ", counter)
                print("Stage: ", stage)
            print("-----------------------------")

        # print('NOSE \n', landmark_to_data['nose'])
        # print('LEFT EYE \n', landmark_to_data['left_eye'])
        # print('DISTANCE \n', get_distance(landmark_to_data['nose'], landmark_to_data['left_eye']))
        
        annot_image = self.image.copy()
        mp_drawing.draw_landmarks(
                    annot_image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        
        
        cv2.imshow('Frame',cv2.cvtColor(annot_image, cv2.COLOR_RGB2BGR))

        if cv2.waitKey(25) & 0xFF == ord('q'):
            exit()

        # cv2.imwrite('image.png', cv2.cvtColor(annot_image, cv2.COLOR_RGB2BGR))
        


    # def start(self):
    #     rospy.loginfo("Timing images")
    #     #rospy.spin()
    #     while not rospy.is_shutdown():
    #         rospy.loginfo('publishing image')
    #         #br = CvBridge()
    #         if self.image is not None:
    #             self.pub.publish(br.cv2_to_imgmsg(self.image))
    #         self.loop_rate.sleep()

if __name__ == "__main__":
    rospy.init_node('read_angles', anonymous=True)

    #Create the subscriber and then have callback function in the main scope

    nodo = Nodo()
    rospy.spin()