#!/usr/bin/env python3

import numpy as np
import random
import sys, threading
import math
import os
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from vectors import Vector
from faceAnimation import FaceAnimation
from faceFeature import FaceFeature
from emotions import Emotion
# from worker import Worker

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import rospy
import cv2

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        #set size of window
        self.setGeometry(0, 0, 600, 600)
        self.setStyleSheet("background-color: white;") 
        self.setWindowTitle('Face Simulation')
        self.layout = QVBoxLayout()
        self.widget = QWidget(self)

        self.widget.setLayout(self.layout)
        
        # Initialize frame number
        self.frame_number = 0
        # Specify the output directory
        self.output_dir = "Frames/"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

        #TIMER
        #specify frames per second
        self.fps = 15
        self.animationDuration = 20 #how long does animation take
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.animation)
        self.timeElapsed = 0
        self.timer.start(10)
        self.threadpool = QThreadPool()

        #keeps track of all of the widgets in Main Window
        self.widgets = []

        #keep track of all of the properties that will be changing with each call to function
        #number of seconds to hold the emotion
        self.numSecondsToHoldEmotion = 90

        #TODO: get speed from time to show emotion
        self.speed = 1
        self.timeToShowEmotion = 70
        #direction of movement - negative when moving back to neutral 
        self.direction = 1

        #define the emotions
        self.joy = Emotion("joy", {"Inner Occipitofrontalis": 0.3, "Outer Occipitofrontalis": 0.3, 
        "Orbicularis Oculi": 0.8, "Levator Labii":0.5, "Zygomaticus": 1.0})
        self.sadness = Emotion("sadness", {"Inner Occipitofrontalis": 1.0, "Outer Occipitofrontalis":0.2, 
           "Corrugator": 0.1, "Procerus": 1.0, "Orbicularis Oculi": 1.0, 
           "Levator Labii": 0.5, "Levator Palpabrae": 0.8, "Buccinator":0.5, 
           "Orbicularis Oris": 0.5, "Depressor Anguli Oris": 1.0, "Mentalis":1.0})
        self.anger = Emotion("anger", {"Corrugator": 1.0, "Procerus":1.0, "Orbicularis Oculi":0.5, "Levator Labii":1.0,
         "Buccinator":0.8, "Orbicularis Oris": 0.5, "Depressor Anguli Oris":0.3})
        self.disgust = Emotion("disgust",{"Corrugator":1.0, "Procerus":0.5,"Orbicularis Oculi":0.8, "Levator Labii":1.0, 
           "Orbicularis Oris":1.0, "Depressor Anguli Oris": 1.0, "Mentalis":1.0})
        self.fear = Emotion("fear",{"Inner Occipitofrontalis":1.0, "Outer Occipitofrontalis":0.3, "Corrugator":1.0, 
        "Procerus":0.5, "Orbicularis Oculi": 0.8, "Levator Labii":1.0, "Levator Palpabrae":1.0,
        "Buccinator":0.8, "Orbicularis Oris":0.5, "Depressor Anguli Oris":0.7, 
        "Mentalis":1.0, "Jaw":0.8})
        self.surprise = Emotion("surprise",{"Inner Occipitofrontalis":0.5, "Outer Occipitofrontalis":0.8, "Levator Labii": 0.8, 
            "Levator Palpabrae":0.7, "Orbicularis Oris":1.0, "Depressor Anguli Oris":0.3, "Mentalis":0.3,
            "Jaw":1.0})
        
        self.currEmotion = self.sadness
        self.currPercentOfEmotion = 0.2
        self.currEmotion.modifyEmotionValuesByPercent(self.currPercentOfEmotion)
        self.isNeutral = True


        #have a bank of starting neutral positions?
        #TODO: have the ability to start at different neutral positions
        #change the vector start points according to the end points of an emotion
        self.startingNeutral = None

        self.emotionSub = rospy.Subscriber('quori/face_generator_emotion', Float64MultiArray, self.emotionCallback, queue_size=10)
    
    #adds widgets to the main window
    def addWidgetToLayout(self, widget):
        self.clearLayout()
        self.layout.addWidget(widget)
        self.widgets = self.widgets + [widget]
    
    #clears widgets from layout, important if restarting app with different emotion
    def clearLayout(self):
        for i in reversed(range(self.layout.count())):
            item = self.layout.takeAt(i)
            if item.widget():
                item.widget().deleteLater()
        
    def animation(self):
        # if self.isNeutral:
        #     self.askForUserInput()
        for w in self.widgets:
            
            # if self.timeElapsed >= self.numSecondsToHoldEmotion + self.timeToShowEmotion:
            #     self.direction = -1
            # if self.timeElapsed % 20 == 0:
                # print(self.timeElapsed, 'target', self.numSecondsToHoldEmotion + self.timeToShowEmotion)
                # print(self.currEmotion.name, self.direction, self.speed, self.currPercentOfEmotion)
            # print(self.direction)
            w.moveWidget(self.currEmotion, self.direction, self.speed, self.currPercentOfEmotion)
        self.update()
        self.timeElapsed += 1
        # if not self.isNeutral:
        # print(self.timeElapsed)
        #set neutral back to true after enough time
        if self.timeElapsed >= self.numSecondsToHoldEmotion + self.timeToShowEmotion:
            self.isNeutral = True
            self.timeElapsed = 0
            # # Add a small delay (you can adjust this value if needed)
            self.direction = 1
            # self.currPercentOfEmotion = 0.2
            # self.currEmotion = self.joy
            # self.currEmotion.modifyEmotionValuesByPercent(self.currPercentOfEmotion)

        # Capture and save the frame
        self.capture_frame()

    def capture_frame(self):
        pixmap = self.widget.grab()  # Capture the current frame
        frame_path = os.path.join(self.output_dir, f"frame_{self.frame_number:04d}.png")
        pixmap.save(frame_path)  # Save the frame as an image
        # self.frame_number += 1

        robot_image = cv2.imread(frame_path)
        robot_image = robot_image[100:-100, 100:-100, :]
        robot_image = cv2.resize(robot_image, (1000, 750), interpolation = cv2.INTER_AREA)
        image_center = tuple(np.array(robot_image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, -2, 1.0)
        robot_image = cv2.warpAffine(robot_image, rot_mat, robot_image.shape[1::-1], flags=cv2.INTER_LINEAR)

        robot_image = cv2.rotate(robot_image, cv2.ROTATE_90_CLOCKWISE)
        robot_image[:150,:,:] = 255
        robot_image[-150:,:,:] = 255
        robot_image[:,:100,:] = 255
        robot_image[:,-100:,:] = 255

        robot_image = cv2.copyMakeBorder(
                robot_image, 
                0, #top - corresponds to left side of the face
                55, #bottom - corresponds to right side of the face
                0, #left - corresponds to bottom of the face
                0, #right - corresponds to top of the face
                cv2.BORDER_CONSTANT, 
                value=[255,255,255]
            )
        
        cv2.namedWindow("face", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("face", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("face", robot_image)
        cv2.waitKey(10)
    
    def emotionCallback(self, data):
        print('Heard', data.data)
        emotions = ["joy", "sadness", "anger", "disgust", "fear", "surprise"]
        current_emotion = ''
        for e_ind, e in enumerate(emotions):
            if data.data[e_ind] > 0:
                current_emotion = e
                percentage = data.data[e_ind]
        
            
        print('Heard {} emotion at {} level'.format(current_emotion, percentage))
        emotions = {"joy": self.joy, "anger": self.anger, "fear": self.fear, "disgust":self.disgust, "surprise":self.surprise, "sadness": self.sadness}

        if not self.currEmotion.name == emotions[current_emotion].name:
            self.direction = -1
            self.currPercentOfEmotion = 0.01
            self.currEmotion.modifyEmotionValuesByPercent(self.currPercentOfEmotion)
            self.isNeutral = False
            self.timeElapsed = 0
            print(self.direction, self.currEmotion.name, self.currPercentOfEmotion)
            self.update()
            rospy.sleep(1)

            self.direction = 1
            self.currEmotion = emotions[current_emotion]
            self.currPercentOfEmotion = percentage
            self.currEmotion.modifyEmotionValuesByPercent(self.currPercentOfEmotion)
            self.isNeutral = False
            self.timeElapsed = 0
            print(self.direction, self.currEmotion.name, self.currPercentOfEmotion)
            self.update()

        else:
            if self.currPercentOfEmotion > percentage:
                self.direction = -1
            else:
                self.direction = 1

            self.currEmotion = emotions[current_emotion]
            self.currPercentOfEmotion = percentage
            self.currEmotion.modifyEmotionValuesByPercent(self.currPercentOfEmotion)
            print(self.direction, self.currEmotion.name, self.currPercentOfEmotion)
            self.isNeutral = False
            self.timeElapsed = 0
            self.update()


    # def getInput(self):
    #     emotions = ["joy", "sadness", "anger", "disgust", "fear", "surprise"]
    #     userInputEmotion = None
    #     userInputPercentage = None

    #     # Read from the text file
    #     with open('emotionInput.txt', 'r') as file:
    #         line = file.readline().strip()
    #         emotion, percentage = line.split()
    #         if emotion not in emotions:
    #             print("Invalid emotion. Please enter a valid emotion from the list:", emotions)
    #             return None
    #         try:
    #             percentage = float(percentage)
    #         except ValueError:
    #             print("Invalid percentage. Please enter a valid number.")
    #             return None
    #         return emotion, percentage

    # def askForUserInput(self):
    #     self.worker = Worker(self.getInput)
    #     self.worker.signals.result.connect(self.eventUpdateEmotion)
    #     self.worker.signals.finished.connect(self.eventWorkerFinished)

    #     self.threadpool.start(self.worker)


    # def eventWorkerFinished(self):
    #     print("Worker thread complete")
    # def eventUpdateEmotion(self, val):
    #     emotions = {"joy": self.joy, "anger": self.anger, "fear": self.fear, "disgust":self.disgust, "surprise":self.surprise, "sadness": self.sadness}
    #     emotion, percentage = val
    #     self.currEmotion = emotions[emotion]
    #     self.currPercentOfEmotion = percentage
    #     self.isNeutral = False
    #     self.timeElapsed = 0
    #     self.update()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    
    eyebrowLeftPoints = [(164,186),(192,177), (257,188)]
    #each list of vectors at index i should correspond to the vectors that pull on the same point at index i
    eyebrowLeftVectors = [[((164,186), (164,172), "Outer Occipitofrontalis")],
                        [((192,177),(189,161),"Outer Occipitofrontalis"), ((192,177),(199,156), "Inner Occipitofrontalis"), ((192,177),(210, 182), "Corrugator"), ((192, 177),(218,197), "Procerus")],
                        [((257,188),(255,180), "Outer Occipitofrontalis"), ((257,188),(257,154), "Inner Occipitofrontalis"), ((257,188),(274,194), "Corrugator"),((257,188),(254,208), "Procerus")] ]

    eyebrowRightPoints = [(330,188),(395,177), (422,186)]
    eyebrowRightVectors = [[((330,188),(343,180), "Outer Occipitofrontalis"), ((330,188),(331,154), "Inner Occipitofrontalis"), ((330,188),(311,194), "Corrugator"), ((330,188),(337,208), "Procerus")],
                        [((395,177),(398,161), "Outer Occipitofrontalis"), ((395,177),(388,156), "Inner Occipitofrontalis"), ((395,177),(375, 182), "Corrugator"), ((395,177),(370, 197), "Procerus")],
                        [((422,186), (423,172), "Outer Occipitofrontalis")]
                        ]

    rightEyePoints = [(185,224), (198,206), (235, 206), (248,224)]
    leftEyePoints = [(338,224), (352, 206), (389,206), (400,224)]

    rightEyeVectors = [[((185,224), (191, 224), "Orbicularis Oculi")],
                    [((198,206), (200, 211), "Orbicularis Oculi"), ((198,206), (205, 199), "Levator Palpabrae")],
                    [((235, 206), (230, 211), "Orbicularis Oculi"), ((235, 206), (226, 198), "Levator Palpabrae")],
                    [((248,224), (242, 224), "Orbicularis Oculi")]
                    ]
    
    leftEyeVectors = [[((338,224), (344, 224), "Orbicularis Oculi")],
                    [((352, 206), (356, 211), "Orbicularis Oculi"), ((352, 206), (360, 198), "Levator Palpabrae")],
                    [((389,206), (385, 211), "Orbicularis Oculi"), ((389,206), (380, 199), "Levator Palpabrae")],
                    [((400,224), (394, 224), "Orbicularis Oculi")]
                    ]
    
    nosePoints = [(267, 321), (292, 325), (320, 321)]
    noseVectors = [[((267, 321), (264, 310),"Levator Labii")],
                [((292, 325), (292, 324), "Levator Labii")],
                [((320, 321), (323, 310), "Levator Labii")]
                ]
    mouthPoints = [(238, 373), (265, 377), (293, 377),(320, 377), (347, 373),(320, 373), (293, 373),(265, 373),(238, 373)]
    mouthVectors = [ [((238, 373), (237, 362), "Levator Labii"), ((238, 373), (218, 357),"Zygomaticus"), ((238, 373), (224, 380),"Buccinator"), ((238, 373), (250, 373), "Orbicularis Oris"), ((238, 373), (239, 386), "Depressor Anguli Oris"), ((238, 373),(237, 410), "Jaw")],
                    [((265, 377), (258, 371),"Zygomaticus"), ((265, 377),(261, 380), "Buccinator"), ((265, 377), (277, 377), "Orbicularis Oris"), ((265, 377), (263, 383), "Depressor Anguli Oris"), ((265, 377),(265, 369),"Mentalis"), ((265, 377),(265, 426), "Jaw")],
                    [((293, 377), (293, 370), "Mentalis"), ((293, 377),(291, 426), "Jaw")],
                    [((320, 377), (327, 371), "Zygomaticus"), ((320, 377), (324, 380), "Buccinator"), ((320, 377), (308, 377), "Orbicularis Oris"), ((320, 377), (323, 383),"Depressor Anguli Oris"), ((320, 377),(318, 370), "Mentalis"), ((320, 377),(316, 426), "Jaw")],
                    [((347, 373), (347, 362), "Levator Labii"), ((347, 373), (369, 357), "Zygomaticus"), ((347, 373),(361, 380), "Buccinator"), ((347, 373), (335, 373), "Orbicularis Oris"), ((347, 373), (347, 386), "Depressor Anguli Oris"), ((347, 373),(344, 410), "Jaw")],
                    [((320, 373), (321, 348), "Levator Labii"), ((320, 373), (324, 376), "Buccinator"), ((320, 373), (308, 373), "Orbicularis Oris"), ((320, 373),(317, 358), "Mentalis"), ((320, 373),(319, 388), "Jaw")],
                    [((293, 373), (293, 351), "Levator Labii"), ((293, 373), (293, 360), "Mentalis")],
                    [((265, 373), (263, 348), "Levator Labii"), ((265, 373),(261, 376), "Buccinator"), ((265, 373),(277, 373), "Orbicularis Oris"), ((265, 373),(268, 358), "Mentalis"), ((265, 373),(263, 388),"Jaw")],
                    [((238, 373), (237, 362), "Levator Labii"), ((238, 373), (218, 357),"Zygomaticus"), ((238, 373), (224, 380),"Buccinator"), ((238, 373), (250, 373), "Orbicularis Oris"), ((238, 373), (239, 386), "Depressor Anguli Oris"), ((238, 373),(237, 410), "Jaw")],
                    ]
    chinPoints = [(282, 425), (292, 423),(303, 425)]
    chinVectors = [[((282, 425), (283, 412), "Mentalis"), ((282, 425), (281, 457), "Jaw")],
                [((292, 423), (293, 411), "Mentalis"), ((292, 423),(292, 454), "Jaw")],
                [((303, 425), (302, 412), "Mentalis"), ((303, 425),(301, 457), "Jaw")]
                ]
    #[(x,y), w, h is rect beginning at (x,y) with width w and height h
    leftPupilPoints = [(210, 212), 20, 20]
    rightPupilPoints = [(360, 212), 20, 20]
    leftIrisPoints = [(217, 219), 5, 5]
    rightIrisPoints = [(367, 219), 5, 5]
    #[(x,y), w, h, d, dt, t  is white rect beginning at (x,y) with width w and height h 
    # for blinking: direction d, and time to hold eyes open dt, and timer which decreases
    width = 22
    height = 1
    direction = 1
    #amount of time for each blink is going to be chosen randomly using tuple (mintime, maxtime)
    blinkTimeRange = (1,20) #if changing blinkTimeRange need to also change blinktime in faceAnimation.py 
    timer = random.randint(blinkTimeRange[0], blinkTimeRange[1])
    leftEyelidPoints = [(209, 207), width, height, direction, blinkTimeRange, timer]
    rightEyelidPoints = [(359, 207), width, height, direction, blinkTimeRange, timer]

    eyebrowLeft = FaceFeature(eyebrowLeftPoints, eyebrowLeftVectors, "eyebrowLeft")
    eyebrowRight = FaceFeature(eyebrowRightPoints, eyebrowRightVectors, "eyebrowRight")
    rightEye = FaceFeature(rightEyePoints,rightEyeVectors, "rightEye")
    leftEye = FaceFeature(leftEyePoints,leftEyeVectors, "leftEye")
    nose = FaceFeature(nosePoints, noseVectors, "nose")
    mouth = FaceFeature(mouthPoints, mouthVectors, "mouth")
    chin = FaceFeature(chinPoints, chinVectors, "chin")
    leftPupil = FaceFeature(leftPupilPoints, [], "leftPupil")
    rightPupil = FaceFeature(rightPupilPoints, [], "rightPupil")
    rightIris = FaceFeature(rightIrisPoints, [], "rightIris")
    leftIris = FaceFeature(leftIrisPoints, [], "leftIris")
    rightEyelid = FaceFeature(rightEyelidPoints, [], "rightEyelid")
    leftEyelid= FaceFeature(leftEyelidPoints, [], "leftEyelid")
    
    faceFeatures = [eyebrowLeft, eyebrowRight, rightEye, leftEye, nose, mouth, chin, leftPupil, rightPupil, leftIris, rightIris, rightEyelid, leftEyelid]
    face = FaceAnimation(faceFeatures)
    window.addWidgetToLayout(face)
    window.setCentralWidget(window.widget)
    window.show()
    
    
    app.exec_()

if __name__ == '__main__':
    rospy.init_node('python_face', anonymous=True)
    rate = rospy.Rate(10)
    main()


