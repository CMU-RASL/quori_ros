import numpy as np
import random
import sys
import math
import os
from PyQt5.QtWidgets import *
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from vectors import Vector
from faceFeature import FaceFeature
from emotions import Emotion

class FaceAnimation(QWidget):
    #faceFeatures is a list of faceFeatures
    #points come in the form of a 2D list containing tuples (x,y)
    #vectors are a list of lists, where each list corresponds to a specific facial feature (i.e. eyebrow, mouth)
    #each list of vectors at index i should correspond to the vectors that pull on the same point at index i in points
    #vectors are in the form (p1, p2, muscleGroup)
    def __init__(self, faceFeatures):
        super().__init__()
        self.faceFeatures = faceFeatures
        self.points = []
        vectors = []
        for faceFeature in self.faceFeatures:
            if faceFeature.name not in ["leftPupil", "rightPupil", "leftIris", "rightIris", "rightEyelid", "leftEyelid"]:
                self.points.append(faceFeature.points)
                vectors.append(faceFeature.vectors)
                
        self.vectors = [[[] for j in range(len(self.points[i]))] for i in range(len(self.points))]
        #convert vectors to type Vector and store in self.vectors
        for faceFeatureIndex in range(len(vectors)):
            for point in range(len(vectors[faceFeatureIndex])):
                for v in range(len(vectors[faceFeatureIndex][point])):
                    vector = vectors[faceFeatureIndex][point][v]
                    newVector = Vector(vector[0], vector[1], vector[2])
                    self.vectors[faceFeatureIndex][point].append(newVector)

        #convert points to QPointF
        for faceFeatureIndex in range(len(self.points)):
            for i in range(len(self.points[faceFeatureIndex])):
                point = self.points[faceFeatureIndex][i]
                self.points[faceFeatureIndex][i] = QPointF(point[0],point[1])

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        # Set the pen for drawing points
        #draw the eyes which are circles
        for faceFeature in self.faceFeatures:
            painter.setPen(QPen(Qt.black,  5))
            if faceFeature.name == "leftPupil" or faceFeature.name == "rightPupil" or faceFeature.name == "leftIris" or faceFeature.name == "rightIris":
                painter.drawEllipse (faceFeature.points[0][0], faceFeature.points[0][1],faceFeature.points[1], faceFeature.points[2])
            if faceFeature.name == "leftEyelid" or faceFeature.name == "rightEyelid":   
                #blinking
                painter.setPen(QPen(Qt.white,  5))
                painter.setBrush(QBrush(Qt.white, Qt.SolidPattern))
                painter.drawRect(faceFeature.points[0][0], faceFeature.points[0][1],faceFeature.points[1], faceFeature.points[2])
        #draw the other facial Features
        painter.setPen(QPen(Qt.black,  5))
        for faceFeatureIndex in range(len(self.points)):
            for i in range (len(self.points[faceFeatureIndex])-1):
                painter.drawLine(self.points[faceFeatureIndex][i], self.points[faceFeatureIndex][i+1])

    #when moving the face we are given an emotion, direction and speed
    #the emotion should be of the Emotion class
    def moveWidget(self, emotion, direction, speed, percentage):
        #modify the pulling percentages of each muscle pulling for emotion
        # print('movewidget', emotion.name)
        emotion.modifyEmotionValuesByPercent(percentage)
        activeMuscles = emotion.emotionDict.keys()
        for faceFeatureIndex in range(len(self.points)):
            faceFeaturePoints = self.points[faceFeatureIndex]
            faceFeatureVectors = self.vectors[faceFeatureIndex]
            for i in range(len(faceFeaturePoints)):
                currPoint = faceFeaturePoints[i]
                pullingVectors = []
                for vector in faceFeatureVectors[i]:
                    if vector.muscleGroup in activeMuscles:
                        #update the vector being pulled on based on percentages from emotion
                        vector.updateVectorUsingPercentage(emotion.getMusclePercent(vector.muscleGroup))
                        pullingVectors.append(vector)
                if len(pullingVectors) != 0:
                    vector = self.createNetForce(pullingVectors)
                    #print((self.isPointNotAtTargetLocation(i, currPoint, vector, direction)))
                    if (self.isPointNotAtTargetLocation(i, currPoint, vector, direction)):
                        # Calculate the new position
                        newPoint = self.pullOnPoint(currPoint, vector, speed, direction)
                        self.points[faceFeatureIndex][i] = newPoint

        #blinking animation - rectangle that expands
        blinktime = random.randint(120, 140)
        for faceFeature in self.faceFeatures:
            if faceFeature.name in ["rightEyelid", "leftEyelid"]:
                height = faceFeature.points[2]
                if height >= 23:
                    faceFeature.points[3] = -1 #dir becomes -1 and we are opening eyes
                if height <= 1:
                    if faceFeature.points[5] != 0: #if timer for holding eyes open has not run out
                        faceFeature.points[3] = 0 #dir becomes 0
                        faceFeature.points[5] -= 1 #timer decreases
                    if faceFeature.points[5] == 0: # timer for holding eyes open has run out
                        faceFeature.points[3] = 1 #dir becomes 1 and we are closing eyes
                        #change timer back to random blinktime
                        faceFeature.points[5] = blinktime
                #change the height of rectangle
                faceFeature.points[2] = height + faceFeature.points[3]*3


    
    #given a point, change the points location using a vector
    def pullOnPoint(self, point, vector, speed, direction):
        #print("pulling on point!")
        unitVector = vector.unitVector
        targetPos = None
        if direction == 1:
            targetPos = np.array(vector.p2).astype(float)
        else:
            targetPos = np.array(vector.p1).astype(float)
        # Define a point's initial position
        point = np.array([point.x(), point.y()])
        # Define a force vector that represents the pull
        force = np.array(unitVector * speed * direction)  # Adjust the values as needed

        # Update the point's location by applying the force
        newPosition = point + force
        #return the new position
        # if True in newPosition > targetPos :
        #     return QPointF(targetPos[0], targetPos[1])
        # if True in newPosition < targetPos :
        #     return QPointF(targetPos[0], targetPos[1])
        # else:
        return QPointF(newPosition[0], newPosition[1])


    #if there are multiple forces pulling on one point
    #takes in list of forces aka unit vectors
    def createNetForce(self, forces):
        if len(forces) == 1: 
            return forces[0]
        netForce = np.array([0.0, 0.0])
        for f in forces:
            netForce += np.array([f.x, f.y])
        p2 = (forces[0].p1[0] + netForce[0],forces[0].p1[1] + netForce[1])
        netVector = Vector(forces[0].p1, p2, "combined")
        return netVector 

    def isPointNotAtTargetLocation(self, i, point, vector, direction):
        currPos = np.array([point.x(), point.y()])
        threshold = 1e-2  # Adjust the threshold as needed

        if direction == 1:
            targetPos = np.array(vector.p2).astype(float)
            distanceTraveled = np.linalg.norm(currPos - np.array(vector.p1).astype(float))
            #print(f"Direction 1 - Distance Traveled: {distanceTraveled}, Vector Length: {vector.getVectorLength(1)}, point: {i}")
            if np.all(np.abs(currPos - targetPos) <= threshold) or distanceTraveled >= vector.getVectorLength():
                return False
            return True

        if direction == -1:
            # targetPos = np.array(vector.p2).astype(float)
            # distanceTraveled = (((currPos[0] - vector.p2[0]) ** 2)+ ((currPos[1] - vector.p2[1]) ** 2))**0.5
            #print(distanceTraveled)
            #np.linalg.norm(np.array(vector.p2).astype(float) - currPos)
            # print(f"Direction -1 - Distance Traveled: {distanceTraveled}, Vector Length: {vector.getVectorLength()}, point: {i}")
            # print(distanceTraveled, vector.getVectorLength())


            # curpos - p1 == p2 - p1
            distanceTraveled = (((currPos[0] - vector.p1[0]) ** 2)+ ((currPos[1] - vector.p1[1]) ** 2))**0.5
            targetDistance = (((vector.p2[0] - vector.p1[0]) ** 2)+ ((vector.p2[1] - vector.p1[1]) ** 2))**0.5

            if distanceTraveled >= targetDistance:
                return True
            else:
                return False

            # if np.all(np.abs(currPos - targetPos) <= threshold) or distanceTraveled <= vector.getVectorLength():
            #     return False
            # return True
