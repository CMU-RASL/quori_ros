import numpy as np

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtCore import Qt

#the vector class, which can be seen as the "muscles" that pull on the face
#points p1 and p2 should be tuples of floats and muscleClass should be of type muscleClass
class Vector():
    def __init__(self, p1, p2, muscleGroup):
        self.p1 = p1
        self.p2 = p2
        self.muscleGroup = muscleGroup
        self.unitVector = self.createUnitVectorFromTwoPoints(self.p1,self.p2)
        self.maxLength = self.getVectorLength()
        self.x = self.createVectorFromTwoPoints(p1, p2)[0]
        self.y = self.createVectorFromTwoPoints(p1, p2)[1]
    
    #percent should be a decimal ie 50% as 0.5
    def updateVectorUsingPercentage(self, percent):
        self.p2 = (self.p1[0] + self.unitVector[0] * self.maxLength * percent,
               self.p1[1] + self.unitVector[1] * self.maxLength * percent)
        self.x = self.createVectorFromTwoPoints(self.p1, self.p2)[0]
        self.y = self.createVectorFromTwoPoints(self.p1, self.p2)[1]


    def getVectorLength(self):
        p1 = self.p1
        p2 = self.p2
        distance = (((p2[0] - p1[0]) ** 2)+ ((p2[1] - p1[1]) ** 2))**0.5
        return distance
    
    
    def createUnitVectorFromTwoPoints(self, p1, p2):
        x1 = p1[0]
        x2 = p2[0]
        y1 = p1[1]
        y2 = p2[1]

        # Define two points as NumPy arrays
        point1 = np.array([x1, y1])
        point2 = np.array([x2, y2])

        # Calculate the vector from point1 to point2
        vector = point2 - point1

        # Calculate the magnitude of the vector
        magnitude = np.linalg.norm(vector)

        # Create the unit vector by dividing the vector by its magnitude
        if magnitude != 0:
            unitVector = vector / magnitude
        else:
            unitVector = np.array([0.0, 0.0])
        return unitVector
    
    def createVectorFromTwoPoints(self, p1, p2):
        x1 = p1[0]
        x2 = p2[0]
        y1 = p1[1]
        y2 = p2[1]

        # Define two points as NumPy arrays
        point1 = np.array([x1, y1])
        point2 = np.array([x2, y2])

        # Calculate the vector from point1 to point2
        vector = point2 - point1
        return vector
