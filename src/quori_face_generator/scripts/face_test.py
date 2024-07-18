#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

neutral_emotion = [0, 0, 0, 0, 0, 0.05]
happy_emotion = [1, 0, 0, 0, 0, 0]
sad_emotion = [0, 1, 0, 0, 0, 0]
angry_emotion = [0, 0, 1, 0, 0, 0]
disgust_emotion = [0, 0, 0, 1, 0, 0]
fear_emotion = [0, 0, 0, 0, 1, 0]
surprise_emotion = [0, 0, 0, 0, 0, 0.2]

def talker():
    emotion_pub = rospy.Publisher('quori/face_generator_emotion', Float64MultiArray, queue_size=10)
    rospy.init_node('face_test', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz

    e1 = [0, 0, 0, 0, 0, 0.2]
    e2 = [0, 0, 0, 0, 0, 0.05]
    for ii in range(15):
        if ii % 2 == 0:
            emotion = e1
        else:
            emotion = e2
        emotion_to_send = Float64MultiArray()
        emotion_to_send.data = emotion
        emotion_pub.publish(emotion_to_send)
        print('Sent', emotion)
        rospy.sleep(1)

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass






