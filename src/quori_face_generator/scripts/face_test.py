#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

neutral_emotion = [0.2, 0, 0, 0, 0, 0]
happy_emotion = [1, 0, 0, 0, 0, 0]
sad_emotion = [0, 1, 0, 0, 0, 0]
angry_emotion = [0, 0, 1, 0, 0, 0]
disgust_emotion = [0, 0, 0, 1, 0, 0]
fear_emotion = [0, 0, 0, 0, 1, 0]
surprise_emotion = [0, 0, 0, 0, 0, 1]

def talker():
    emotion_pub = rospy.Publisher('quori/face_generator_emotion', Float64MultiArray, queue_size=10)
    rospy.init_node('face_test', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz


    for emotion in [sad_emotion, neutral_emotion, happy_emotion, neutral_emotion, sad_emotion, neutral_emotion]:

        emotion_to_send = Float64MultiArray()
        emotion_to_send.data = emotion
        emotion_pub.publish(emotion_to_send)
        print('Sent', emotion)
        rospy.sleep(8)

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass






