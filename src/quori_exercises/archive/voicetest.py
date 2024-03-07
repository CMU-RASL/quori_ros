#voice generation test 
#!/usr/bin/env python3
import os
import rospy
from std_msgs.msg import String
from gtts import gTTS
import pygame
from io import BytesIO
import intake_messages as im

def say(text):
    tts = gTTS(text=text, lang='en')
    fp = BytesIO()
    tts.write_to_fp(fp)
    fp.seek(0)
    pygame.mixer.init()
    pygame.mixer.music.load(fp)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)

def callback(data):
    say(data.data)

def listener():
    rospy.init_node('quori_sound', anonymous=True)

    rospy.Subscriber("quori_sound", String, callback)

    rospy.spin()



if __name__ == '__main__':
    pygame.init()
    text=im.INTAKE_MESSAGES["Introduction"]["Greeting"]
    say(text)
    