#!/usr/bin/env python3
import logging
# import terminal_input as ti
import os
from pynput import keyboard
import rospy
from std_msgs.msg import String, Int32
import numpy as np
from datetime import datetime

#Change at the beginning of each session
PARTICIPANT_ID = '4'

rospy.init_node('wizard_speech', anonymous=True)

sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)

folder_path = 'src/quori_exercises/wizard_logs/' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path)

log_filename= 'Participant_{}_Wizard_{}.log'.format(PARTICIPANT_ID, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
log_fname = os.path.join(folder_path, log_filename) 

logger = logging.getLogger('logging')
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler(log_fname)
fh.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
logger.addHandler(fh)   

logger.info('Begin, {}'.format('Starting Wizard of Oz'))

done_flag = False

while not done_flag:
    speak=input("Speak:    ")
    if speak=="q":
        done_flag=True
        logger.info('Quit')
    else:
        logger.info('Quori said: {}'.format(speak))
        m = str(speak)
        sound_pub.publish(m)