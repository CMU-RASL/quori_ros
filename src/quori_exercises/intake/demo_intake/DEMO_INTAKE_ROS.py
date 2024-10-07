#!/usr/bin/env python3
import matplotlib.pyplot as plt
from datetime import datetime
from demo_messages import *
import logging
import demo_input as ti
import os
from pynput import keyboard
import rospy
from std_msgs.msg import String, Int32
import numpy as np

#Change at the beginning of each session
PARTICIPANT_ID = '20' 

def on_press(key):
    try:
        logger2.info(f'Key {key.char} pressed')
        return
        
    except AttributeError:
        logger2.info(f'Special key {key} pressed')
        return


def get_message(name="Edward"):
    key=ti.get_key()
    logger.info('Key, {}'.format(key))


    key_specific=ti.get_terminal_input(key)
    if key_specific=="Greeting":
        message=DEMO_MESSAGES[key][key_specific][0].format(name=name)
        return message
    if key_specific=="back":
        logger.info('Key Specific, {}'.format(key_specific))
        return get_message()
    elif key_specific=="quit":
        logger.info('Quit')
        return "quit"
       
    logger.info('Key Specific, {}'.format(key_specific))

        
    return DEMO_MESSAGES[key][key_specific]


    

def heart_rate_callback(data):
    heart_rates.append(data.data)


rospy.init_node('intake_session', anonymous=True)

sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)

heart_rate_sub = rospy.Subscriber("/heart_rate", Int32, heart_rate_callback, queue_size=3000)
heart_rates = []


folder_path = 'src/quori_exercises/intake_logs/' 
if not os.path.exists(folder_path): 
    os.makedirs(folder_path) 

#Start log file
intake_log_filename= 'Participant_{}_Intake_{}.csv'.format(PARTICIPANT_ID, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
log_fname = os.path.join(folder_path, intake_log_filename)

keylog_filename= 'Participant_{}_keypress_{}.csv'.format(PARTICIPANT_ID, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
key_log_fname = os.path.join(folder_path, keylog_filename)


#Initialize logging
logger = logging.getLogger('logging')
logger2 = logging.getLogger('keypress')
logger.setLevel(logging.DEBUG)
logger2.setLevel(logging.DEBUG)
fh = logging.FileHandler(format(log_fname))
fh2 = logging.FileHandler(format(key_log_fname))
fh.setLevel(logging.DEBUG)
fh2.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter=logging.Formatter(
    fmt='%(asctime)s.%(msecs)03d,%(message)s',
    datefmt='%Y-%m-%d,%H:%M:%S'
)
#formatter = logging.Formatter('%(asctime)s,%(message)s')
fh.setFormatter(formatter)
fh2.setFormatter(formatter)
ch.setFormatter(formatter)
logger.addHandler(fh)
#logger.addHandler(ch)
logger2.addHandler(fh2)
#logger2.addHandler(ch)

             
done_intake = False

#accessing the terminal input 
logger.info('Begin, {}'.format('Starting intake'))
logger2.info('Begin, {}'.format('Starting intake'))
print("starting intake")



listener = keyboard.Listener(on_press=on_press)
listener.start()

#PARTICIPANT_NAME="Edward"
PARTICIPANT_NAME="Emily"
#PARTICIPANT_NAME="Edith"


while not done_intake:
    message=get_message(PARTICIPANT_NAME)
    if message=="quit":
        done_intake=True
        logger.info('Quit')
        break
    print(message)
    # print("here")
    speak=input("Press enter to speak, else type back to go back")
    if speak == "":
        #logger.info('Begin speaking')
        # print("Begin speaking")
        logger.info('Begin speaking,{}'.format(message))
        m = str(message)
        m = m.replace("'", "")
        m = m.replace("]", "")
        sound_pub.publish(m)
        # logger.info('End speaking')
        # print("End speaking")
        
    elif speak=="b":
        #done_intake=True
        #logger.info('Quit')
        continue
    
    cont=input("Quit or Continue?")
    if cont=="q":
        done_intake=True
        logger.info('Quit')
    elif cont=="":
        logger.info('Continue')
        continue
   

listener.stop()
logger.info('Resting HR, {}'.format(np.round(np.mean(heart_rates)), 1))
logger.handlers.clear()
logging.shutdown()
print('Done!')



