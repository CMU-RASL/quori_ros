#!/usr/bin/env python3
import matplotlib.pyplot as plt
from datetime import datetime
from intake_messages import *
import logging
import terminal_input as ti
import os
from pynput import keyboard
import rospy
from std_msgs.msg import String

#Change at the beginning of each session
PARTICIPANT_ID = '1'

def on_press(key):
    try:
        logger2.info(f'Key {key.char} pressed')
        return
        
    except AttributeError:
        logger2.info(f'Special key {key} pressed')
        return
# def on_release(key):
#     logger.info(f'Key {key} released')
#     if key == keyboard.Key.esc:
#         # Stop listener
#         return False

def get_message():
    key=ti.get_key()
    logger.info('Key, {}'.format(key))

    key_specific=ti.get_terminal_input(key)
    if key_specific=="back":
        logger.info('Key Specific, {}'.format(key_specific))
        return get_message()
    logger.info('Key Specific, {}'.format(key_specific))
    #logger.info('Message, {}'.format(INTAKE_MESSAGES[key][key_specific]))

    return INTAKE_MESSAGES[key][key_specific]

rospy.init_node('intake_session', anonymous=True)

sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)



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


# with keyboard.Listener(
#         on_press=on_press,
#         on_release=on_release) as listener:
#     listener.join()
listener = keyboard.Listener(on_press=on_press)
listener.start()


while not done_intake:
    message=get_message()
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
        
    elif speak=="back":
        #done_intake=True
        #logger.info('Quit')
        continue
    
    cont=input("Quit or Continue?")
    if cont=="quit":
        done_intake=True
        logger.info('Quit')
    elif cont=="":
        logger.info('Continue')
        continue
   

listener.stop()
logger.handlers.clear()
logging.shutdown()
print('Done!')



