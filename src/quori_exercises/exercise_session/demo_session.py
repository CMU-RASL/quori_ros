#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from config_computer import *
from ExerciseController_computer import ExerciseController
from datetime import datetime
from pytz import timezone
import logging
import time
import pickle
from std_msgs.msg import Int32
from pynput import keyboard

#Parameters
SET_LENGTH = 45
# EXERCISE_LIST = ['bicep_curls'] #comment out before actual sessions
EXERCISE_LIST = ['bicep_curls']

#Change at beginning of study
PARTICIPANT_ID = '0'
RESTING_HR = 68
AGE = 67

#Change at beginning of each round
ROBOT_STYLE = 3 #1 is firm, 3 is encouraging, 5 is adaptive
ROUND_NUM = 1

MAX_HR = 220-AGE

VERBAL_CADENCE = 2 #1 is low, 2 is medium, 3 is high
NONVERBAL_CADENCE = 2

#Initialize ROS node
rospy.init_node('demo_session', anonymous=True)
rate = rospy.Rate(10)
set_pub = rospy.Publisher("set_performance", Int32, queue_size=10)

#Start log file
log_filename = 'Participant_{}_Style_{}_Round_{}_{}.log'.format(PARTICIPANT_ID, ROBOT_STYLE, ROUND_NUM, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
data_filename = 'Participant_{}_Style_{}_Round_{}_{}.pickle'.format(PARTICIPANT_ID, ROBOT_STYLE, ROUND_NUM, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))

#Initialize evaluation object
controller = ExerciseController(False, log_filename, ROBOT_STYLE, RESTING_HR, MAX_HR)
rospy.sleep(2)

rospy.sleep(4)

input("Press Enter to to start exercise session...")

#For each exercise
for set_num, exercise_name in enumerate(EXERCISE_LIST):
            
    #Start a new set
    controller.start_new_set(exercise_name, set_num+1, len(EXERCISE_LIST))
    
    controller.logger.info('-------------------Recording!')
    start_message = False
    halfway_message = False

    #Lower arm all the way down
    controller.move_right_arm('halfway', 'sides')
    
    inittime = datetime.now(timezone('EST'))
    
    #Stop between minimum and maximum time and minimum reps
    while (datetime.now(timezone('EST')) - inittime).total_seconds() < SET_LENGTH:        
                
        #Robot says starting set
        if not start_message:
            robot_message = "Start %s now" % (exercise_name.replace("_", " " ))
            controller.message(robot_message)
            start_message = True

        controller.flag = True

        if (datetime.now(timezone('EST')) - inittime).total_seconds() > SET_LENGTH/2 and not halfway_message:
            robot_message = "You are halfway"
            controller.message(robot_message)
            halfway_message = True 

        if (datetime.now(timezone('EST')) - inittime).total_seconds() > SET_LENGTH:
            break 

    controller.flag = False
    controller.logger.info('-------------------Done with exercise')

    robot_message = "Almost done."
    controller.message(robot_message)
    rospy.sleep(3)

    # #Calculate set-level performance
    # rep_performance = []
    # for f in controller.feedback[-1]:
    #     if np.min(f['evaluation']) > 0:
    #         rep_performance.append(1)
    #     else:
    #         rep_performance.append(0)

    # if len(rep_performance) == 0:
    #     set_performance = 0
    #     set_performance_explanation = 'Medium'
    # elif np.mean(rep_performance) > 0.6:
    #     set_performance = 1
    #     set_performance_explanation = 'Excellent'
    # else:
    #     set_performance = 0
    #     set_performance_explanation = 'Medium'
    
    # set_pub.publish(set_performance)
    # controller.logger.info('|||||Rep performance: {}, Set performance: {}'.format(rep_performance, set_performance_explanation))

    robot_message = "Rest."
    controller.message(robot_message)
    controller.change_expression('smile', controller.start_set_smile, 4)

    #Raise arm all the way up
    controller.move_right_arm('sides', 'up')

    def on_press(key):
        try:
            if key == keyboard.Key.enter:
                return False
        except AttributeError:
            pass
    
    with keyboard.Listener(on_press=on_press) as listener:
        for _ in range(100):
            set_pub.publish(set_performance)
            print('Press enter to start next set, Publishing set performance {}'.format(set_performance))
            time.sleep(1)
            if not listener.running:
                break

# controller.message('You are all done with exercises today. Great job!')
controller.change_expression('smile', controller.start_set_smile, 4)

if controller.robot_style == 5:
    controller.process.stdin.write('exit\n')
    controller.process.stdin.flush()

    if controller.process.stdin:
        controller.process.stdin.close()
    if controller.process.stdout:
        controller.process.stdout.close()
    controller.process.wait()

data = {'angles': controller.angles, 'peaks': controller.peaks, 'feedback': controller.feedback, 'times': controller.times, 'exercise_names': controller.exercise_name_list, 'all_hr': controller.all_heart_rates, 'heart_rates': controller.heart_rates, 'hrr': controller.hrr, 'actions': controller.actions, 'context': controller.context, 'rewards': controller.rewards}

dbfile = open('/home/roshni/quori_files/quori_ros/src/quori_exercises/saved_data/{}'.format(data_filename), 'ab')

pickle.dump(data, dbfile)                    
dbfile.close()

controller.logger.info('Saved file {}'.format(data_filename))

controller.logger.handlers.clear()
logging.shutdown()
print('Done!')

# plt.show()