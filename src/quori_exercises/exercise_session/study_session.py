#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from config import *
from ExerciseController import ExerciseController
from datetime import datetime
from pytz import timezone
import logging
import time
import pickle

#Parameters
MIN_LENGTH = 20
MAX_LENGTH = 30
MAX_REPS = 10
REST_TIME = 20
EXERCISE_LIST = ['bicep_curls', 'lateral_raises']
ROBOT_STYLE = 3

#Change at beginning of study
PARTICIPANT_ID = '1'
RESTING_HR = 75
AGE = 22
MAX_HR = 220-AGE

VERBAL_CADENCE = 2 #1 is low, 2 is medium, 3 is high
NONVERBAL_CADENCE = 2

#Initialize ROS node
rospy.init_node('study_session', anonymous=True)
rate = rospy.Rate(10)

#Start log file
log_filename = '{}.log'.format(datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
data_filename = '{}.pickle'.format(datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))

#Initialize evaluation object
controller = ExerciseController(False, log_filename, ROBOT_STYLE, RESTING_HR, MAX_HR)

#For each exercise
for set_num, exercise_name in enumerate(EXERCISE_LIST):
            
    #Start a new set
    controller.start_new_set(exercise_name, set_num+1, len(EXERCISE_LIST))

    inittime = datetime.now(timezone('EST'))
    controller.logger.info('-------------------Recording!')
    start_message = False

    #Lower arm all the way down
    controller.move_right_arm('halfway', 'sides')

    #Stop between minimum and maximum time and minimum reps
    while (datetime.now(timezone('EST')) - inittime).total_seconds() < MAX_LENGTH:        
                
        #Robot says starting set
        if not start_message:
            robot_message = "Start %s now" % (exercise_name.replace("_", " " ))
            controller.message(robot_message)
            start_message = True

        controller.flag = True

        #If number of reps is greater than 8 and they have been exercising at least the minimum length
        if len(controller.peaks[-1])-1 > MAX_REPS and (datetime.now(timezone('EST')) - inittime).total_seconds() > MIN_LENGTH:
            break 

    controller.flag = False
    controller.logger.info('-------------------Done with exercise')

    robot_message = "Almost done."
    controller.message(robot_message)
    rospy.sleep(3)

    robot_message = "Rest."
    controller.message(robot_message)
    controller.change_expression('smile', controller.start_set_smile, 4)

    rest_start = datetime.now(timezone('EST'))

    #Raise arm all the way up
    controller.move_right_arm('sides', 'up')

    if set_num + 1 < len(EXERCISE_LIST):
        halfway_message = False
        while (datetime.now(timezone('EST')) - rest_start).total_seconds() < REST_TIME:
            
            #Print halfway done with rest here
            if (datetime.now(timezone('EST')) - rest_start).total_seconds() > REST_TIME/2 and not halfway_message:
                halfway_message = True
                robot_message = "Rest for {} more seconds.".format(int(REST_TIME/2))
                controller.message(robot_message)
    else:
        robot_message = "Exercise session complete"
        controller.message(robot_message)
    
controller.plot_angles()

data = {'angles': controller.angles, 'peaks': controller.peaks, 'feedback': controller.feedback, 'times': controller.times, 'exercise_names': controller.exercise_name_list, 'all_hr': controller.all_heart_rates, 'heart_rates': controller.heart_rates, 'hrr': controller.hrr}
dbfile = open('src/quori_exercises/saved_data/{}'.format(data_filename), 'ab')

pickle.dump(data, dbfile)                    
dbfile.close()

controller.logger.info('Saved file {}'.format(data_filename))

controller.logger.handlers.clear()
logging.shutdown()
print('Done!')

plt.show()
