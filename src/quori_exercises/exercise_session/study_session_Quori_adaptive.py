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
SET_LENGTH = 20
REST_TIME = 40
EXERCISE_LIST = ['bicep_curls'] #comment out before actual sessions
# EXERCISE_LIST = ['bicep_curls', 'bicep_curls', 'lateral_raises', 'lateral_raises']

#Change at beginning of study - make sure to change in adaptive_controller.py as well
PARTICIPANT_ID = '0'
RESTING_HR = 97
AGE = 26

#Change at beginning of each round
ROBOT_STYLE = 5 #1 is firm, 3 is encouraging, 5 is adaptive
ROUND_NUM = 1 #0 is intro

MAX_HR = 220-AGE

VERBAL_CADENCE = 2 #1 is low, 2 is medium, 3 is high
NONVERBAL_CADENCE = 2


#Initialize ROS node
rospy.init_node('study_session', anonymous=True)
rate = rospy.Rate(10)
set_pub = rospy.Publisher("set_performance", Int32, queue_size=10)

#Start log file
log_filename = 'Participant_{}_Style_{}_Round_{}_{}.log'.format(PARTICIPANT_ID, ROBOT_STYLE, ROUND_NUM, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))
data_filename = 'Participant_{}_Style_{}_Round_{}_{}.pickle'.format(PARTICIPANT_ID, ROBOT_STYLE, ROUND_NUM, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))

#Initialize evaluation object
controller = ExerciseController(False, log_filename, ROBOT_STYLE, RESTING_HR, MAX_HR, PARTICIPANT_ID)
rospy.sleep(2)

rospy.sleep(4)

intake_heart_rates = []
def intake_heart_rate_callback(msg):
    intake_heart_rates.append(msg.data)
    

if ROUND_NUM == 0:
    heart_rate_sub = rospy.Subscriber("/heart_rate", Int32, intake_heart_rate_callback, queue_size=3000)

    controller.message('Welcome and thank you for taking the time to be part of this research study. My name is Quori and I will be your exercise coach today.')

    input('Press Enter to continue...')

    controller.message('Before we start, please look through the consent form on the laptop next to you and write your name as a signature. Please also choose one of the three options for the optional permissions question. If you have any questions, please ask the researcher, and let me know when you are done.')

    input('Press Enter to continue...')

    controller.message('Great, thank you for signing. Next, can you please fill out the demographic survey on the laptop next to you. Let me know when you are done.')

    input('Press Enter to continue...')

    controller.message('Thank you for filling out the survey. Just to make sure I understood, can you please tell me your age?')

    input('Press Enter to continue...')

    controller.message('Thank you. I will now explain the exercise session we will do today. You will do three rounds with 4 sets each. The first two sets of each round will be bicep curls and the second two sets will be lateral raises. Each set will be 45 seconds. I will guide you through the exercises and give you feedback. You can take a look at the laptop screen for how to perform the two exercises, and please let the researcher know if you have any questions.')

    input('Press Enter to continue...')

    controller.message('Great, I have one more question to ask before you begin. I want to be the best coach I can be for you. Can you read the question on the laptop and choose one of the three options about the types of feedback you prefer. Please let me know when you are done.')

    input('Press Enter to continue...')

    controller.message('Thank you for answering the question. We will start the first round of exercise now. As a reminder, you will be starting with bicep curls and I will tell you when to start and stop. You can pick up the dumbbells if you want to use them. There are images of the two exercises on the sheet taped to your left for your reference. Please stand in the blue square when you are ready to start.')

    controller.logger.info('Resting Heart Rate Computed: {}'.format(np.mean(intake_heart_rates)))

if ROUND_NUM > 0:
    input("Press Enter to to start exercise session...")
    controller.message('Let us start Round {} now. Please stand in the blue square and pick up the dumbbells if you want to use them'.format(ROUND_NUM))
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

        rest_start = datetime.now(timezone('EST'))

        robot_message = "Time to rest."
        controller.message(robot_message)
        controller.change_expression('smile', controller.start_set_smile, 4)

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
            robot_message = "Round complete. Please take a seat in the chair and complete a survey about this round on the laptop next to you."
            controller.message(robot_message)

    controller.change_expression('smile', controller.start_set_smile, 4)

    if controller.robot_style == 5:
        controller.process.stdin.write('exit\n')
        controller.process.stdin.flush()

        if controller.process.stdin:
            controller.process.stdin.close()
        if controller.process.stdout:
            controller.process.stdout.close()
        # controller.process.wait()

    data = {'angles': controller.angles, 'peaks': controller.peaks, 'feedback': controller.feedback, 'times': controller.times, 'exercise_names': controller.exercise_name_list, 'all_hr': controller.all_heart_rates, 'heart_rates': controller.heart_rates, 'hrr': controller.hrr, 'actions': controller.actions, 'context': controller.contexts, 'rewards': controller.rewards}
    
    dbfile = open('/home/roshni/quori_files/quori_ros/src/quori_exercises/saved_data/{}'.format(data_filename), 'ab')

    pickle.dump(data, dbfile)                    
    dbfile.close()

    controller.logger.info('Saved file {}'.format(data_filename))

controller.logger.handlers.clear()
logging.shutdown()
print('Done!')

controller.plot_angles()

plt.show()