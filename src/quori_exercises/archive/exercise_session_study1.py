#!/usr/bin/env python3
import rospy
from pytz import timezone
from datetime import datetime
import numpy as np

import logging

from ExerciseEval import ExerciseEval
from FeedbackController import FeedbackController

#Fixed parameters
MIN_LENGTH = 30
MAX_LENGTH = 50
NUM_SETS = 2
REST_TIME = 40

#Change at beginning of study
PARTICIPANT_ID = '24'

#Change between each round
ROBOT_NUM = 2

ROUND_NUM = 3

def replay(filename, re_eval):

    #Initialize the feedback controller
    feedback_controller = FeedbackController(True, '', ROBOT_NUM)

    data_file = np.load('src/quori_exercises/saved_data/{}'.format(filename), allow_pickle=True)

    exercise_eval = ExerciseEval(True, data_file['exercise_name'], feedback_controller)

    exercise_eval.angles = data_file['angles']
    exercise_eval.peaks=data_file['peaks']
    exercise_eval.feedback=data_file['feedback']
    exercise_eval.times=data_file['times']
    exercise_eval.good_experts = np.array([ii for ii, label in enumerate(exercise_eval.labels) if 'Good' in label]).astype(int)

    #Get joint groups
    exercise_eval.set_joint_groups()

    if re_eval:

        #Clear all the computed values
        exercise_eval.peaks=[]
        exercise_eval.feedback=[]

        #Get joint groups
        exercise_eval.set_joint_groups()
        exercise_eval.performance = np.empty((0, len(exercise_eval.joint_groups)))

        exercise_eval.reeval()

    exercise_eval.plot_results()

def live_session(exercise_name, set_num, is_final):
    #Start log file
    log_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.log'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, exercise_name, set_num)

    #Initialize the feedback controller
    feedback_controller = FeedbackController(False, log_filename, ROBOT_NUM)

    #Initialize evaluation object
    exercise_eval = ExerciseEval(False, EXERCISE_NAME, feedback_controller)

    #Starting set
    exercise_eval.feedback_controller.logger.info('=====================================')
    exercise_eval.feedback_controller.logger.info('STARTING SET {} OF {}'.format(set_num, exercise_name))
    exercise_eval.feedback_controller.logger.info('=====================================')

    #Raise arm all the way up
    feedback_controller.move_right_arm('up', 'halfway')

    #Robot says starting set and smile
    rospy.sleep(2)
    robot_message = "Get ready for set %s out of %s of %s" % (set_num,
                                                        NUM_SETS, exercise_name.replace("_", " " ))
    exercise_eval.feedback_controller.message(robot_message)
    if ROBOT_NUM == 2:
        feedback_controller.change_expression('smile', 0.6, 4)
    elif ROBOT_NUM == 3:
        feedback_controller.change_expression('smile', 0.8, 4)
    rospy.sleep(6)

    inittime = datetime.now(timezone('EST'))
    exercise_eval.feedback_controller.logger.info('-------------------Recording!')
    start_message = False

    #Lower arm all the way down
    feedback_controller.move_right_arm('halfway', 'sides')

    #Stop between minimum and maximum time and minimum reps
    while (datetime.now(timezone('EST')) - inittime).total_seconds() < MAX_LENGTH:        
    
        #Robot says starting set
        if not start_message:
            robot_message = "Start %s now" % (exercise_name.replace("_", " " ))
            exercise_eval.feedback_controller.message(robot_message)
            start_message = True

        exercise_eval.flag = True
        feedback_controller.flag = True

        #If number of reps is greater than 8 and they have been exercising at least the minimum length
        if len(exercise_eval.peaks)-1 > 8 and (datetime.now(timezone('EST')) - inittime).total_seconds() > MIN_LENGTH:
            break 

    exercise_eval.flag = False
    exercise_eval.feedback_controller.logger.info('-------------------Done with exercise')

    robot_message = "Almost done."
    exercise_eval.feedback_controller.message(robot_message)
    rospy.sleep(3)

    robot_message = "Rest."
    exercise_eval.feedback_controller.message(robot_message)
    if ROBOT_NUM == 2:
        feedback_controller.change_expression('smile', 0.6, 4)
    elif ROBOT_NUM == 3:
        feedback_controller.change_expression('smile', 0.8, 4)

    rest_start = datetime.now(timezone('EST'))

    robot_message = "Using the scale next to you, how difficult was that last set, from 1 to 10?"
    exercise_eval.feedback_controller.message(robot_message)

    #Raise arm all the way up
    feedback_controller.move_right_arm('sides', 'up')

    #Get summary statistics
    exercise_eval.feedback_controller.logger.info('Total Number of Reps {}'.format(len(exercise_eval.peaks)-1))

    exercise_eval.feedback_controller.logger.info('Total Angles {}'.format(exercise_eval.angles.shape[0]))

    exercise_eval.pose_sub.unregister()
    
    data_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.npz'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, exercise_name, set_num)
    np.savez('src/quori_exercises/saved_data/{}'.format(data_filename),      
                            angles=exercise_eval.angles,
                            peaks=exercise_eval.peaks,
                            feedback=exercise_eval.feedback,
                            times=exercise_eval.times,
                            exercise_name=exercise_name
                        )
    exercise_eval.feedback_controller.logger.info('Saved file {}'.format(data_filename))

    if not is_final:
        halfway_message = False
        while (datetime.now(timezone('EST')) - rest_start).total_seconds() < REST_TIME:
            
            #Print halfway done with rest here
            if (datetime.now(timezone('EST')) - rest_start).total_seconds() > REST_TIME/2 and not halfway_message:
                halfway_message = True
                robot_message = "Rest for {} more seconds.".format(int(REST_TIME/2))
                exercise_eval.feedback_controller.message(robot_message)
    else:
        rospy.sleep(8)
        robot_message = "Please walk over to the researcher to fill out a survey."
        exercise_eval.feedback_controller.message(robot_message)

    logging.shutdown()

if __name__ == '__main__':
    
    #Initialize ROS node
    rospy.init_node('exercise_session', anonymous=True)

    #Set flag for live or not
    replay_flag = False

    if replay_flag:
        #Set flags and variables for reaching from files
        re_eval = False

        PARTICIPANT_ID = '3'
        ROUND_NUM = 1
        ROBOT_NUM = 1
        SET_NUM = 1
        EXERCISE_NAME = 'bicep_curls'

        data_filename = 'Participant_{}_Round_{}_Robot_{}_Exercise_{}_Set_{}.npz'.format(PARTICIPANT_ID, ROUND_NUM, ROBOT_NUM, EXERCISE_NAME, SET_NUM)

        replay(data_filename, re_eval)
    else:

        rate = rospy.Rate(10)

        #For each exercise and set
        for EXERCISE_NAME in ['bicep_curls', 'lateral_raises']:
            for SET_NUM in range(1, NUM_SETS+1):
                if EXERCISE_NAME == 'lateral_raises' and SET_NUM == NUM_SETS:
                    is_final = True
                else:
                    is_final = False
                live_session(EXERCISE_NAME, SET_NUM, is_final)
