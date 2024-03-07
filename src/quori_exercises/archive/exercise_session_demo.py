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
ROUND_REST_TIME = 80
NUM_ROUNDS = 1 

#Change at beginning of study
PARTICIPANT_ID = '1'

#Change between each round
ROBOT_NUM = 3

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

def live_session():

    #Start log file
    log_filename = 'Participant_{}_Robot_{}.log'.format(PARTICIPANT_ID, ROBOT_NUM)

    #Initialize the feedback controller
    feedback_controller = FeedbackController(False, log_filename, ROBOT_NUM)

    #Initialize evaluation object
    exercise_eval = ExerciseEval(False, feedback_controller)
    exercise_eval.flag = False

    #For each exercise and set
    for round_num in range(1, NUM_ROUNDS+1):

        #New Round
        exercise_eval.feedback_controller.logger.info('=====================================')
        exercise_eval.feedback_controller.logger.info('STARTING ROUND {} OF {}'.format(round_num, 3))
        exercise_eval.feedback_controller.logger.info('=====================================')

        rospy.sleep(2)
        robot_message = "Round %s out of 3." % (round_num)
        exercise_eval.feedback_controller.message(robot_message)

        for exercise_name in ['bicep_curls', 'lateral_raises']:
            for set_num in range(1, NUM_SETS+1):
                if exercise_name == 'lateral_raises' and set_num == NUM_SETS:
                    is_final = True
                else:
                    is_final = False

                #Start a new set
                exercise_eval.start_new_set(exercise_name)
                feedback_controller.start_new_set()
                rospy.sleep(2)

                exercise_eval.feedback_controller.logger.info('=====================================')
                exercise_eval.feedback_controller.logger.info('STARTING SET {} OF {}'.format(set_num, exercise_name))
                exercise_eval.feedback_controller.logger.info('=====================================')

                #Raise arm all the way up
                feedback_controller.move_right_arm('up', 'halfway')

                #Robot says starting set and smile
                rospy.sleep(2)
                robot_message = "Get ready for set %s out of %s of %s" % (set_num, NUM_SETS, exercise_name.replace("_", " " ))

                exercise_eval.feedback_controller.message(robot_message)

                if ROBOT_NUM == 2:
                    feedback_controller.change_expression('smile', 0.6, 4)
                elif ROBOT_NUM == 3:
                    feedback_controller.change_expression('smile', 0.8, 4)
                rospy.sleep(2)

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
                    if len(exercise_eval.peaks[-1])-1 > 8 and (datetime.now(timezone('EST')) - inittime).total_seconds() > MIN_LENGTH:
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

                #Raise arm all the way up
                feedback_controller.move_right_arm('sides', 'up')

                if not is_final:
                    halfway_message = False
                    while (datetime.now(timezone('EST')) - rest_start).total_seconds() < REST_TIME:
                        
                        #Print halfway done with rest here
                        if (datetime.now(timezone('EST')) - rest_start).total_seconds() > REST_TIME/2 and not halfway_message:
                            halfway_message = True
                            robot_message = "Rest for {} more seconds.".format(int(REST_TIME/2))
                            exercise_eval.feedback_controller.message(robot_message)
                else:
                    robot_message = "End of Round {}".format(round_num)
                    exercise_eval.feedback_controller.message(robot_message)

                    halfway_message = False
                    while (datetime.now(timezone('EST')) - rest_start).total_seconds() < ROUND_REST_TIME:
                        
                        #Print halfway done with rest here
                        if (datetime.now(timezone('EST')) - rest_start).total_seconds() > ROUND_REST_TIME/2 and not halfway_message:
                            halfway_message = True
                            robot_message = "Rest for {} more seconds.".format(int(ROUND_REST_TIME/2))
                            exercise_eval.feedback_controller.message(robot_message)

    
    data_filename = 'Participant_{}_Robot_{}.npz'.format(PARTICIPANT_ID, ROBOT_NUM)
    np.savez('src/quori_exercises/saved_data/{}'.format(data_filename),      
                            angles=exercise_eval.angles,
                            peaks=exercise_eval.peaks,
                            feedback=exercise_eval.feedback,
                            times=exercise_eval.times,
                            exercise_names=exercise_eval.exercise_name_list
                        )
    exercise_eval.feedback_controller.logger.info('Saved file {}'.format(data_filename))

    exercise_eval.feedback_controller.logger.handlers.clear()
    logging.shutdown()
    print('Done!')

if __name__ == '__main__':
    
    #Initialize ROS node
    rospy.init_node('exercise_session', anonymous=True)

    rate = rospy.Rate(10)
  
    live_session()
