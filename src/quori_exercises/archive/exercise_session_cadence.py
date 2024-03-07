#!/usr/bin/env python3
import rospy
from pytz import timezone
from datetime import datetime
import numpy as np

import logging

from ExerciseEval import ExerciseEval
from FeedbackController import FeedbackController

#Parameters
VERBAL_CADENCE = 1 #1 is low, 2 is medium, 3 is high
NONVERBAL_CADENCE = 1
ROBOT_STYLE = 3

def session():

    #Start log file
    log_filename = 'Style_{}_Verbal_{}_Nonverbal_{}.log'.format(ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)

    #Initialize the feedback controller
    feedback_controller = FeedbackController(False, log_filename, ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)

    #Initialize evaluation object
    exercise_eval = ExerciseEval(False, feedback_controller)
    exercise_eval.flag = False

    #For each exercise and set
    for round_num in range(1, NUM_ROUNDS+1):

        for exercise_name in ['bicep_curls']:
            for set_num in range(1, NUM_SETS+1):

                #Start a new set
                exercise_eval.start_new_set(exercise_name)
                feedback_controller.start_new_set()
                # rospy.sleep(2)

                exercise_eval.feedback_controller.logger.info('=====================================')
                exercise_eval.feedback_controller.logger.info('STARTING SET {} OF {}'.format(set_num, exercise_name))
                exercise_eval.feedback_controller.logger.info('=====================================')

                #Raise arm all the way up
                # feedback_controller.move_right_arm('up', 'halfway')

                #Robot says starting set and smile
                # rospy.sleep(2)
                robot_message = "Get ready for set %s out of %s of %s" % (set_num, NUM_SETS, exercise_name.replace("_", " " ))

                # exercise_eval.feedback_controller.message(robot_message)

                # if ROBOT_STYLE == 2:
                #     feedback_controller.change_expression('smile', 0.6, 4)
                # elif ROBOT_STYLE == 3:
                #     feedback_controller.change_expression('smile', 0.8, 4)
                # rospy.sleep(2)

                inittime = datetime.now(timezone('EST'))
                exercise_eval.feedback_controller.logger.info('-------------------Recording!')
                start_message = False

                #Lower arm all the way down
                # feedback_controller.move_right_arm('halfway', 'sides')

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
                    if len(exercise_eval.peaks[-1])-1 > MIN_REPS and (datetime.now(timezone('EST')) - inittime).total_seconds() > MIN_LENGTH:
                        break 

                exercise_eval.flag = False
                exercise_eval.feedback_controller.logger.info('-------------------Done with exercise')

                robot_message = "Almost done."
                exercise_eval.feedback_controller.message(robot_message)
                rospy.sleep(3)

                robot_message = "Set Completed."
                exercise_eval.feedback_controller.message(robot_message)
                if ROBOT_STYLE == 2:
                    feedback_controller.change_expression('smile', 0.6, 4)
                elif ROBOT_STYLE == 3:
                    feedback_controller.change_expression('smile', 0.8, 4)

                rest_start = datetime.now(timezone('EST'))

                #Raise arm all the way up
                feedback_controller.move_right_arm('sides', 'up')

    # for proc in psutil.process_iter():
    #     if "record" in proc.name() and set(command[2:]).issubset(proc.cmdline()):
    #         proc.send_signal(subprocess.signal.SIGINT)

    # proc.send_signal(subprocess.signal.SIGINT)

    data_filename = 'Style_{}_Verbal_{}_Nonverbal_{}.npz'.format(ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)
    
    exercise_eval.plot_angles()

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
    
    session(mode=['play'])
