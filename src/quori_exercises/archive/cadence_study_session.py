#!/usr/bin/env python3
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from config import *
from FeedbackController_cadence import FeedbackController
from datetime import datetime
from pytz import timezone
import logging
import time

#Parameters
current_exercise = 'bicep_curls'
bag_name = 'Reid_bicep_curls.bag'
VERBAL_CADENCE = 2 #1 is low, 2 is medium, 3 is high
NONVERBAL_CADENCE = 2
ROBOT_STYLE = 3

evaluation = [
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'low_range right_shoulder', 'left_shoulder': 'low_range left_shoulder', 'right_elbow': 'low_range right_elbow', 'left_elbow': 'low_range left_elbow'}, 'evaluation': [-1, -1, -1, -1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'low_range right_shoulder', 'left_shoulder': 'low_range left_shoulder', 'right_elbow': 'low_range right_elbow', 'left_elbow': 'low_range left_elbow'}, 'evaluation': [-1, -1, -1, -1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'low_range right_shoulder', 'left_shoulder': 'low_range left_shoulder', 'right_elbow': 'low_range right_elbow', 'left_elbow': 'low_range left_elbow'}, 'evaluation': [-1, -1, -1, -1]},
    {'speed': 'slow', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'slow', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'slow', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]},
    {'speed': 'good', 'correction': {'right_shoulder': 'Good right_shoulder', 'left_shoulder': 'Good left_shoulder', 'right_elbow': 'Good right_elbow', 'left_elbow': 'Good left_elbow'}, 'evaluation': [1, 1, 1, 1]}
]

#Start log file
log_filename = 'Style_{}_Verbal_{}_Nonverbal_{}.log'.format(ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)

#Initialize the feedback controller
feedback_controller = FeedbackController(True, log_filename, ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)

feedback_controller.start_new_set()

#Raise arm all the way up
feedback_controller.move_right_arm('up', 'halfway')

#Robot says starting set and smile
robot_message = "Get ready for %s" % (exercise_name.replace("_", " " ))

feedback_controller.message(robot_message)

if ROBOT_STYLE == 2:
    feedback_controller.change_expression('smile', 0.6, 4)
elif ROBOT_STYLE == 3:
    feedback_controller.change_expression('smile', 0.8, 4)
rospy.sleep(2)

feedback_controller.logger.info('-------------------Recording!')

#Lower arm all the way down
feedback_controller.move_right_arm('halfway', 'sides')

robot_message = "Start %s now" % (exercise_name.replace("_", " " ))
feedback_controller.message(robot_message)


bag = rosbag.Bag(bag_name)

all_angles = np.empty((0, 12))
peaks = []
feedback = []
performance = np.empty((0, len(EXERCISE_INFO[exercise_name]['comparison_joints'])))
times = []

said_almost_done = False

for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/joint_angles'])):
    if idx == 0:
        times.append(t)

    times.append(t)

    duration_since_previous = times[-1] - times[-2]
    
    start_time = datetime.now(timezone('EST'))

    if len(peaks) == 13 and not said_almost_done:
        robot_message = "Almost done."
        feedback_controller.message(robot_message)
        said_almost_done = True

    #Skip the empty points
    if (bag_name == 'Reid_bicep_curls.bag' and idx > 200 and idx < 1820+200) or \
        (bag_name == 'Reid_bicep_curls2.bag' and idx > 225 and idx < 1720+225) or \
        (bag_name == 'Reid_lateral_raises.bag' and idx > 440 and idx < 2440) or \
        (bag_name == 'Reid_lateral_raises2.bag' and idx > 220 and idx < 2270):

        #Save angle
        all_angles = np.vstack((all_angles, np.array(msg.data)))

        #If enough has past from the beginning
        if all_angles.shape[0] > 70:
            
            #If far enough away from previous peak
            if len(peaks) == 0 or (peaks[-1] + 70 < all_angles.shape[0]):
                
                #Check if new rep

                if (current_exercise == 'bicep_curls' and np.max(all_angles[-20:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]) < 40) or \
                (current_exercise == 'lateral_raises' and np.max(all_angles[-20:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]) < 35) :
                    
                    #Add peak
                    peaks.append(all_angles.shape[0]-1)

                    #Get feedback
                    feedback.append(evaluation[len(peaks)-1])
                    performance = np.vstack((performance, evaluation[len(peaks)-1]['evaluation']))

                    #React
                    feedback_controller.logger.info('Feedback {}'.format(evaluation[len(peaks)-1]))
                    print('---')
                    print('Rep {} - Feedback {}'.format(len(peaks), feedback[-1]))
                    feedback_controller.react(feedback, current_exercise)

        # while (datetime.now(timezone('EST')) - start_time).total_seconds() <  duration_since_previous.to_sec():
        #     time.sleep(0.001)

bag.close()

robot_message = "Set Completed."
feedback_controller.message(robot_message)
if ROBOT_STYLE == 2:
    feedback_controller.change_expression('smile', 0.6, 4)
elif ROBOT_STYLE == 3:
    feedback_controller.change_expression('smile', 0.8, 4)


fig, ax = plt.subplots(4, 3)
ii = 0
for row in range(4):
    for col in range(3):
        ax[row, col].plot(all_angles[:,ii])
        for peak in peaks:
            ax[row, col].plot(peak, all_angles[peak,ii], 'ok')
        ii += 1

print(peaks)
print('Num reps', len(peaks))

fig, ax = plt.subplots()
ii = 11
ax.plot(all_angles[:,ii])
ax.plot(np.gradient(all_angles[:,ii]))
for peak in peaks:
    ax.plot(peak, all_angles[peak, ii], 'ok')


data_filename = 'Style_{}_Verbal_{}_Nonverbal_{}.npz'.format(ROBOT_STYLE, VERBAL_CADENCE, NONVERBAL_CADENCE)
np.savez('src/quori_exercises/saved_data/{}'.format(data_filename),      
                            angles=all_angles,
                            peaks=peaks,
                            feedback=feedback
                        )
feedback_controller.logger.info('Saved file {}'.format(data_filename))

feedback_controller.logger.handlers.clear()
logging.shutdown()
print('Done!')


plt.show()
