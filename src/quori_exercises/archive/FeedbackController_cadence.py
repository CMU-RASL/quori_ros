#!/usr/bin/env python3
import logging
import sys
from datetime import datetime, timedelta
from pytz import timezone
import numpy as np
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
import syllables
import time
from all_messages import *
from config import *

class FeedbackController:
    def __init__(self, replay, log_filename, robot_style, verbal_cadence, nonverbal_cadence):
        self.flag = False
        self.replay = replay
        self.logger = logging.getLogger()
        formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')
        stdout_handler = logging.StreamHandler(sys.stdout)
        stdout_handler.setLevel(logging.DEBUG)
        stdout_handler.setFormatter(formatter)
        self.logger.addHandler(stdout_handler)

        if not self.replay:
            #Initialize the publishers/subscribers
            self.sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)
            self.movement_pub = rospy.Publisher('quori/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
            self.emotion_pub = rospy.Publisher('quori/face_generator_emotion', Float64MultiArray, queue_size=10)
        
            #Add file handler
            file_handler = logging.FileHandler('src/quori_exercises/saved_logs/{}'.format(log_filename))
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)
        
        self.message_log = []
        self.message_time_stamps = []
        self.eval_case_log = []
        self.speed_case_log = []
        self.robot_style = int(robot_style)
        self.verbal_cadence = verbal_cadence
        self.nonverbal_cadence = nonverbal_cadence
        self.intercept = 0.6477586140350873
        self.slope = 0.31077594
        
        if robot_style == 1:
            self.neutral_expression = [0.1, 0, 0, 0, 0, 0]
            self.neutral_posture = [0, -1.1, 0, -1.1, 0]
        elif robot_style == 2:
            self.neutral_expression = [0.2, 0, 0, 0, 0, 0]
            self.neutral_posture = [-0.2, -1.1, 0, -1.1, 0.2]
        elif robot_style == 3:
            self.neutral_expression = [0.3, 0, 0, 0, 0, 0]
            self.neutral_posture = [0.2, -1.1, 0, -1.1, -0.2]

    def start_new_set(self):
        self.eval_case_log.append([])
        self.speed_case_log.append([])

    def message(self, m, priority=2):
        #Only message if it has been 3 sec since last message ended
        # if (len(self.message_time_stamps)) > 0:
            # last_message_time = self.message_time_stamps[-1]
            # if (datetime.now(timezone('EST')) - last_message_time).total_seconds() < 3.5 and priority < 2:
            #     #Skip message
            #     self.logger.info('Skipping {}'.format(m))
            #     return
                
        self.logger.info('Robot says: {}'.format(m))
        print('Robot says: {}'.format(m))
        length_estimate = np.round(self.slope*syllables.estimate(m) + self.intercept)
        if not self.replay:
            self.sound_pub.publish(m)
        self.message_log.append(m)
        self.message_time_stamps.append(datetime.now(timezone('EST')) + timedelta(seconds=length_estimate) )

    def get_bad_eval_cases(self, feedback):
        c = []

        num_bad_in_row = 4 - self.verbal_cadence
        
        if len(feedback) >= num_bad_in_row:

            bad_joint_groups = {'low_range': [], 'high_range': [], 'bad': []}
            for joint_group, _ in EXERCISE_INFO[self.current_exercise]['comparison_joints']:
                for message in ['low_range', 'high_range', 'bad']:
                    to_check = []
                    for ii in range(-num_bad_in_row, 0):
                        to_check.append(feedback[ii]['correction'][joint_group])
                    count = [1 for tmp in to_check if message in tmp]
                    count = np.sum(count)
                    if count == num_bad_in_row:
                        bad_joint_groups[message].append(joint_group) 
            
            for message, bad_joints in bad_joint_groups.items():
                if len(bad_joints) > 0:
                    left_count = np.sum([1 for j in bad_joints if 'left' in j ])
                    right_count = np.sum([1 for j in bad_joints if 'right' in j ])
                    
                    if right_count == 0:
                        c.append('{} left side'.format(message))
                    elif left_count == 0:
                        c.append('{} right side'.format(message))
                    else:
                        c.append('{} both sides'.format(message))

            if len(c) > 0:
                for message in ['low_range', 'high_range', 'bad']:
                    if len(self.eval_case_log[-1]) > 1:
                        for e in self.eval_case_log[-1][-1]:
                            if message in e:
                                c = []

        return c

    def get_correction_eval_cases(self, feedback):
        c = []

        if np.min(feedback[-1]['evaluation']) < 0:
            return c

        num_bad_in_row = 4 - self.verbal_cadence
        
        if len(feedback) >= num_bad_in_row + 1:

            bad_joint_groups = {'low_range': [], 'high_range': [], 'bad': []}
            for joint_group, _ in EXERCISE_INFO[self.current_exercise]['comparison_joints']:
                for message in ['low_range', 'high_range', 'bad']:
                    to_check = []
                    for ii in range(-num_bad_in_row-1, -1):
                        to_check.append(feedback[ii]['correction'][joint_group])
                    count = [1 for tmp in to_check if message in tmp]
                    count = np.sum(count)
                    if count == num_bad_in_row:
                        bad_joint_groups[message].append(joint_group) 
            
            for message, bad_joints in bad_joint_groups.items():
                if len(bad_joints) > 0:
                    left_count = np.sum([1 for j in bad_joints if 'left' in j ])
                    right_count = np.sum([1 for j in bad_joints if 'right' in j ])
                    
                    if right_count == 0:
                        c.append('corrected {} left side'.format(message))
                    elif left_count == 0:
                        c.append('corrected {} right side'.format(message))
                    else:
                        c.append('corrected {} both sides'.format(message))

        return c

    def get_good_eval_cases(self, feedback):
        c = []

        num_good_in_row = 5 - self.verbal_cadence

        if len(feedback) >= num_good_in_row:
            for ii in range(-num_good_in_row, 0):
                if np.min(feedback[ii]['evaluation']) < 0:
                    return c
            
            c.append('good form')
            #last positive message
            
            if len(self.eval_case_log[-1]) >= num_good_in_row:
                for ii in range(-num_good_in_row, 0):
                    if 'good form' in self.eval_case_log[-1][ii]:
                        c = []

        return c

    def find_eval_case(self, feedback):
        c = []

        if self.verbal_cadence == 0:
            #no verbal feedback
            return c

        c.extend(self.get_bad_eval_cases(feedback))

        c.extend(self.get_correction_eval_cases(feedback))
        
        c.extend(self.get_good_eval_cases(feedback))
       
        return c

    def get_bad_speed_cases(self, feedback):
        c = []

        num_bad_in_row = 5 - self.verbal_cadence
        if len(feedback) >= num_bad_in_row:

            fast_count = 0
            slow_count = 0
            for ii in range(-num_bad_in_row, 0):
                if feedback[ii]['speed'] == 'fast':
                    fast_count += 1
                if feedback[ii]['speed'] == 'slow':
                    slow_count += 1
            
            if fast_count == num_bad_in_row:
                c.append('fast')
            
            if slow_count == num_bad_in_row:
                c.append('slow')

            if len(self.eval_case_log[-1]) >= num_bad_in_row:
                for message in ['fast', 'slow']:
                    if message in self.speed_case_log[-1][-1]:
                        c = []

        return c
    
    def get_correction_speed_cases(self, feedback):
        c = []

        if not feedback[-1]['speed'] == 'good':
            return c

        num_bad_in_row = 5 - self.verbal_cadence

        if len(feedback) >= num_bad_in_row + 1:

            fast_count = 0
            slow_count = 0
            for ii in range(-num_bad_in_row-1, -1):
                if feedback[ii]['speed'] == 'fast':
                    fast_count += 1
                if feedback[ii]['speed'] == 'slow':
                    slow_count += 1
            
            if fast_count == num_bad_in_row:
                c.append('corrected fast')
            
            if slow_count == num_bad_in_row:
                c.append('corrected slow')
        
        return c
    
    def get_good_speed_cases(self, feedback):
        c = []

        num_good_in_row = 5 - self.verbal_cadence

        if len(feedback) >= num_good_in_row:
            for ii in range(-num_good_in_row, 0):
                if not feedback[ii]['speed'] == 'good':
                    return c
            
            c.append('good speed')

            #last positive message
            if len(self.speed_case_log[-1]) >= num_good_in_row:
                for ii in range(num_good_in_row):
                    if 'good speed' in self.speed_case_log[-1][-ii]:
                        c = []

        return c

    def find_speed_case(self, feedback):
        c = []

        if self.verbal_cadence == 0:
            #no verbal feedback
            return c

        c.extend(self.get_bad_speed_cases(feedback))
        c.extend(self.get_correction_speed_cases(feedback))
        c.extend(self.get_good_speed_cases(feedback))
        
        return c

    def get_message(self, c, exercise_name):
        
        m = []
        message_to_case = []
        for ci in c:
            m_to_add = ALL_MESSAGES[exercise_name][ci][self.robot_style]
            m.extend(m_to_add)
            message_to_case.extend([ci]*len(m_to_add))

        if len(m) > 0:
            #Pick the option that has been chosen the least
            counts = []
            for option in m:
                if option in self.message_log:
                    counts.append(self.message_log.count(option))
                else:
                    counts.append(0)
            
            #Get the minimum count
            ind = np.argmin(counts)
            
            return message_to_case[ind], m[ind]

        return -1, ''

    def move_right_arm(self, start, end):

        #["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]
        arm_halfway = list(self.neutral_posture)
        arm_halfway[0] = 1.1
        
        arm_sides = list(self.neutral_posture)
        
        arm_up = list(self.neutral_posture)
        arm_up[0] = 1.7
        
        positions = {'halfway': arm_halfway, 'sides': arm_sides, 'up': arm_up}
        
        self.logger.info('Moving arm from {} to {}'.format(start, end))
        
        self.send_body(positions[start], positions[end], 4)

    def change_expression(self, expression, intensity, duration):
        #['joy', 'sadness', 'anger', 'disgust', 'fear', 'surprise']
        if expression == 'smile':
            if not self.replay:
                self.send_expression([intensity, 0, 0, 0, 0, 0], self.neutral_expression, duration)
            self.logger.info('Robot smiling at intensity {} for duration {}'.format(intensity, duration))
            print('Robot smiling at intensity {} for duration {}'.format(intensity, duration))
        elif expression == 'frown':
            if not self.replay:
                self.send_expression([0, intensity, 0, 0, 0, 0], self.neutral_expression, duration)
            self.logger.info('Robot frowning at intensity {} for duration {}'.format(intensity, duration))
            print('Robot frowning at intensity {} for duration {}'.format(intensity, duration))

    def send_expression(self, start_emotion, end_emotion, duration):
        emotion_to_send = Float64MultiArray()
        emotion_to_send.data = start_emotion
        self.emotion_pub.publish(emotion_to_send)

        time.sleep(duration/2)

        emotion_to_send = Float64MultiArray()
        emotion_to_send.data = end_emotion
        self.emotion_pub.publish(emotion_to_send)

    def send_body(self, start_position, end_position, duration):
        # self.logger.info('Moving from {} to {} for duration {}'.format(start_position, end_position, duration))
        if not self.replay:
            # print('Start: {}, End: {}, Duration {}'.format(start_position, end_position, duration))
            #Start point
            traj = JointTrajectory()
            traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]
            point_1 = JointTrajectoryPoint()
            point_1.time_from_start = rospy.Duration(duration / 2)
            point_1.positions = start_position
            traj.points=[point_1]
            self.movement_pub.publish(traj)

            time.sleep(duration/2)

            #End point
            traj = JointTrajectory()
            traj.joint_names = ["r_shoulder_pitch", "r_shoulder_roll", "l_shoulder_pitch", "l_shoulder_roll", "waist_pitch"]

            point_2 = JointTrajectoryPoint()
            point_2.time_from_start = rospy.Duration(duration / 2)
            point_2.positions = end_position
            traj.points=[point_2]
            self.movement_pub.publish(traj)
    
    def react_nonverbal(self, value):

        if value == 'neutral':
            a = -0.1
            b = 0.1
            start_position = (self.neutral_posture + (b-a) * np.random.random_sample((5,)) + a).tolist()
            end_position = (self.neutral_posture + (b-a) * np.random.random_sample((5,)) + a).tolist()
            self.send_body(start_position, end_position, 2)

        elif value == 'positive':
            
            if self.robot_style == 2:
                #Robot 2 - moves less
                end_arm = [0, -0.8]
                torso = 0.21*0.25
                
            elif self.robot_style == 3:
                #Robot 3 - moves more
                end_arm = [0, -0.8]
                torso = 0.21*0.4
                
            start_position = [end_arm[0], end_arm[1], end_arm[0], end_arm[1], torso]
            self.send_body(start_position, self.neutral_posture, 4)

            if self.robot_style == 2:
                self.change_expression('smile', 0.6, 4)
            elif self.robot_style == 3:
                self.change_expression('smile', 0.9, 4)

        elif value == 'negative':

            if self.robot_style == 2:
                #Robot 2 - moves less
                torso = -0.47*0.25
                
            elif self.robot_style == 3:
                #Robot 3 - moves more
                torso = -0.47*0.4

            start_position = self.neutral_posture
            start_position[-1] = torso
            self.send_body(start_position, self.neutral_posture, 4)

            if self.robot_style == 2:
                self.change_expression('frown', 0.5, 4)
            elif self.robot_style == 3:
                self.change_expression('frown', 0.0, 4)
    
    def nonverbal_case(self, feedback, c):
        if c == '':
            if np.min(feedback[-1]['evaluation']) >= 0:
                if self.nonverbal_cadence == 3:
                    self.react_nonverbal('positive')
                elif self.nonverbal_cadence == 2 and np.random.random_sample() < 0.5:
                    self.react_nonverbal('positive')
                elif self.nonverbal_cadence == 1 and np.random.random_sample() < 0.25:
                    self.react_nonverbal('positive')
                else:
                    self.react_nonverbal('neutral')
            else:
                if self.nonverbal_cadence == 3:
                    self.react_nonverbal('negative')
                elif self.nonverbal_cadence == 2 and np.random.random_sample() < 0.5:
                    self.react_nonverbal('negative')
                elif self.nonverbal_cadence == 1 and np.random.random_sample() < 0.25:
                    self.react_nonverbal('negative')
                else:
                    self.react_nonverbal('neutral')        
        else:
            if 'good' in c or 'corrected' in c:
                self.react_nonverbal('positive')
            else:
                self.react_nonverbal('negative')

    def react(self, feedback, exercise_name): 
        
        self.current_exercise = exercise_name

        eval_case = self.find_eval_case(feedback)
        self.eval_case_log[-1].append(eval_case)

        speed_case = self.find_speed_case(feedback)
        self.speed_case_log[-1].append(speed_case)

        #Get message for each case
        eval_chosen_case, eval_message = self.get_message(eval_case, exercise_name)
        speed_chosen_case, speed_message = self.get_message(speed_case, exercise_name)

        self.logger.info('Evaluation case {} with chosen {} and message - {}'.format(eval_case, eval_chosen_case, eval_message))
        self.logger.info('Speed case {} with chosen {} and message - {}'.format(speed_case, speed_chosen_case, speed_message))
        # print('Evaluation case {} with chosen {} and message - {}'.format(eval_case, eval_chosen_case, eval_message))
        # print('Speed case {} with chosen {} and message - {}'.format(speed_case, speed_chosen_case, speed_message))
        
        #If both messages available, choose the eval message
        if len(eval_message) > 0:
            chosen_case = eval_chosen_case
            self.message(eval_message, priority=1)
        elif len(speed_message) > 0:
            chosen_case = speed_chosen_case
            self.message(speed_message, priority=1)
        
        #If feedback case, want to match the reaction, otherwise go for the last feedback
        if speed_message == '' and eval_message == '':
            self.nonverbal_case(feedback, '')
        else:
            self.nonverbal_case(feedback, chosen_case)