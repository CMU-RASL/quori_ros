#!/usr/bin/env python3
import numpy as np
from scipy import signal
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt
import multiprocessing
import rospy
from std_msgs.msg import Float64MultiArray, String
from datetime import datetime, timedelta
from pytz import timezone
import time
import logging
from config import *
import sys
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import syllables
from all_messages import *

class ExerciseController:

    def __init__(self, replay, log_filename, robot_style, verbal_cadence, nonverbal_cadence):

        #Set initial parameters
        self.replay = replay
        self.flag = False
        self.verbal_cadence = verbal_cadence
        self.nonverbal_cadence = nonverbal_cadence
        self.intercept = 0.6477586140350873
        self.slope = 0.31077594

        #Robot style 0 - very firm, 1 - firm, 2 - neutral, 3 - encouraging, 4 - very encouraging
        self.update_robot_style(robot_style)

        #Initialize subscribers and publishers if running in real-time
        if not self.replay:
            self.pose_sub = rospy.Subscriber("/joint_angles", Float64MultiArray, self.pose_callback, queue_size=3000)
            self.sound_pub = rospy.Publisher("quori_sound", String, queue_size=10)
            self.movement_pub = rospy.Publisher('quori/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
            self.emotion_pub = rospy.Publisher('quori/face_generator_emotion', Float64MultiArray, queue_size=10)

        #Create data storage
        self.angles = []
        self.performance = []
        self.peaks = []
        self.feedback = []
        self.times = []
        self.current_exercise = ''
        self.exercise_name_list = []
        self.message_log = []
        self.message_time_stamps = []
        self.eval_case_log = []
        self.speed_case_log = []

        #Initialize logging
        self.logger = logging.getLogger('logging')
        self.logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler('src/quori_exercises/saved_logs/{}'.format(log_filename))
        fh.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        self.logger.addHandler(fh)
        self.logger.addHandler(ch)

    def update_robot_style(self, robot_style):
        self.robot_style = robot_style
        self.neutral_expression = NEUTRAL_EXPRESSIONS[robot_style]
        self.neutral_posture = NEUTRAL_POSTURES[robot_style]
        self.start_set_smile = START_SET_SMILE[robot_style]

    def start_new_set(self, exercise_name, set_num):

        #Update data storage
        self.angles.append(np.empty((0, len(ANGLE_INFO)*3)))
        self.performance.append(np.empty((0, len(EXERCISE_INFO[exercise_name]['comparison_joints']))))
        self.peaks.append([])
        self.feedback.append([])
        self.times.append([])
        self.current_exercise = exercise_name
        self.exercise_name_list.append(exercise_name)
        self.eval_case_log.append([])
        self.speed_case_log.append([])

        #Raise arm all the way up
        self.move_right_arm('up', 'halfway')

        #Add starting set to log
        self.logger.info('=====================================')
        self.logger.info('STARTING SET {} OF {}'.format(set_num, exercise_name))
        self.logger.info('=====================================')

        #Robot says starting set and smile
        robot_message = "Get ready for set %s of %s" % (set_num, exercise_name.replace("_", " " ))
        self.message(robot_message)
        self.change_expression('smile', self.start_set_smile, 4)


    def calc_dist_worker(self, ind, series1, series2, q):
        q.put((ind, fastdtw(series1, series2, dist=euclidean)))

    def calc_dist(self, start, stop, indices, label):

        input_values = NEW_EXPERTS[self.current_exercise][label]

        # result = []
        # for x in input_values:
        #     series1 = self.angles[-1][start:stop, indices]
        #     series2 = x[:, 1:]
        #     series2 = series2[:, indices]
        #     output = fastdtw(series1, series2, dist=euclidean)
        #     result.append(output[0])

        # input_values = [EXPERTS[self.current_exercise]['experts'][ii][:,indices] for ii in range(len(EXPERTS[self.current_exercise]['experts']))]

        new_input_values = []
        for x in input_values:
            series2 = x[:, 1:]
            series2 = series2[:, indices]
            new_input_values.append(series2)

        qout = multiprocessing.Queue()

        processes = [multiprocessing.Process(target=self.calc_dist_worker, args=(ind, self.angles[-1][start:stop, indices], val, qout))
                for ind, val in enumerate(new_input_values)]
        
        for p in processes:
            p.start()

        for p in processes:
            p.join()

        unsorted_result = [qout.get() for p in processes]
        result = [t[1][0] for t in sorted(unsorted_result)]

        return result

    def evaluate_rep(self, start, stop, rep_duration):

        corrections = {}
        eval_list = []

        for joint_group, angles_to_include in EXERCISE_INFO[self.current_exercise]['comparison_joints']:
            indices = ANGLE_ORDER[joint_group]
            indices_to_include = []
            for ii in range(3):
                if ii in angles_to_include:
                    indices_to_include.append(indices[ii])
            
            distances = []
            labels = []
            for label in ['good', 'low range', 'high range']:
                result = self.calc_dist(start, stop, indices_to_include, label)
                distances.extend(result)
                labels.extend([label]*len(result))

            closest_expert = np.argmin(distances)
            best_distance = distances[closest_expert]
            expert_label = labels[closest_expert]

            if best_distance < EXERCISE_INFO[self.current_exercise]['threshold1']:
                if expert_label == 'good':
                    eval_list.append(1)
                else:
                    eval_list.append(-1)
                correction = expert_label

            elif best_distance < EXERCISE_INFO[self.current_exercise]['threshold2']:
                if expert_label == 'good':
                    correction = 'ok'
                    eval_list.append(0)
                else:
                    correction = 'bad'
                    eval_list.append(-1)
            else:
                correction = 'bad'
                eval_list.append(-1)
            
            correction += ' {}'.format(joint_group)
            corrections[joint_group] = correction

        if rep_duration < NEW_EXPERTS[self.current_exercise]['average duration'] - 2:
            speed = 'fast'
        elif rep_duration > NEW_EXPERTS[self.current_exercise]['average duration'] - 2 and rep_duration < 7:
            speed = 'slow'
        else:
            speed = 'good'
        
        self.logger.info('Actual Duration {}, Average Expert Duration {}'.format(rep_duration, NEW_EXPERTS[self.current_exercise]['average duration']))

        feedback = {'speed': speed, 'correction': corrections, 'evaluation': eval_list}

        self.feedback[-1].append(feedback)
        self.performance[-1] = np.vstack((self.performance[-1], feedback['evaluation']))
        
        self.logger.info('Rep {}: Feedback {}'.format(len(self.feedback[-1]), feedback))
        # self.react(self.feedback[-1], self.current_exercise)
        
        return feedback

    def pose_callback(self, angle_message):

        if len(self.angles) == 0 or len(self.peaks) == 0 or not self.flag:
            return

        #Read angle from message
        angle = angle_message.data
        self.angles[-1] = np.vstack((self.angles[-1], np.array(angle)))
        # self.logger.info('Angle {}'.format(len(self.angles[-1])))

        #Get time
        time = rospy.get_time()
        self.times[-1].append(time)

        #Look for new peaks
        if self.angles[-1].shape[0] > 70 and self.angles[-1].shape[0] % 15 == 0:
            
            #If far enough away from previous peak
            if len(self.peaks[-1]) == 0 or (self.peaks[-1][-1] + 40 < self.angles[-1].shape[0]):

                to_check_amount = 25

                #Check if new rep
                current_angles_min = np.min(self.angles[-1][-to_check_amount:,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']])
                current_angles_max = np.max(self.angles[-1][-to_check_amount:,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']])

                grad = []
                for index in EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']:
                    grad.append(np.max(np.gradient(self.angles[-1][-to_check_amount:,][:,index])))
                
                max_grad = np.max(grad)
                max_val_pos = np.argmax(self.angles[-1][-to_check_amount:,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']])

                peak_to_add = None
                print(self.angles[-1].shape[0]-to_check_amount+max_val_pos-1, 'angle max', current_angles_max, 'angle_min', current_angles_min, 'grad_max', max_grad)
                if self.current_exercise == 'bicep_curls':
                    if current_angles_max > 150 and max_grad > 3:

                        peak_candidate = self.angles[-1].shape[0]-to_check_amount+max_val_pos-1

                        peak_candidate = np.min([self.angles[-1].shape[0]-1, peak_candidate])
                        
                        print('peak candidate', peak_candidate)
                        if len(self.peaks[-1]) > 0:
                            print('min in range', np.min(self.angles[-1][self.peaks[-1][-1]:peak_candidate,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']]))

                        if len(self.peaks[-1]) == 0 or (self.peaks[-1][-1] + 40 < peak_candidate and np.min(self.angles[-1][self.peaks[-1][-1]:peak_candidate,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']]) < 100):
                            
                            print('min at peak', np.min(self.angles[-1][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']][peak_candidate,:]))
                            
                            if np.min(self.angles[-1][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']][peak_candidate,:]) > 100:
                                peak_to_add = peak_candidate

                elif self.current_exercise == 'lateral_raises':
                    if current_angles_max > 75 and max_grad > 6:
                        peak_candidate = self.angles[-1].shape[0]-to_check_amount+max_val_pos-1
                        print('pre peak candidate', self.angles[-1].shape[0], to_check_amount, max_val_pos, peak_candidate)
                        peak_candidate = np.min([self.angles[-1].shape[0]-1, peak_candidate])
                        
                        print('peak candidate', peak_candidate)
                        if len(self.peaks[-1]) > 0:
                            print('min in range', np.min(self.angles[-1][self.peaks[-1][-1]:peak_candidate,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']]))

                        if len(self.peaks[-1]) == 0 or (self.peaks[-1][-1] + 40 < peak_candidate):
                            
                            print('min at peak', np.min(self.angles[-1][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']][peak_candidate,:]))
                            if np.min(self.angles[-1][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']][peak_candidate,:]) > 55:
                                peak_to_add = peak_candidate

                if peak_to_add:
                    self.peaks[-1].append(peak_candidate)
                            
                    self.logger.info('Current peak - {}'.format(self.peaks[-1][-1]))

                    #Evaluate new rep
                    if len(self.peaks[-1]) > 1:
                        # print(len(self.times[-1]), self.peaks[-1])
                        rep_duration = (self.times[-1][self.peaks[-1][-1]] - self.times[-1][self.peaks[-1][-2]])
                        # self.evaluate_rep(self.peaks[-1][-2], self.peaks[-1][-1], rep_duration)
                        self.message('Rep')

    def plot_angles(self):
        fig, ax = plt.subplots(4, 3)
        fig2, ax2 = plt.subplots(4, 3)
        ii = 0
        for row in range(4):
            for col in range(3):
                ax[row, col].plot(self.angles[-1][:,ii])
                ax2[row, col].plot(np.gradient(self.angles[-1][:,ii]))
                for peak_num, (beg, end) in enumerate(zip(self.peaks[-1][:-1], self.peaks[-1][1:])):
                    ax[row, col].plot(beg, self.angles[-1][beg,ii], 'or', markersize=20)
                    ax[row, col].plot(end, self.angles[-1][end,ii], 'or', markersize=20)
                    # if np.min(self.feedback[-1][peak_num]['evaluation']) > 0:
                    #     color = 'g'
                    # else:
                    #     color = 'r'
                    # ax[row, col].plot(np.arange(beg, end), self.angles[-1][beg:end,ii], color)
                ii += 1
        
        plt.show()
    
    def message(self, m, priority=2):
        #Only message if it has been 3 sec since last message ended
        if (len(self.message_time_stamps)) > 0:
            last_message_time = self.message_time_stamps[-1]
            if (datetime.now(timezone('EST')) - last_message_time).total_seconds() < 3 and priority < 2:

                #Skip message
                self.logger.info('Skipping {}'.format(m))
                return
                
        self.logger.info('Robot says: {}'.format(m))
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

            #Temporary until LLM updates!
            if self.robot_style in [0, 1, 2]:
                m_to_add = ALL_MESSAGES[exercise_name][ci][2]
            elif self.robot_style in [3, 4]:
                m_to_add = ALL_MESSAGES[exercise_name][ci][3]

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

        elif expression == 'frown':
            if not self.replay:
                self.send_expression([0, intensity, 0, 0, 0, 0], self.neutral_expression, duration)
            self.logger.info('Robot frowning at intensity {} for duration {}'.format(intensity, duration))

    def send_expression(self, start_emotion, end_emotion, duration):
        emotion_to_send = Float64MultiArray()
        emotion_to_send.data = start_emotion
        self.emotion_pub.publish(emotion_to_send)

        time.sleep(duration/2)

        emotion_to_send = Float64MultiArray()
        emotion_to_send.data = end_emotion
        self.emotion_pub.publish(emotion_to_send)

    def send_body(self, start_position, end_position, duration):
        if not self.replay:

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

            end_arm = [0, -0.8]
            start_position = [end_arm[0], end_arm[1], end_arm[0], end_arm[1], NONVERBAL_REACT['positive']['torso'][self.robot_style]]
            self.send_body(start_position, self.neutral_posture, 4)

            self.change_expression('smile', NONVERBAL_REACT['positive']['expression'][self.robot_style], 4)

        elif value == 'negative':

            start_position = self.neutral_posture
            start_position[-1] = NONVERBAL_REACT['negative']['torso'][self.robot_style]
            self.send_body(start_position, self.neutral_posture, 4)

            self.change_expression('frown', NONVERBAL_REACT['negative']['expression'][self.robot_style], 4)
    
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