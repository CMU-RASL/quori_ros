#!/usr/bin/env python3
import numpy as np
from scipy import signal
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt
import multiprocessing
import rospy
from std_msgs.msg import Float64MultiArray, String
from datetime import datetime
from pytz import timezone
import time
from config import *

class ExerciseEval:

    def __init__(self, replay, feedback_controller):
        self.replay = replay
        self.flag = False

        if not self.replay:
            #Initialize the subscribers
            self.pose_sub = rospy.Subscriber("/joint_angles", Float64MultiArray, self.pose_callback, queue_size=3000)

        self.feedback_controller = feedback_controller
        self.angles = []
        self.performance = []
        self.peaks = []
        self.feedback = []
        self.times = []
        self.current_exercise = ''
        self.exercise_name_list = []

    def start_new_set(self, exercise_name, set_num):
        self.angles.append(np.empty((0, len(ANGLE_INFO)*3)))
        self.performance.append(np.empty((0, len(EXERCISE_INFO[exercise_name]['comparison_joints']))))
        self.peaks.append([])
        self.feedback.append([])
        self.times.append([])
        self.current_exercise = exercise_name
        self.exercise_name_list.append(exercise_name)

        self.feedback_controller.logger.info('=====================================')
        self.feedback_controller.logger.info('STARTING SET {} OF {}'.format(set_num, exercise_name))
        self.feedback_controller.logger.info('=====================================')

    def find_peaks(self, index_to_search):
        peaks = []
        grads = []
        angles = self.angles[-1][index_to_search,:]
        grads = np.zeros_like(angles)
        
        for joint_ind in range(angles.shape[1]):
            grads[:, joint_ind] = np.gradient(angles[:,joint_ind])
            if joint_ind in EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']:
                peaks.append(signal.find_peaks(grads[:, joint_ind], height=1.5, distance=20, prominence=0.5)[0].astype('int'))
        peaks = np.sort(np.concatenate(peaks))
        return peaks, grads

    def check_if_new_peak(self, grads, peak_candidate, index_to_search):
        #Peaks in absolute indices
        #Grads, peak_candidates in relative units
        
        if (len(self.peaks[-1]) == 0 and index_to_search[peak_candidate] > 15) or (len(self.peaks[-1]) > 0 and self.peaks[-1][-1] + 15 < index_to_search[peak_candidate]):
            
            range_to_check = np.arange(np.max([peak_candidate-4, 0]), np.min([peak_candidate+4, grads.shape[0]])).astype('int')
            
            max_val = np.argmax(grads[range_to_check,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']],axis=0)
            max_val = np.mean(max_val).astype('int')

            grad_min = -5
            grad_max = 5

            actual_grad_max = np.max(grads[range_to_check][:, EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']])
            actual_grad_min = np.min(grads[range_to_check][:, EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']])
            
            actual_max = np.max(self.angles[-1][range_to_check][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']])
            actual_min = np.min(self.angles[-1][range_to_check][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']])

            actual_max_loc = np.where(self.angles[-1][range_to_check][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']] == actual_max)[0][0]
            actual_min_loc = np.where(self.angles[-1][range_to_check][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']] == actual_min)[0][0]

            if (actual_grad_max > grad_max \
                or actual_grad_min < grad_min) \
                    and actual_max_loc > actual_min_loc:
                
                if actual_min < 50 and actual_max > 100:
                    # print('added')
                    return range_to_check[max_val]
            
        return False

    def calc_dist_worker(self, ind, series1, series2, q):
            q.put((ind, fastdtw(series1, series2, dist=euclidean)))
            # q.put((ind, (np.random.random()*100 + 100, [])))

    def calc_dist(self, start, stop, indices):

        input_values = [EXPERTS[self.current_exercise]['experts'][ii][:,indices] for ii in range(len(EXPERTS[self.current_exercise]['experts']))]
        qout = multiprocessing.Queue()

        processes = [multiprocessing.Process(target=self.calc_dist_worker, args=(ind, self.angles[-1][start:stop, indices], val, qout))
                for ind, val in enumerate(input_values)]
        
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
            
            expert_distances = self.calc_dist(start, stop, indices_to_include)

            closest_expert = np.argmin(expert_distances)
            best_distance = np.min(expert_distances)
            expert_label = EXPERTS[self.current_exercise]['labels'][closest_expert]


            if best_distance < EXERCISE_INFO[self.current_exercise]['threshold1']:
                if expert_label == 'Good':
                    eval_list.append(1)
                else:
                    eval_list.append(-1)
                correction = expert_label
            elif best_distance < EXERCISE_INFO[self.current_exercise]['threshold2']:
                if expert_label == 'Good':
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
        
        if rep_duration < np.mean(EXPERTS[self.current_exercise]['duration']) - 3:
            speed = 'fast'
        elif rep_duration > np.mean(EXPERTS[self.current_exercise]['duration']) + 3 and rep_duration < 6:
            speed = 'slow'
        else:
            speed = 'good'
        
        self.feedback_controller.logger.info('Actual Duration {}, Average Expert Duration {}'.format(rep_duration, np.mean(EXPERTS[self.current_exercise]['duration'])))

        feedback = {'speed': speed, 'correction': corrections, 'evaluation': eval_list}

        self.feedback[-1].append(feedback)
        self.performance[-1] = np.vstack((self.performance[-1], feedback['evaluation']))
        
        self.feedback_controller.logger.info('Rep {}: Feedback {}'.format(len(self.feedback[-1]), feedback))
        self.feedback_controller.react(self.feedback[-1], self.current_exercise)

        return feedback

    def pose_callback(self, angle_message):

        if len(self.angles) == 0 or len(self.peaks) == 0:
            return

        if not self.flag:
            
            if self.angles[-1].shape[0] > 10 and len(self.peaks[-1]) > 0 and self.peaks[-1][-1] + 20 < self.angles[-1].shape[0]:
                #Add last point
                self.peaks[-1].append(self.angles[-1].shape[0]-1)

                #Evaluate rep
                current_rep = self.angles[-1][self.peaks[-1][-2]:self.peaks[-1][-1],:]
                rep_duration = (self.times[-1][self.peaks[-1][-2]] - self.times[-1][self.peaks[-1][-1]]).total_seconds()
                self.evaluate_rep(self.peaks[-1][-2], self.peaks[-1][-1], rep_duration)

            return

        #Read angle from message
        angle = angle_message.data
        self.angles[-1] = np.vstack((self.angles[-1], np.array(angle)))
        self.feedback_controller.logger.info('Angle {}'.format(len(self.angles[-1])))

        #Get time
        time = datetime.now(timezone('EST'))
        self.times[-1].append(time)

        if len(self.angles[-1]) % 2 == 0 or len(self.angles[-1]) < 200 or len(self.angles[-1]) > 1820+200:
            return

        #Look for new peaks
        if self.angles[-1].shape[0] > 70:
            
            #If far enough away from previous peak
            if len(self.peaks[-1]) == 0 or (self.peaks[-1][-1] + 70 < self.angles[-1].shape[0]):

                #Check if new rep

                if (self.current_exercise == 'bicep_curls' and np.max(self.angles[-1][-20:,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']]) < 40) or \
                (self.current_exercise == 'lateral_raises' and np.max(self.angles[-1][-20:,:][:,EXERCISE_INFO[self.current_exercise]['segmenting_joint_inds']]) < 35) :

                    #Add peak
                    self.peaks[-1].append(self.angles[-1].shape[0]-1)

                    self.feedback_controller.logger.info('Current peak - {}'.format(self.peaks[-1][-1]))

                    #Evaluate new rep
                    if len(self.peaks[-1]) > 1:
                        rep_duration = (self.times[-1][self.peaks[-1][-1]] - self.times[-1][self.peaks[-1][-2]]).total_seconds()
                        self.evaluate_rep(self.peaks[-1][-2], self.peaks[-1][-1], rep_duration)

        # if self.angles[-1].shape[0] % 10 == 1 and self.angles[-1].shape[0] > 15:

        #     index_to_search = np.arange(np.max([0,self.angles[-1].shape[0]-200]), self.angles[-1].shape[0]).astype('int')
            
        #     peak_candidates, grads  = self.find_peaks(index_to_search)
            
        #     #Get actual list of peaks
        #     for peak_candidate in peak_candidates:
        #         res = self.check_if_new_peak(grads, peak_candidate, index_to_search)
        #         if res:
        #             self.peaks[-1].append(index_to_search[res])
        #             self.feedback_controller.logger.info('Current peak - {}'.format(self.peaks[-1][-1]))

        #             #Evaluate new rep
        #             if len(self.peaks[-1]) > 1:
        #                 rep_duration = (self.times[-1][self.peaks[-1][-1]] - self.times[-1][self.peaks[-1][-2]]).total_seconds()
        #                 self.evaluate_rep(self.peaks[-1][-2], self.peaks[-1][-1], rep_duration)

    def plot_angles(self):
        
        fig, ax = plt.subplots(4, 3)
        ii = 0
        for row in range(4):
            for col in range(3):
                ax[row, col].plot(self.angles[-1][:,ii])
                for peak in self.peaks[-1]:
                    ax[row, col].plot(peak, self.angles[-1][peak,ii], 'ok')
                ii += 1
        
        plt.show()