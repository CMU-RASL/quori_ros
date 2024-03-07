import numpy as np
import matplotlib.pyplot as plt
# import pandas as pd
from scipy import signal
# from fastdtw import fastdtw
# from scipy.spatial.distance import euclidean
# from datetime import datetime


def find_peaks(angles, segmenting_joints):
    grads = np.zeros_like(angles)
    peaks = []
    for joint_ind in range(angles.shape[1]):
        grads[:, joint_ind] = np.gradient(angles[:,joint_ind])
        if joint_ind in segmenting_joints:
            peaks.append(signal.find_peaks(grads[:, joint_ind], height=1.5, distance=15, prominence=0.5)[0].astype('int'))
    peaks = np.sort(np.concatenate(peaks))
    print('Peaks', peaks)
    return peaks, grads

def check_if_new_peak(angles, grads, peaks, peak_candidate, segmenting_joints):

    #Peaks in absolute indices
    #angles, Grads, peak_candidates in relative units
    if (len(peaks) == 0 and peak_candidate > 15) or (len(peaks) > 0 and peaks[-1] + 15 < peak_candidate):
        
        range_to_check = np.arange(np.max([peak_candidate-5, 0]), np.min([peak_candidate+5, angles.shape[0]])).astype('int')

        max_val = np.argmax(grads[range_to_check,:][:,segmenting_joints],axis=0)
        max_val = np.mean(max_val).astype('int')
        
        grad_min = -3
        grad_max = 3
        actual_grad_max = np.max(grads[range_to_check][:, segmenting_joints])
        actual_grad_min = np.min(grads[range_to_check][:, segmenting_joints])
        actual_max = np.max(angles[range_to_check][:,segmenting_joints])
        actual_min = np.min(angles[range_to_check][:,segmenting_joints])

        actual_max_loc = np.where(angles[range_to_check][:,segmenting_joints] == actual_max)[0][0]
        actual_min_loc = np.where(angles[range_to_check][:,segmenting_joints] == actual_min)[0][0]
        # print(actual_max_loc, actual_min_loc)
        # print(range_to_check[max_val], actual_grad_max, actual_grad_min, actual_max, actual_min)
        # print('-')
        if (actual_grad_max > grad_max \
            or actual_grad_min < grad_min) \
                and actual_max_loc > actual_min_loc:
            
            return range_to_check[max_val]
    
    return False

def plot_results(angles, peaks, joints):
    fig, ax = plt.subplots(angles.shape[1], sharex=True, sharey=True)

    for ii in range(angles.shape[1]):
        ax[ii].plot(angles[:,ii], 'k', linestyle=':')
        ax[ii].plot(np.gradient(angles[:,ii]), 'k')
        counter = 0
        for start, end in zip(peaks[:-1], peaks[1:]):
            if counter % 2 == 0:
                style = '--'
                color = 'g'
            else:
                style = '-'
                color = 'b'

            ax[ii].plot(np.arange(start, end), angles[start:end,ii], linestyle=style, color=color)

            counter += 1

        ax[ii].set_title('{}-{}-{}-{}'.format(joints[ii][0], joints[ii][1], joints[ii][2], joints[ii][3]))
    plt.show()


def main(exercise_name, filenames, segmenting_joints, labels):

    #Read the angles
    angles = []
    times = []
    for filename in filenames:
        file = np.load(filename, allow_pickle=True)
        angles.append(file['angles'])
        times.extend(file['times'])
    angles = np.vstack(angles)
    times = np.array(times)
    
    #Find peaks
    peak_candidates, grads = find_peaks(angles, segmenting_joints)

    #Get actual list of peaks
    peaks = []
    for peak_candidate in peak_candidates:
        res = check_if_new_peak(angles, grads, peaks, peak_candidate, segmenting_joints)
        if res:
            peaks.append(peak_candidate)
    # peaks.append(angles.shape[0])

    joint_groups = {'right_shoulder': [
                ['right_hip', 'right_shoulder', 'right_elbow', 'xy'],
                ['right_hip', 'right_shoulder', 'right_elbow', 'yz'],
                ['right_hip', 'right_shoulder', 'right_elbow', 'xz']
            ],
            'left_shoulder': [
                ['left_hip', 'left_shoulder', 'left_elbow', 'xy'],
                ['left_hip', 'left_shoulder', 'left_elbow', 'yz'],
                ['left_hip', 'left_shoulder', 'left_elbow', 'xz']
            ],
            'right_elbow': [
                ['right_shoulder', 'right_elbow', 'right_wrist', 'xy'],
                ['right_shoulder', 'right_elbow', 'right_wrist', 'yz'],
                ['right_shoulder', 'right_elbow', 'right_wrist', 'xz']
            ],
            'left_elbow': [
                ['left_shoulder', 'left_elbow', 'left_wrist', 'xy'],
                ['left_shoulder', 'left_elbow', 'left_wrist', 'yz'],
                ['left_shoulder', 'left_elbow', 'left_wrist', 'xz']
            ]}

    angle_order = [['right_hip', 'right_shoulder', 'right_elbow', 'xy'],
                    ['left_hip', 'left_shoulder', 'left_elbow', 'xy'],
                    ['right_hip', 'right_shoulder', 'right_elbow', 'yz'],
                    ['left_hip', 'left_shoulder', 'left_elbow', 'yz'],
                    ['right_hip', 'right_shoulder', 'right_elbow', 'xz'],
                    ['left_hip', 'left_shoulder', 'left_elbow', 'xz'],
                    ['right_shoulder', 'right_elbow', 'right_wrist', 'xy'],
                    ['left_shoulder', 'left_elbow', 'left_wrist', 'xy'],
                    ['right_shoulder', 'right_elbow', 'right_wrist', 'xz'],
                    ['left_shoulder', 'left_elbow', 'left_wrist', 'xz'],
                    ['right_shoulder', 'right_elbow', 'right_wrist', 'yz'],
                    ['left_shoulder', 'left_elbow', 'left_wrist', 'yz']]

    experts = []
    expert_duration = []
    for start, end in zip(peaks[:-1], peaks[1:]):
        experts.append({'right_shoulder': [], 'left_shoulder': [], 'right_elbow': [], 'left_elbow': []})
        for joint_group in joint_groups:
            data = []
            for angle_name in joint_groups[joint_group]:
                index = angle_order.index(angle_name)
                data.append(angles[start:end, index])
            experts[-1][joint_group].append(data)
        # experts.append(angles[start:end,:])
        length = (times[end-1] - times[start]).total_seconds()
        if length < 20:
            expert_duration.append(length)
    print(len(peaks)-1)
    plot_results(angles, peaks, file['joints'])

    np.savez('{}_updated_experts.npz'.format(exercise_name), experts=experts,
                        expert_duration=expert_duration,
                        labels=labels
                        )


if __name__ == '__main__':
    exercise_name = 'lateral_raises'

    if exercise_name == 'bicep_curls':
        filenames = ['src/quori_exercises/experts/bicep_curls_demos.npz']
        segmenting_joints = [4, 5, 8, 9]
        labels = []
        for ii in range(12):
            labels.append('Good')
        for ii in range(8):
            labels.append('low range of motion')
    
    else:
        filenames = ['src/quori_exercises/experts/lateral_raises_demos3.npz']
        segmenting_joints = [4, 5, 8, 9]
        labels = []
        for ii in range(10):
            labels.append('Good')
        for ii in range(5):
            labels.append('low range of motion')
        for ii in range(2):
            labels.append('high range of motion')
    main(exercise_name, filenames, segmenting_joints, labels)