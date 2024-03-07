#!/usr/bin/env python3
#import rospy
from array import array
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from config import *
import datetime
import pickle

#Storage
all_exercises = {'bicep_curls': {'good': [], 'low range': []}, 'lateral_raises': {'good': [], 'low range': [], 'high range': []}}

#Bag File Names
bag_names = ['Tarun_testing.bag']

#Labels
labels = [
    ['good', 'good', 'good', 'good', 'good', 'low range', 'low range', 'low range', 'low range', 'low range']
]

#Exercise Names
current_exercises = ['bicep_curls']

'''file = np.load('../experts/Tarun_testing.npz')
newAngles = file['angles']
#print(len(newAngles))
newPeaks = [0]
newTimes = []
current_exercise = 'lateral_raises'
grad = []
    
for i in range(len(newAngles)):
    angleSubAry = newAngles[:i]
    newTimes.append(i)
    if angleSubAry.shape[0] > 70 and angleSubAry.shape[0] % 15 == 0:
        #If far enough away from previous peak
                
        #Check if new rep
        for index in EXERCISE_INFO[current_exercise]['segmenting_joint_inds']:
            grad.append(np.max(np.gradient(angleSubAry[-30:,][:,index])))

        current_angles = angleSubAry[-30:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]
        ind = len(grad) - 60
        grad = grad[ind:]
        #print("grad", grad)
        #print("current angles", current_angles, np.min(current_angles), np.max(current_angles))

        #print(angleSubAry.shape[0], np.min(current_angles), grad)

        max_ind = np.argmax(current_angles)
        min_ind = np.argmin(current_angles)
        print(max_ind, min_ind)
        interval_ends_at_top = np.max(current_angles[min_ind//2:]) > 150


        if (current_exercise == 'bicep_curls' and np.min(current_angles) < 60) or \
        (current_exercise == 'lateral_raises' and np.max(current_angles) > 150 and interval_ends_at_top):
            
            print(interval_ends_at_top, np.max(current_angles))
            #Add peak
            current_angles = angleSubAry[-30:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]
            min_angles_index = np.argmin(current_angles, axis=0)
            min_angles = [current_angles[min_angles_index[i], i] for i in range(current_angles.shape[1])]
            #print('--', min_angles_index, min_angles)
            
            # peaks.append(angles.shape[0] - 1)

            peak_candidate = max_ind//2
            if i%30==0:
                peak_candidate += i-30
            else:
                peak_candidate += i-30

            print(i-30, max_ind//2, peak_candidate, i)
            # print(np.min(angles[peaks[-1]:peak_candidate,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]))
            to_check = angleSubAry[newPeaks[-1]:peak_candidate,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]

            indx = EXERCISE_INFO[current_exercise]['segmenting_joint_inds'][0]
            print("to_check", to_check.shape[0])
            print(newAngles[newPeaks[-1]:peak_candidate,:][:,indx])
            if to_check.shape[0] and np.min(newAngles[newPeaks[-1]:peak_candidate,:][:,indx]) < 100: 
                print("peak check", newAngles[newPeaks[-1]:peak_candidate,:][:,indx])                                         
                newPeaks.append(peak_candidate)
            # print('--', angles.shape[0]-min_angles_index[np.argmin(min_angles)])
            print("new peak", newPeaks[-1])

newPeaks.append(newAngles.shape[0] - 1)
print(newPeaks)
newTimes = np.array(newTimes)

# for peak_num, (beg, end) in enumerate(zip(peaks[:-1], peaks[1:])):
#     if end-beg > 2:
#         time_data = (times[beg:end] - times[beg]).reshape(-1,1)
#         angle_data = angles[beg:end,:]
#         data = np.hstack([time_data, angle_data])
#         all_exercises[current_exercise][label[peak_num]].append(data)

fig, ax = plt.subplots(4, 3)
ii = 0
for row in range(4):
    for col in range(3):
        ax[row, col].plot(newAngles[:,ii], 'k')
        peak_num = 0
        for peak_num, (beg, end) in enumerate(zip(newPeaks[:-1], newPeaks[1:])):
            if peak_num % 2 == 0:
                color = 'b'
            else:
                color = 'g'
            ax[row, col].plot(np.arange(beg, end), newAngles[beg:end,ii], color)
            
        ii += 1'''

for bag_name, current_exercise, label in zip(bag_names, current_exercises, labels):
    bag = rosbag.Bag(bag_name)

    angles = np.empty((0, 12))
    peaks = [0]
    times = []
    grad = []

    for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/joint_angles'])):

        #Read angle from message
        angle = msg.data
        angles = np.vstack((angles, np.array(angle)))

        #Get time
        dt = datetime.datetime.utcfromtimestamp(t.to_sec())
        if angles.shape[0] == 1:
            initial_time = dt
        times.append((dt - initial_time).total_seconds())

        #Look for new peaks
        if angles.shape[0] > 70 and angles.shape[0] % 15 == 0:
        
            #If far enough away from previous peak
                
            #Check if new rep
            #grad = []
            for index in EXERCISE_INFO[current_exercise]['segmenting_joint_inds']:
                grad.append(np.max(np.gradient(angles[-30:,][:,index])))
            
            current_angles = angles[-30:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]
            ind = len(grad) - 60
            grad = grad[ind:]

            print(angles.shape[0], np.min(current_angles), grad)


            max_ind = np.argmax(current_angles)
            min_ind = np.argmin(current_angles)
            print(max_ind, min_ind)
            interval_ends_at_top = np.max(current_angles[min_ind//2:]) > 150

            if (current_exercise == 'bicep_curls' and np.min(current_angles) < 60) or \
            (current_exercise == 'lateral_raises' and np.max(current_angles) > 150 and interval_ends_at_top):

                #Add peak
                current_angles = angles[-30:,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]
                min_angles_index = np.argmin(current_angles, axis=0)
                min_angles = [current_angles[min_angles_index[i], i] for i in range(current_angles.shape[1])]
                print('--', min_angles_index, min_angles)
                
                # peaks.append(angles.shape[0] - 1)
                #peak_candidate = angles.shape[0]-20+min_angles_index[np.argmin(min_angles)]

                peak_candidate = max_ind//2
                peak_candidate += idx-30


                # print(np.min(angles[peaks[-1]:peak_candidate,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]))
                to_check = angles[peaks[-1]:peak_candidate,:][:,EXERCISE_INFO[current_exercise]['segmenting_joint_inds']]

                indx = EXERCISE_INFO[current_exercise]['segmenting_joint_inds'][0]
            
                if to_check.shape[0] and np.min(angles[peaks[-1]:peak_candidate,:][:,indx]) < 100:                                         
                    peaks.append(peak_candidate)
                # print('--', angles.shape[0]-min_angles_index[np.argmin(min_angles)])
                print(peaks[-1])
    
    peaks.append(angles.shape[0] - 1)
    print(peaks)
    times = np.array(times)

    # for peak_num, (beg, end) in enumerate(zip(peaks[:-1], peaks[1:])):
    #     if end-beg > 2:
    #         time_data = (times[beg:end] - times[beg]).reshape(-1,1)
    #         angle_data = angles[beg:end,:]
    #         data = np.hstack([time_data, angle_data])
    #         all_exercises[current_exercise][label[peak_num]].append(data)

    fig, ax = plt.subplots(4, 3)
    ii = 0
    for row in range(4):
        for col in range(3):
            ax[row, col].plot(angles[:,ii], 'k')
            peak_num = 0
            for peak_num, (beg, end) in enumerate(zip(peaks[:-1], peaks[1:])):
                if peak_num % 2 == 0:
                    color = 'b'
                else:
                    color = 'g'
                ax[row, col].plot(np.arange(beg, end), angles[beg:end,ii], color)
                
            ii += 1

    #Save data
    # with open('new_experts.pickle', 'wb') as handle:
    #     pickle.dump(all_exercises, handle, protocol=pickle.HIGHEST_PROTOCOL)

    np.savez('Tarun_testing.npz', angles=angles)
plt.show()