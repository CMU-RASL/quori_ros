import numpy as np
import pickle
import os
import json

ANGLE_INFO =  [('right_shoulder', ['right_hip', 'right_shoulder', 'right_elbow']),
           ('left_shoulder', ['left_hip', 'left_shoulder', 'left_elbow']),
           ('right_elbow', ['right_shoulder', 'right_elbow', 'right_wrist']),
           ('left_elbow', ['left_shoulder', 'left_elbow', 'left_wrist'])]
ANGLE_ORDER = {
    'right_shoulder': [0, 1, 2],
    'left_shoulder': [3, 4, 5],
    'right_elbow': [6, 7, 8],
    'left_elbow': [9, 10, 11],

}
EXERCISE_INFO = {
    'bicep_curls': {
        'segmenting_joints': [('right_elbow', 0),
                                ('left_elbow',  0),
                                ('right_elbow',  2),
                                ('left_elbow',  2)],
        'comparison_joints': [('right_shoulder', [0, 1, 2]), 
                                ('left_shoulder', [0, 1, 2]), 
                                ('right_elbow', [0, 1, 2]),
                                ('left_elbow',  [0, 1, 2])],
        'threshold1': 7500,
        'threshold2': 10000,
        'segmenting_joint_inds': [],
        'current_angles_min': 50,
        'max_grad': 3,
        'max_in_range': 130

    },
    'lateral_raises': {
        'segmenting_joints': [('right_shoulder', 2), 
                                ('left_shoulder', 2),
                                ('right_elbow', 2)],
        'comparison_joints': [('right_shoulder', [0, 1, 2]), 
                        ('left_shoulder', [0, 1, 2]), 
                        ('right_elbow', [0, 1, 2]),
                        ('left_elbow',  [0, 1, 2])],
        'threshold1': 7500,
        'threshold2': 12500,
        'segmenting_joint_inds': [],
        'current_angles_min': 35,
        'max_grad': 1.5,
        'max_in_range': 45
    }

}

for exercise in EXERCISE_INFO:
    for group, ind in EXERCISE_INFO[exercise]['segmenting_joints']:
        EXERCISE_INFO[exercise]['segmenting_joint_inds'].append(ANGLE_ORDER[group][ind])

#/home/quori4/quori_files/quori_ros/ (Add that to the beginning to run on the robot, otherwise, stick with src/)
with open('src/quori_exercises/experts/new_experts_smaller.pickle', 'rb') as handle:
    NEW_EXPERTS = pickle.load(handle)

for exercise in NEW_EXPERTS:
    candidates = []
    for rep in NEW_EXPERTS[exercise]['good']:
        candidates.append(rep[-1,0] - rep[0, 0])
    if len(candidates) > 0:
        NEW_EXPERTS[exercise]['average duration'] = np.mean(candidates)
    else:
        NEW_EXPERTS[exercise]['average duration'] = 4

NEUTRAL_EXPRESSIONS = [
    [0.1, 0, 0, 0, 0, 0],
    [0.175, 0, 0, 0, 0, 0],
    [0.25, 0, 0, 0, 0, 0],
    [0.325, 0, 0, 0, 0, 0],
    [0.4, 0, 0, 0, 0, 0]
]
NEUTRAL_POSTURES = [
    [-0.2, -1.1, 0, -1.1, 0.2],
    [-0.2, -1.1, 0, -1.1, 0.2],
    [0, -1.1, 0, -1.1, 0],
    [0.2, -1.1, 0, -1.1, -0.2],
    [0.2, -1.1, 0, -1.1, -0.2]
]

START_SET_SMILE = [
    0.3, 0.375, 0.45, 0.525, 0.6
]

NONVERBAL_REACT = {
    'positive': {'torso': [0.21*0.2, 0.21*0.25, 0.21*0.3, 0.21*0.35, 0.21*0.4],
                'expression': [0.4, 0.525, 0.65, 0.775, 0.9]},
    'negative': {'torso': [-0.47*0.2, 0-0.47*0.25, -0.47*0.3, -0.47*0.35, -0.47*0.4],
                'expression': [0.7, 0.55, 0.4, 0.25, 0.1]}
}

ALL_MESSAGES = {}
for e in ['bicep curls', 'lateral raises']:
    ALL_MESSAGES[e] = {}
    for f in ['low', 'moderate', 'high']:
        with open('src/quori_exercises/scripts/{}_{}.json'.format(e, f)) as file:
            ALL_MESSAGES[e][f] = json.load(file)