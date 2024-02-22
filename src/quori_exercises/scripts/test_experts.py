import numpy as np
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
import matplotlib.pyplot as plt
import time
import pickle

with open('src/quori_exercises/experts/new_experts.pickle', 'rb') as handle:
    NEW_EXPERTS = pickle.load(handle)

plt.rcParams.update({'font.size': 30})

all_exercises = {'bicep_curls': {'good': [], 'low range': [], 'high range': []}, 'lateral_raises': {'good': [], 'low range': [], 'high range': []}}

labels = ['good', 'low range', 'high range']

for exercise in ['bicep_curls', 'lateral_raises']:
    print(exercise)
    all_experts = []
    all_expert_labels = []
    for label in labels:
        if label in NEW_EXPERTS[exercise]:
            all_experts.extend(NEW_EXPERTS[exercise][label])
            all_expert_labels.extend([label for ii in range(len(NEW_EXPERTS[exercise][label]))])
    print('All experts', len(all_experts))
    # pairwise_distances = {'same': [], 'different': []}

    experts_to_remove = []
    for ind1, expert1 in enumerate(all_experts):
        for ind2, expert2 in enumerate(all_experts):
            if ind1 > ind2:
                dist = fastdtw(expert1[:,1:], expert2[:,1:], dist=euclidean)[0]
                if all_expert_labels[ind1] == all_expert_labels[ind2] and dist < 2000:
                    if ind2 not in experts_to_remove:
                        experts_to_remove.append(ind2)

    all_expert_labels = [ele for idx, ele in enumerate(all_expert_labels) if idx not in experts_to_remove]
    all_experts = [ele for idx, ele in enumerate(all_experts) if idx not in experts_to_remove]

    # for ind1, expert1 in enumerate(all_experts):
    #     for ind2, expert2 in enumerate(all_experts):
    #         if ind1 > ind2 and ind1 not in experts_to_remove and ind2 not in experts_to_remove:
    #             dist = fastdtw(expert1[:,1:], expert2[:,1:], dist=euclidean)[0]
    #             if all_expert_labels[ind1] == all_expert_labels[ind2]:
    #                 pairwise_distances['same'].append(dist)
    #             else:
    #                 pairwise_distances['different'].append(dist)

    # fig2, ax2 = plt.subplots()
    # n_bins = 10

    # labels = []
    # data = []
    # for key_ind, key in enumerate(pairwise_distances.keys()):
    #     labels.append(str(key))
    #     ax2.boxplot(pairwise_distances[key], positions=[key_ind], labels=[key])

    # ax2.set_title(exercise)
    # plt.show()

    #Create new experts
    print('New all experts', len(all_experts))
    for label, expert in zip(all_expert_labels, all_experts):
        all_exercises[exercise][label].append(expert)

print(len(all_exercises['lateral_raises']['good']))

with open('new_experts_smaller.pickle', 'wb') as handle:
    pickle.dump(all_exercises, handle, protocol=pickle.HIGHEST_PROTOCOL)


#time a new expert
# test_expert = np.random.random((120, 13))

# start = time.time()
# for label, experts in new_experts.items():
#     for expert in experts:
#         dist = fastdtw(expert[:,1:], test_expert[:,1:], dist=euclidean)
# end = time.time()
# print('Time', end-start)