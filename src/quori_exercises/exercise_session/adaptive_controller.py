from bayesianbandits import (
    Arm,
    ContextualAgent,
    UpperConfidenceBound,
    GammaRegressor
)
from datetime import datetime
import time

USER_ID = '1'

model_filename = 'models/Participant_{}_{}.pickle'.format(USER_ID, datetime.now().strftime("%Y-%m-%d"))

data_filename = 'adaptive_data/Participant_{}_{}.txt'.format(USER_ID, datetime.now().strftime("%Y-%m-%d"))
action_filename = 'adaptive_data/Participant_{}_{}_actions.txt'.format(USER_ID, datetime.now().strftime("%Y-%m-%d"))

with open('src/quori_exercises/exercise_session/' + data_filename, 'w') as fp:
    pass

with open('src/quori_exercises/exercise_session/' + action_filename, 'w') as fp:
    pass

arms = [
            Arm(0, learner=GammaRegressor(alpha=1, beta=1)),
            Arm(1, learner=GammaRegressor(alpha=1, beta=1)),
            ]
    
policy = UpperConfidenceBound()
agent = ContextualAgent(arms, policy)

current_file_length = 0

contexts = [0]
rewards = []
actions = [1]

for ii in range(80000):
    with open('src/quori_exercises/exercise_session/' + data_filename, 'r') as f:
        lines = f.readlines()

    num_lines = len(lines)
    if num_lines > current_file_length:
        
        current = lines[num_lines-1]
        context = int(current[0])
        reward = int(current[2])

        contexts.append(context)
        rewards.append(reward)

        #Train on previous rep
        agent.select_for_update(actions[-1]).update(contexts[-1], rewards[-1])
        print('Training on context:', contexts[-1], 'reward:', rewards[-1]), 'action:', actions[-1]

        #Get agent's action
        action = agent.pull(contexts[-1])[0]

        #Save action
        #0 is firm, 1 is encouraging
        with open('src/quori_exercises/exercise_session/' + action_filename, 'a') as f2:
            f2.write(str(action) + '\n')

        print('Chose action:', action)
        current_file_length += 1
        
    
    time.sleep(0.1)