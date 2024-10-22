from bayesianbandits import (
    Arm,
    ContextualAgent,
    UpperConfidenceBound,
    GammaRegressor
)
from datetime import datetime
import time
import sys
import pickle

USER_ID = '0'

model_filename = 'src/quori_exercises/exercise_session/models/Participant_{}_{}.pickle'.format(USER_ID, datetime.now().strftime("%Y-%m-%d--%H-%M-%S"))

arms = [
            Arm(0, learner=GammaRegressor(alpha=1, beta=1)),
            Arm(1, learner=GammaRegressor(alpha=1, beta=1)),
            ]
    
policy = UpperConfidenceBound()
agent = ContextualAgent(arms, policy)

contexts = []
rewards = []
actions = []


while True:
    try:
        # print('Awaiting input...')
        user_input = input().strip()

        if user_input.lower() == 'exit':
            break

        c, a, r = map(int, user_input.split(','))

        #We are sampling the model
        if a == -1:
            action = agent.pull(c)[0]
            print(action)
            sys.stdout.flush()
        
        else:
            #We are training the model
            contexts.append(c)
            actions.append(a)
            rewards.append(r)

            agent.select_for_update(actions[-1]).update(contexts[-1], rewards[-1])
            print(1)
            sys.stdout.flush()


    except ValueError:
        pass
    
    except EOFError:
        break
    
    except Exception as e:
        print('Error: {}'.format(e))
        break
time.sleep(0.1)

data = {'agent': agent, 'contexts': contexts, 'rewards': rewards, 'actions': actions}
try:
    with open(model_filename, 'wb') as f:
        pickle.dump(data, f)
except Exception as e:
    print('Failed to save results: {}'.format(e))