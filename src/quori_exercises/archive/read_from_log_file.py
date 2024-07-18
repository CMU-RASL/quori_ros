import numpy as np
import matplotlib.pyplot as plt
import pickle

log_filename = 'src/quori_exercises/saved_data/{}.pickle'
dbfile = open(log_filename, 'rb')    
data = pickle.load(dbfile)
dbfile.close()

print(data['hrr'].shape)
plt.plot(data['heart_rates'][0,:])

plt.show()