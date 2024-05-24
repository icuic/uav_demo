import numpy as np
import matplotlib.pyplot as plt
import pickle


def load_return_list(i, path):
    with open(f"{path}/{i}_return_list.pkl", 'rb') as f:
        return pickle.load(f)
    
def load_steps_distance_list(i, path):
    with open(f"{path}/{i}_step_distance_list.pkl", 'rb') as f:
        return pickle.load(f)
    
def moving_average(a, window_size):
    cumulative_sum = np.cumsum(np.insert(a, 0, 0)) 
    middle = (cumulative_sum[window_size:] - cumulative_sum[:-window_size]) / window_size
    r = np.arange(1, window_size-1, 2)
    begin = np.cumsum(a[:window_size-1])[::2] / r
    end = (np.cumsum(a[:-window_size:-1])[::2] / r)[::-1]
    return np.concatenate((begin, middle, end))

a = load_return_list(499, "./checkpoints/0521-1039")

episodes_list = list(range(len(a)))
plt.plot(episodes_list, a)
plt.xlabel('Episodes')
plt.ylabel('Returns')
plt.title('Return value per episode')
plt.show()

mv_return = moving_average(a, 9)
plt.plot(episodes_list, mv_return)
plt.xlabel('Episodes')
plt.ylabel('Returns')
plt.title('Sliding average per 9 episodes')
plt.show()      