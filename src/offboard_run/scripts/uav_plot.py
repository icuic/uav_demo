import numpy as np
import matplotlib.pyplot as plt
import pickle


def load_return_list(i, path):
    with open(f"{path}/{i}_return_list.pkl", 'rb') as f:
        return pickle.load(f)
    
def load_success_rate_list(i, path):
    with open(f"{path}/{i}_success_rate_list.pkl", 'rb') as f:
        return pickle.load(f)
    
def plot_nth_element(lst):
    # 提取每个元组的元素
    episodes = [tup[0] for tup in lst]  # 新增：提取 episode 编号
    success_rates = [tup[1] for tup in lst]
    timeout_rates = [tup[2] for tup in lst]
    crash_rates = [tup[3] for tup in lst]
    outmap_rates = [tup[4] for tup in lst]

    alpha = 0.7  # 透明度值
    linestyles = ['-', '--', '-.', ':']
    labels = ["success", "timeout", "crash", "out of map"]

    # 绘制图形
    plt.plot(episodes, success_rates, alpha=alpha, linestyle=linestyles[0], label=labels[0])
    plt.plot(episodes, timeout_rates, alpha=alpha, linestyle=linestyles[1], label=labels[1])
    plt.plot(episodes, crash_rates, alpha=alpha, linestyle=linestyles[2], label=labels[2])
    plt.plot(episodes, outmap_rates, alpha=alpha, linestyle=linestyles[3], label=labels[3])

    # 设置图形标题和坐标轴标签
    plt.title("Termination Reasons by Episode")
    plt.xlabel("Episode")
    plt.ylabel("Rate")

    # 显示图例
    plt.legend()
    # 显示图形
    plt.show()

def moving_average(a, window_size):
    cumulative_sum = np.cumsum(np.insert(a, 0, 0)) 
    middle = (cumulative_sum[window_size:] - cumulative_sum[:-window_size]) / window_size
    r = np.arange(1, window_size-1, 2)
    begin = np.cumsum(a[:window_size-1])[::2] / r
    end = (np.cumsum(a[:-window_size:-1])[::2] / r)[::-1]
    return np.concatenate((begin, middle, end))

restore_from = 3350
test_time = "0319-2315"

a = load_return_list(restore_from, f"./checkpoints/{test_time}")
# print(a[4900:4950])

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

# success/timeout/crash/out of map rate
a = load_success_rate_list(restore_from, f"./checkpoints/{test_time}")
plot_nth_element(a)
