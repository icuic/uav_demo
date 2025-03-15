import numpy as np
import matplotlib.pyplot as plt
import pickle


def load_return_list(i, path):
    with open(f"{path}/{i}_return_list.pkl", 'rb') as f:
        return pickle.load(f)
    
def load_success_rate_list(i, path):
    with open(f"{path}/{i}_success_rate_list.pkl", 'rb') as f:
        return pickle.load(f)
    
def plot_nth_element(lst, i):
    # 检查序号 i 是否在有效范围内
    if i < 0 or i >= len(lst[0]):
        raise IndexError("序号 i 超出了元素的索引范围。")
    # 提取每个元素的第 i 个元素
    nth_elements = [tup[i] for tup in lst]
    # 生成 x 轴坐标
    x = range(len(lst))
    alpha = 0.7  # 透明度值
    linestyles = ['-', '--', '-.', ':']
    labels = ["success", "timeout", "crash", "out of map"]
    # 绘制图形
    for i in range(4):
        nth_elements = [tup[i] for tup in lst]
        plt.plot(x, nth_elements, alpha=alpha, linestyle=linestyles[i], label=labels[i])
    # 设置图形标题和坐标轴标签
    plt.title("")
    plt.xlabel("episodes")
    plt.ylabel("rates")

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

restore_from = 4700
test_time = "0314-1310"

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
plot_nth_element(a, 0)
