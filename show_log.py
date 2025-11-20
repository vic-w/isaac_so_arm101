import re
import matplotlib.pyplot as plt

# --------------------------------------
# 可扩展关键词列表（你只要加到这里即可）
# --------------------------------------
KEYWORDS = [
    "Episode_Reward/reaching_object:",
    "Episode_Reward/lifting_object:",
    "Episode_Reward/object_goal_tracking:"
]

# log 文件路径
LOG_PATH = "log.txt"

# --------------------------------------
# 读取 log.txt + 自动匹配关键词
# --------------------------------------
def parse_log(file_path, keywords):
    # 每个关键词对应一个 list
    data = {k: [] for k in keywords}

    with open(file_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # 匹配浮点数的正则
    float_pattern = re.compile(r"[-+]?\d*\.\d+|\d+")

    for line in lines:
        for key in keywords:
            if key in line:
                # 提取数值
                numbers = float_pattern.findall(line)
                if numbers:
                    value = float(numbers[-1])   # 取最后一个数字
                    data[key].append(value)

    return data


# --------------------------------------
# 绘图
# --------------------------------------
def plot_curves(data_dict):
    num_plots = len(data_dict)
    fig, axes = plt.subplots(num_plots, 1, figsize=(10, 3 * num_plots))
    fig.suptitle("Training Logs", fontsize=16)

    # 若只有一个子图，axes不是list，需要转list
    if num_plots == 1:
        axes = [axes]

    for ax, (key, values) in zip(axes, data_dict.items()):
        ax.plot(values, label=key, linewidth=1.5)
        ax.set_title(key)
        ax.set_xlabel("Iteration")
        ax.set_ylabel("Value")
        ax.grid(True)
        ax.legend()

    plt.tight_layout()
    plt.show()


# --------------------------------------
# 主流程
# --------------------------------------
if __name__ == "__main__":
    data = parse_log(LOG_PATH, KEYWORDS)
    plot_curves(data)
