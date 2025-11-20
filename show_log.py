import re
import matplotlib.pyplot as plt

def plot_reaching_object(log_path):
    reaching_values = []

    # 正则匹配：Episode_Reward/reaching_object: 0.0349
    pattern = re.compile(r"Episode_Reward/reaching_object:\s*([+-]?\d+\.\d+)")

    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            match = pattern.search(line)
            if match:
                value = float(match.group(1))
                reaching_values.append(value)

    # 打印数量以确认
    print(f"找到 {len(reaching_values)} 个 reaching_object 数据点")

    # 画图
    plt.figure(figsize=(10, 4))
    plt.plot(reaching_values, linewidth=1.5)
    plt.title("Episode_Reward / reaching_object 曲线")
    plt.xlabel("Iteration")
    plt.ylabel("Reaching Reward")
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_reaching_object("log.txt")
