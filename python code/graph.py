import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

# **讀取數據**
file_path = "output_20250309_211607.log"
data = []

# 讀取第 10 項數據
with open(file_path, 'r') as file:
    for line in file:
        parts = line.split()
        if len(parts) >= 13:  # 確保索引不超出範圍
            a = parts[12]
            tenth_value = float(a.replace(",", ""))  # 轉換為浮點數
            data.append(tenth_value)

# **檢查數據**
if not data:
    print("沒有提取到任何數據！")
    exit()

# **設定視窗範圍**
window_size = 100  # 預設顯示的數據點數量
total_data = len(data)

# **建立圖表**
fig, ax = plt.subplots(figsize=(10, 5))
plt.subplots_adjust(bottom=0.25)  # 調整下方留白，避免擋住滑桿

ax.set_xlim(0, window_size)  # 設定 X 軸範圍
ax.set_ylim(min(data), max(data))  # Y 軸根據數據範圍變動

line, = ax.plot([], [], marker='o', linestyle='-', color='b')

# **初始化顯示數據**
def update_plot(start):
    start = int(start)  # 轉換為整數
    end = min(start + window_size, total_data)  # 計算視窗終點
    x_data = np.arange(start, end)  # X 軸為索引
    y_data = data[start:end]  # Y 軸為對應數據
    line.set_data(x_data, y_data)
    ax.set_xlim(start, end)  # 調整 X 軸範圍
    fig.canvas.draw_idle()  # 更新圖表

# **添加滑桿**
ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03])  # 設定滑桿位置
slider = Slider(ax_slider, "Start", 0, total_data - window_size, valinit=0, valstep=1)
slider.on_changed(update_plot)  # 當滑桿移動時更新圖表

# **初始化顯示**
update_plot(0)

# **顯示圖表**
plt.title("Manually Controlled Sliding Window")
plt.xlabel("Time (s)")
plt.ylabel("Tenth Item Value")
plt.show()
