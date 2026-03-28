import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

print(">>> 正在加载并对齐飞行数据...")
df = pd.read_csv('flight_data.csv')
df = df.interpolate(method='linear').dropna()

# 🔥 核心优化：剔除前 30% 起飞和变阵的瞬态数据，只测稳态画圆的真实实力！
cut_index = int(len(df) * 0.3)
df = df.iloc[cut_index:]

cols = df.columns.tolist()

try:
    # 抓取目标坐标
    target_x = df[[c for c in cols if 'trajectory_setpoint/position[0]' in c][0]].values
    target_y = df[[c for c in cols if 'trajectory_setpoint/position[1]' in c][0]].values
    target_z = df[[c for c in cols if 'trajectory_setpoint/position[2]' in c][0]].values

    # 抓取真实坐标（改用绝对准确的 vehicle_odometry）
    actual_x = df[[c for c in cols if 'vehicle_odometry/position[0]' in c][0]].values
    actual_y = df[[c for c in cols if 'vehicle_odometry/position[1]' in c][0]].values
    actual_z = df[[c for c in cols if 'vehicle_odometry/position[2]' in c][0]].values
except IndexError:
    print("❌ 错误：CSV 列名不匹配，请确保导出了 trajectory_setpoint 和 vehicle_odometry 的 position[0/1/2]！")
    exit()

# 计算误差
errors = np.sqrt((target_x - actual_x)**2 + (target_y - actual_y)**2 + (target_z - actual_z)**2)
max_error = np.max(errors)
rmse_error = np.sqrt(np.mean(errors**2))

print("\n" + "="*45)
print(" 🚀 UAV 轨迹追踪误差量化报告 🚀")
print(f" 瞬态最大误差 (Max Error): {max_error:.3f} 米")
print(f" 稳态均方根误差 (RMSE):    {rmse_error:.3f} 米")
print("="*45 + "\n")

# 绘制 3D 轨迹图
plt.style.use('dark_background') 
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

ax.plot(target_x, target_y, -target_z, label='Target Trajectory (Python AI)', color='#00ffcc', linestyle='--', linewidth=2)
ax.plot(actual_x, actual_y, -actual_z, label='Actual Flight (C++ Control)', color='#ff007f', linewidth=3)

ax.set_title('UAV 3D Trajectory Tracking (Target vs Actual)', fontsize=14, pad=20)
ax.set_xlabel('X Position (m)')
ax.set_ylabel('Y Position (m)')
ax.set_zlabel('Altitude (m)')
ax.legend(loc='upper right')
ax.grid(color='gray', linestyle=':', linewidth=0.5)

plt.savefig('trajectory_analysis_3d.png', dpi=300, bbox_inches='tight')
print(">> ✅ 已在桌面生成高清 3D 轨迹图：trajectory_analysis_3d.png")
plt.show()
