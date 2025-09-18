import matplotlib.pyplot as plt
import numpy as np

# --- Figure setup ---
fig, ax = plt.subplots(figsize=(6,10))
ax.set_facecolor("gray")
ax.set_xlim(-10, 10)
ax.set_ylim(0, 20)

# Draw rectangular frame
ax.plot([-9, 9, 9, -9, -9], [0, 0, 20, 20, 0], color="white", linewidth=2)

# --- Radar sensor block (at bottom) ---
sensor_width, sensor_height = 3, 1
sensor_x, sensor_y = -sensor_width/2, 0.5
ax.add_patch(plt.Rectangle((sensor_x, sensor_y), sensor_width, sensor_height, 
                           color="white", ec="black"))

# Sensor label
ax.text(0, -0.5, "IWR6843AOP", ha="center", color="white", fontsize=14, weight="bold")
ax.text(0, -1.5, "(60–64 GHz)", ha="center", color="white", fontsize=12)

# --- Outgoing waves (conical, light gray) ---
cone_angle = np.deg2rad(60)  # 60° field of view
for r in range(3, 20, 3):
    theta = np.linspace(-cone_angle/2, cone_angle/2, 200)
    x = r * np.sin(theta)
    y = r * np.cos(theta) + sensor_y + sensor_height
    ax.plot(x, y, color="lightgray", alpha=0.9)

# --- Objects in the field (darker gray boxes) ---
objects = [(-4, 10), (3, 13), (2, 7)]
for ox, oy in objects:
    ax.add_patch(plt.Rectangle((ox-0.6, oy-0.6), 1.2, 1.2, color="dimgray"))

# --- Returning waves (conical, red) from one object ---
target_x, target_y = 2, 7
return_angle = np.deg2rad(40)  # narrower cone
for r in range(2, 8, 2):
    theta = np.linspace(-return_angle/2, return_angle/2, 200)
    x = target_x + r * np.sin(theta)
    y = target_y - r * np.cos(theta)
    ax.plot(x, y, color="red", linewidth=2, alpha=0.8)

# Clean look
ax.axis("off")
plt.show()
