import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Submap aggregation and plotting
def plot_submap(frames_data):
    aggregated_points = []
    frame_count = 0

    for frame in frames_data.values():
        for tlv in frame["TLVs"]:
            if "Type 1 Data" in tlv:
                for point in tlv["Type 1 Data"]:
                    aggregated_points.append([point["X [m]"], point["Y [m]"], point["Z [m]"]])
        frame_count += 1
        if frame_count == 10:
            break

    # Convert to numpy array
    aggregated_points = np.array(aggregated_points)

    # Plot the aggregated points
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(aggregated_points[:, 0], aggregated_points[:, 1], aggregated_points[:, 2], c='b', marker='o')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('Submap Aggregation - First 10 Frames')
    plt.show()