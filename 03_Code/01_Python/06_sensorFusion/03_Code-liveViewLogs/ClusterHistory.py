from collections import deque

# ---------------------------
# Configurable Parameters
# ---------------------------
MAX_HISTORY_FRAMES = 10


# ---------------------------
# Cluster Tracking Classes
# ---------------------------
class ClusterHistory:
    def __init__(self, cluster_id):
        self.cluster_id = cluster_id
        self.history = deque(maxlen=MAX_HISTORY_FRAMES)
        self.missing_counter = 0

    def add_observation(self, cluster_data):
        self.history.append(cluster_data)
        self.missing_counter = 0

    def predict_next_centroid(self):
        if len(self.history) < 2:
            return self.history[-1]['centroid']
        c0 = self.history[-2]['centroid']
        c1 = self.history[-1]['centroid']
        vx = c1[0] - c0[0]
        vy = c1[1] - c0[1]
        vz = c1[2] - c0[2]
        pred = [c1[0] + vx, c1[1] + vy, c1[2] + vz]
        return pred