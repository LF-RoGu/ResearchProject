from ClusterHistory import ClusterHistory
import numpy as np

# ---------------------------
# Configurable Parameters
# ---------------------------
MAX_MISSES_ALLOWED = 5
MAX_ASSOCIATION_DIST = 0.3  # meters


class ClusterTracker:
    def __init__(self):
        self.tracks = {}

    def update(self, current_clusters, frame_idx):
        matched_ids = set()
        for cid, cdata in current_clusters.items():
            if cid not in self.tracks:
                self.tracks[cid] = ClusterHistory(cid, MAX_MISSES_ALLOWED)
            self.tracks[cid].add_observation(cdata, frame_idx)
            matched_ids.add(cid)

        for cid, track in self.tracks.items():
            if cid not in matched_ids:
                track.missing_counter += 1

        self.tracks = {
            cid: track for cid, track in self.tracks.items()
            if track.missing_counter <= MAX_MISSES_ALLOWED
        }

    def get_predictions(self):
        return {
            cid: track.predict_next_centroid()
            for cid, track in self.tracks.items()
            if track.missing_counter > 0
        }
    def get_active_tracks(self):
        """
        Returns a dictionary with the latest data from active tracks.
        """
        active_tracks = {}
        for cid, track in self.tracks.items():
            if track.history:
                active_tracks[cid] = track.history[-1]
        return active_tracks
    def get_predicted_clusters(self):
        """
        Return estimated positions for clusters that are currently missing but not expired.
        """
        predictions = {}
        for cid, track in self.tracks.items():
            if 0 < track.missing_counter < track.max_missing:
                est = track.predict_next_centroid()
                if est is not None:
                    predictions[cid] = est
        return predictions

    def associate_new_clusters(self, new_clusters):
        updated_clusters = {}
        assigned_new_ids = set()
        used_old_ids = set()

        # 1. Check if any existing tracks are missing and predict their next position
        predictions = {tid: track.predict_next_centroid()
                    for tid, track in self.tracks.items()
                    if 0 < track.missing_counter < track.max_missing}

        for new_cid, new_data in new_clusters.items():
            new_centroid = new_data['centroid']
            best_match = None
            best_dist = float("inf")

            for old_id, predicted_centroid in predictions.items():
                dist = np.linalg.norm(np.array(new_centroid[:3]) - np.array(predicted_centroid[:3]))
                if dist < MAX_ASSOCIATION_DIST and dist < best_dist:
                    best_dist = dist
                    best_match = old_id

            if best_match is not None:
                # Replace new cluster ID with old ID
                updated_clusters[best_match] = new_data
                used_old_ids.add(best_match)
            else:
                # Assign new ID normally
                updated_clusters[new_cid] = new_data

        return updated_clusters


