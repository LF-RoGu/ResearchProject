from ClusterHistory import ClusterHistory
import numpy as np

# ---------------------------
# Configurable Parameters
# ---------------------------
MAX_MISSES_ALLOWED = 5
MAX_ASSOCIATION_DIST = 0.5  # meters


class ClusterTracker:
    def __init__(self):
        self.tracks = {}
        self.max_jump  = MAX_ASSOCIATION_DIST

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

        # prune expired
        self.tracks = {
            cid: track
            for cid, track in self.tracks.items()
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
        updated_clusters  = {}
        assigned_new_ids  = set()
        used_track_ids    = set()

        # 1. Gather both predicted positions (for missing tracks) and last centroids (for active ones)
        candidate_tracks = {}
        for cid, track in self.tracks.items():
            if 0 < track.missing_counter < track.max_missing:
                candidate_tracks[cid] = track.predict_next_centroid()
            elif track.missing_counter == 0:
                candidate_tracks[cid] = track.history[-1]['centroid']

        for new_cid, new_data in new_clusters.items():
            new_centroid = new_data['centroid']
            best_match   = None
            best_dist    = float("inf")

            for track_id, ref_centroid in candidate_tracks.items():
                if track_id in used_track_ids:
                    continue  # already matched

                dist = np.linalg.norm(
                    np.array(new_centroid[:3]) -
                    np.array(ref_centroid[:3])
                )

                if dist < MAX_ASSOCIATION_DIST and dist < best_dist:
                    best_match = track_id
                    best_dist  = dist

            # Optional: “no giant jumps” rule
            if best_match is not None:
                last_centroid = self.tracks[best_match].history[-1]['centroid']
                jump_dist = np.linalg.norm(
                    np.array(new_centroid[:3]) -
                    np.array(last_centroid[:3])
                )
                if jump_dist > self.max_jump:
                    best_match = None

            # Assign or create new
            if best_match is not None:
                updated_clusters[best_match] = new_data
                used_track_ids.add(best_match)
            else:
                updated_clusters[new_cid] = new_data
                assigned_new_ids.add(new_cid)

        return updated_clusters


