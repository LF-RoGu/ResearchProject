from ClusterHistory import ClusterHistory

# ---------------------------
# Configurable Parameters
# ---------------------------
MAX_MISSES_ALLOWED = 5


class ClusterTracker:
    def __init__(self):
        self.tracks = {}

    def update(self, current_clusters):
        matched_ids = set()
        for cid, cdata in current_clusters.items():
            if cid not in self.tracks:
                self.tracks[cid] = ClusterHistory(cid)
            self.tracks[cid].add_observation(cdata)
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