import numpy as np

class ClusterTracker:
    """
    A spatial-matching tracker that assigns persistent IDs to clusters
    using nearest-centroid association with a distance threshold,
    reusing IDs of pruned tracks to keep IDs compact.
    Now includes 'hits', 'missed', and full 'history' for each track,
    and uses a simple constant-velocity model for predictions.
    """
    def __init__(self, max_misses=5, dist_threshold=2.0):
        # Next new ID to assign if no free IDs are available
        self.next_id = 0
        # Reusable ID pool for pruned tracks
        self.free_ids = []
        # Active tracks: track_id -> track data dict
        # Each track data contains:
        #   'centroid':    np.array([x,y,z]),
        #   'points':      NxM array,
        #   'doppler_avg': float,
        #   'missed':      int,
        #   'hits':        int,
        #   'history':     list of centroid arrays
        self.tracks = {}
        # Parameters
        self.max_misses = max_misses
        self.dist_threshold = dist_threshold

    def _get_new_id(self):
        """
        Return a new or recycled track ID.
        """
        if self.free_ids:
            self.free_ids.sort()
            return self.free_ids.pop(0)
        tid = self.next_id
        self.next_id += 1
        return tid

    def update(self, detections):
        """
        Update tracks with new detections, matching by spatial distance to
        predicted positions (constant velocity) when missed > 0.

        detections: dict[temp_id -> {'centroid': [x,y,z,...], 'points': NxM array, 'doppler_avg': float}]
        """
        track_ids = list(self.tracks.keys())
        det_ids = list(detections.keys())

        # 1. Initialize if no existing tracks
        if not track_ids:
            for cid in det_ids:
                data = detections[cid]
                new_id = self._get_new_id()
                cent = np.array(data['centroid'][:3])
                self.tracks[new_id] = {
                    'centroid':    cent,
                    'points':      data['points'],
                    'doppler_avg': data.get('doppler_avg', 0.0),
                    'missed':      0,
                    'hits':        1,
                    'history':     [cent.copy()]
                }
            return

        # 2. Compute predicted centroids for all tracks
        predictions = {}
        for tid, trk in self.tracks.items():
            hist = trk['history']
            if trk['missed'] > 0 and len(hist) >= 2:
                # simple constant-velocity: last - prev
                pred = hist[-1] + (hist[-1] - hist[-2])
            else:
                pred = trk['centroid']
            predictions[tid] = pred

        # 3. Compute pairwise distances using predicted positions
        distances = []
        for tid, pred_cent in predictions.items():
            for cid in det_ids:
                d_cent = np.array(detections[cid]['centroid'][:3])
                distances.append((tid, cid, np.linalg.norm(pred_cent - d_cent)))

        # 4. Greedy match by ascending distance within threshold
        distances.sort(key=lambda x: x[2])
        unmatched_tracks = set(track_ids)
        unmatched_dets = set(det_ids)

        for tid, cid, dist in distances:
            if dist <= self.dist_threshold and tid in unmatched_tracks and cid in unmatched_dets:
                data = detections[cid]
                new_cent = np.array(data['centroid'][:3])
                # update matched track
                trk = self.tracks[tid]
                trk['centroid']    = new_cent
                trk['points']      = data['points']
                trk['doppler_avg'] = data.get('doppler_avg', trk['doppler_avg'])
                trk['missed']      = 0
                trk['hits']       += 1
                trk['history'].append(new_cent.copy())
                unmatched_tracks.remove(tid)
                unmatched_dets.remove(cid)

        # 5. Create new tracks for unmatched detections
        for cid in unmatched_dets:
            data = detections[cid]
            new_id = self._get_new_id()
            cent = np.array(data['centroid'][:3])
            self.tracks[new_id] = {
                'centroid':    cent,
                'points':      data['points'],
                'doppler_avg': data.get('doppler_avg', 0.0),
                'missed':      0,
                'hits':        1,
                'history':     [cent.copy()]
            }

        # 6. Increment 'missed' for previously unmatched tracks
        for tid in unmatched_tracks:
            self.tracks[tid]['missed'] += 1

        # 7. Prune tracks that exceed missed threshold and recycle ID
        for tid in list(self.tracks.keys()):
            if self.tracks[tid]['missed'] > self.max_misses:
                self.free_ids.append(tid)
                del self.tracks[tid]

    def get_predictions(self):
        """
        Return predicted positions (tuple) for tracks that missed the current frame.
        """
        preds = {}
        for tid, trk in self.tracks.items():
            if trk['missed'] > 0:
                # repeat the same constant-velocity logic
                hist = trk['history']
                if len(hist) >= 2:
                    pred = hist[-1] + (hist[-1] - hist[-2])
                else:
                    pred = trk['centroid']
                preds[tid] = tuple(pred)
        return preds

    def get_active_tracks(self):
        """
        Return all active tracks and their data, including full history.
        """
        return {tid: trk.copy() for tid, trk in self.tracks.items()}
