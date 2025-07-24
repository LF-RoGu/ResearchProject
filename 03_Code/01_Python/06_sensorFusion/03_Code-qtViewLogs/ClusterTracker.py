import numpy as np

class ClusterTracker:
    """
    A spatial-matching tracker that assigns persistent IDs to clusters
    using nearest-centroid association with a distance threshold,
    reusing IDs of pruned tracks to keep IDs compact.
    Now includes a 'hits' counter for each track.
    """
    def __init__(self, max_misses=5, dist_threshold=2.0):
        # Next new ID to assign if no free IDs are available
        self.next_id = 0
        # Reusable ID pool for pruned tracks
        self.free_ids = []
        # Active tracks: track_id -> track data
        # Each track data contains:
        # 'centroid': np.array([x,y,z]),
        # 'points': NxM array,
        # 'doppler_avg': float,
        # 'missed': int,
        # 'hits': int (number of successful matches)
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
        Update tracks with new detections.

        detections: dict[temp_id -> {'centroid': [x,y,z,...], 'points': NxM array, 'doppler_avg': float}]
        """
        track_ids = list(self.tracks.keys())
        det_ids = list(detections.keys())

        # 1. If no existing tracks, initialize all detections
        if not track_ids:
            for cid in det_ids:
                data = detections[cid]
                new_id = self._get_new_id()
                self.tracks[new_id] = {
                    'centroid':    np.array(data['centroid'][:3]),
                    'points':      data['points'],
                    'doppler_avg': data.get('doppler_avg', 0.0),
                    'missed':      0,
                    'hits':        1
                }
            return

        # 2. Compute pairwise distances for matching
        distances = []  # list of (track_id, det_id, distance)
        for tid in track_ids:
            t_cent = self.tracks[tid]['centroid']
            for cid in det_ids:
                d_cent = np.array(detections[cid]['centroid'][:3])
                distances.append((tid, cid, np.linalg.norm(t_cent - d_cent)))

        # 3. Greedy match by smallest distance within threshold
        distances.sort(key=lambda x: x[2])
        unmatched_tracks = set(track_ids)
        unmatched_dets = set(det_ids)

        for tid, cid, dist in distances:
            if dist <= self.dist_threshold and tid in unmatched_tracks and cid in unmatched_dets:
                data = detections[cid]
                # update matched track
                self.tracks[tid]['centroid']    = np.array(data['centroid'][:3])
                self.tracks[tid]['points']      = data['points']
                self.tracks[tid]['doppler_avg'] = data.get('doppler_avg', self.tracks[tid]['doppler_avg'])
                self.tracks[tid]['missed']      = 0
                # increment hits for this successful match
                self.tracks[tid]['hits']       += 1
                unmatched_tracks.remove(tid)
                unmatched_dets.remove(cid)

        # 4. Create new tracks for unmatched detections
        for cid in unmatched_dets:
            data = detections[cid]
            new_id = self._get_new_id()
            self.tracks[new_id] = {
                'centroid':    np.array(data['centroid'][:3]),
                'points':      data['points'],
                'doppler_avg': data.get('doppler_avg', 0.0),
                'missed':      0,
                'hits':        1
            }

        # 5. Increment 'missed' for tracks not updated
        for tid in unmatched_tracks:
            self.tracks[tid]['missed'] += 1

        # 6. Prune tracks that have been missing too long and recycle ID
        for tid in list(self.tracks.keys()):
            if self.tracks[tid]['missed'] > self.max_misses:
                self.free_ids.append(tid)
                del self.tracks[tid]

    def get_predictions(self):
        """
        Return predicted positions for tracks that missed the current frame.
        """
        preds = {}
        for tid, trk in self.tracks.items():
            if trk['missed'] > 0:
                preds[tid] = tuple(trk['centroid'])
        return preds

    def get_active_tracks(self):
        """
        Return all active tracks and their data.
        """
        return {tid: trk.copy() for tid, trk in self.tracks.items()}
