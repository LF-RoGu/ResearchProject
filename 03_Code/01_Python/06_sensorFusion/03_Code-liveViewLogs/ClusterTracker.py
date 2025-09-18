import numpy as np

# === Global Configurable Parameters ===
NUMBER_OF_PAST_SAMPLES = 5           # number of past centroids to use for line fitting
LINE_FIT_UPDATE_PERIOD = 2           # hits between line refits
MAX_HISTORY_SEGMENT_LENGTH = 1.0     # max end-to-end history length (m)
LINE_DISTANCE_THRESHOLD = 2.0        # max distance from line to accept cluster
MAX_CONSECUTIVE_MISSES = 5           # frames to miss before track removal

class ClusterTracker:
    # Tracks clusters using line-based assignment
    def __init__(
        self,
        maximum_misses=MAX_CONSECUTIVE_MISSES,
        distance_threshold=LINE_DISTANCE_THRESHOLD,
        line_fit_period=LINE_FIT_UPDATE_PERIOD,
        maximum_history_length=NUMBER_OF_PAST_SAMPLES,
        maximum_segment_length=MAX_HISTORY_SEGMENT_LENGTH
    ):
        # ID management
        self.next_track_id = 0
        self.recycled_track_ids = []
        # storage for active tracks: id -> data dict
        self.tracks = {}
        # parameters
        self.maximum_misses = maximum_misses
        self.distance_threshold = distance_threshold
        self.line_fit_period = line_fit_period
        self.maximum_history_length = maximum_history_length
        self.maximum_segment_length = maximum_segment_length

    def _get_new_track_id(self):
        tid = self.next_track_id
        self.next_track_id += 1
        return tid


    def _fit_line_to_history(self, history_points):
        # history_points: list of [x,y] arrays, len ≥ 2
        pts = np.vstack(history_points)
        xs, ys = pts[:,0], pts[:,1]

        # build design matrix for y = m x + b
        A = np.vstack([xs, np.ones_like(xs)]).T
        # solve the normal equations A @ [m,b] = ys
        m, b = np.linalg.lstsq(A, ys, rcond=None)[0]

        return m, b


    def update(self, detections):
        # detections: dict of id -> {'centroid':[x,y], 'points':..., 'doppler_avg':...}
        det_ids = list(detections.keys())
        if not self.tracks:
            # no existing tracks: initialize one per detection
            for cid in det_ids:
                data = detections[cid]
                tid = self._get_new_track_id()
                cent = np.array(data['centroid'][:2])
                self.tracks[tid] = {
                    'centroid': cent,
                    'points': data['points'],
                    'doppler_average': data.get('doppler_avg', 0.0),
                    'hit_count': 1,
                    'miss_count': 0,
                    'history': [cent.copy()],
                    'fitted_line': None,
                    'last_fit_hit_count': 0
                }
            return

        # build matches (track, detection, distance)
        candidates = []
        for tid, track in self.tracks.items():
            history = track['history']
            # refit condition
            if track['hit_count'] >= 2 and \
               (track['hit_count'] - track['last_fit_hit_count']) >= self.line_fit_period:
                # prune by segment length
                while len(history) > 1 and \
                      np.linalg.norm(history[-1] - history[0]) > self.maximum_segment_length:
                    history.pop(0)
                # limit history length
                if len(history) > self.maximum_history_length:
                    history[:] = history[-self.maximum_history_length:]
                # refit line
                slope, intercept = self._fit_line_to_history(history)
                track['fitted_line'] = (slope, intercept)
                track['last_fit_hit_count'] = track['hit_count']

            # compute distance
            if track['hit_count'] >= 2 and track['fitted_line']:
                slope, intercept = track['fitted_line']
                for cid in det_ids:
                    x0, y0 = detections[cid]['centroid'][:2]
                    dist = abs(slope * x0 - y0 + intercept) / np.sqrt(slope**2 + 1)
                    candidates.append((tid, cid, dist))
            else:
                last = history[-1]
                for cid in det_ids:
                    x0, y0 = detections[cid]['centroid'][:2]
                    candidates.append((tid, cid, np.linalg.norm(last - np.array([x0, y0]))))

        # greedy match
        candidates.sort(key=lambda x: x[2])
        unmatched_tracks = set(self.tracks)
        unmatched_dets = set(det_ids)
        for tid, cid, dist in candidates:
            if dist <= self.distance_threshold and tid in unmatched_tracks and cid in unmatched_dets:
                data = detections[cid]
                new_cent = np.array(data['centroid'][:2])
                trk = self.tracks[tid]
                trk['centroid'] = new_cent
                trk['points'] = data['points']
                trk['doppler_average'] = data.get('doppler_avg', trk['doppler_average'])
                trk['hit_count'] += 1
                trk['miss_count'] = 0
                trk['history'].append(new_cent.copy())
                unmatched_tracks.remove(tid)
                unmatched_dets.remove(cid)

        # new tracks for unmatched detections
        for cid in unmatched_dets:
            data = detections[cid]
            tid = self._get_new_track_id()
            cent = np.array(data['centroid'][:2])
            self.tracks[tid] = {
                'centroid': cent,
                'points': data['points'],
                'doppler_average': data.get('doppler_avg', 0.0),
                'hit_count': 1,
                'miss_count': 0,
                'history': [cent.copy()],
                'fitted_line': None,
                'last_fit_hit_count': 0
            }

        # remove stale tracks
        for tid in list(unmatched_tracks):
            trk = self.tracks[tid]
            trk['miss_count'] += 1
            if trk['miss_count'] > self.maximum_misses:
                #self.recycled_track_ids.append(tid)
                del self.tracks[tid]

    def get_active_tracks(self):
        # return copy of tracks
        return {tid: data.copy() for tid, data in self.tracks.items()}
