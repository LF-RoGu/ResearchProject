import numpy as np
from scipy.spatial import cKDTree

# Global variable to store the last valid transformation
_last_valid_transformation = {
    'translation_avg': np.zeros(3),
    'rotation_avg': 0.0
}
#Uses hits^alpha as the weighting factor to reduce the effect of newly detected clusters with low hit counts.
weightedAlpha=2.0
min_alpha = 1.0
max_alpha = 2.0
max_hits = 30  

def _normalize_icp_input(icp):
    """
    Normalize ICP input which may be:
      - None
      - dict mapping cid -> data
      - iterable of (cid, data) pairs
      - other iterable (skip non-pairs)
    Returns a list of (cid, data).
    """
    if icp is None:
        return []
    # dict: use items()
    if isinstance(icp, dict):
        return list(icp.items())
    pairs = []
    try:
        for item in icp:
            if isinstance(item, (list, tuple)) and len(item) == 2:
                pairs.append((item[0], item[1]))
            else:
                # skip elements that cannot unpack
                continue
    except TypeError:
        # not iterable
        return []
    return pairs


def icp_translation_vector(icpP, icpQ):
    """
    Robust ICP for 2D radar odometry with KD-tree matching.
    Preserves last valid transformation if no matches.

    Changes made:
    - Added KD-tree nearest-neighbor matching to handle unequal point counts.
    - Added outlier rejection to ignore bad matches.
    - Uses _last_valid_transformation as a fallback when ICP cannot compute.
    """

    global _last_valid_transformation

    # --- Normalize inputs ---
    ptsP = _normalize_icp_input(icpP)
    ptsQ = _normalize_icp_input(icpQ)

    dataSetP = {}
    dataSetQ = {}
    P_points = []
    Q_points = []

    dictQ = dict(ptsQ)
    for cid, P_data in ptsP:
        if P_data is None:
            continue
        pointsP = P_data.get('points')
        dataSetP[cid] = {
            'points': pointsP,
            'centroid': P_data.get('centroid'),
            'doppler': P_data.get('doppler_avg'),
            'hits': P_data.get('hits'),
            'missed': P_data.get('missed'),
            'history': P_data.get('history')
        }
        # Match only clusters that exist in both frames
        if cid in dictQ:
            pointsQ = dictQ[cid].get('points')
            if pointsP is not None and pointsQ is not None:
                P_points.append(pointsP)
                Q_points.append(pointsQ)

    for cid, Q_data in ptsQ:
        if Q_data is None:
            continue
        dataSetQ[cid] = {
            'points': Q_data.get('points'),
            'centroid': Q_data.get('centroid'),
            'doppler': Q_data.get('doppler_avg'),
            'hits': Q_data.get('hits'),
            'missed': Q_data.get('missed'),
            'history': Q_data.get('history')
        }

    # --- Fallback if no matching clusters ---
    # Previously returned empty dict (caused pipeline to lose transformation)
    # Now reuses last valid transformation for smoother odometry
    if not P_points or not Q_points:
        return _last_valid_transformation

    # --- Combine points ---
    P_all = np.vstack(P_points)[:, :2]
    Q_all = np.vstack(Q_points)[:, :2]

    # --- KD-tree Nearest Neighbor Matching ---
    # Original code assumed equal size arrays → caused ValueError
    # KD-tree ensures every P point has a matched Q point
    from scipy.spatial import cKDTree
    treeQ = cKDTree(Q_all)
    dists, indices = treeQ.query(P_all, k=1)

    # --- Outlier Rejection ---
    # Ignore pairs with distance > 2 meters (configurable)
    threshold = 2.0
    valid_mask = dists < threshold
    matched_P = P_all[valid_mask]
    matched_Q = Q_all[indices[valid_mask]]

    # --- Fallback if not enough matches ---
    # Ensures we don't run SVD with <2 points
    if matched_P.shape[0] < 2:
        return _last_valid_transformation

    # --- Compute SVD-based rotation ---
    centroid_P = np.mean(matched_P, axis=0)
    centroid_Q = np.mean(matched_Q, axis=0)
    P_centered = matched_P - centroid_P
    Q_centered = matched_Q - centroid_Q

    H = Q_centered.T @ P_centered
    U, _, Vt = np.linalg.svd(H)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = U @ Vt

    # --- Translation and rotation ---
    t = centroid_P - R @ centroid_Q
    rotation_angle = np.arctan2(R[1, 0], R[0, 0])

    translations = {0: np.array([t[0], t[1], 0.0])}
    rotations = {0: rotation_angle}

    # --- Store last valid transformation ---
    # Previously missing → now added for robustness
    _last_valid_transformation = {
        'dataSetP': dataSetP,
        'dataSetQ': dataSetQ,
        'translation': translations,
        'rotation': rotations
    }

    return _last_valid_transformation

def icp_rotation_vector(tx, ty):
    """
    [Inference] Returns the 2D rotation angle (radians) corresponding to translation (tx, ty).
    """
    return float(np.arctan2(ty, tx))


def icp_get_transformation_average(transformations):
    """
    Computes weighted averages for translation and rotation from ICP data.
    - Translation is weighted by the 'hits' count of each cluster.
    - Rotation uses circular statistics for robustness.
    - If no valid clusters are found, returns the last valid transformation.
    """
    global _last_valid_transformation

    translations = transformations.get('translation', {})
    rotations = transformations.get('rotation', {})
    dataSetP = transformations.get('dataSetP', {})

    if not translations and not rotations:
        # Return the last valid transformation if available
        return _last_valid_transformation
    
    # ----- Dynamic Alpha -----
    total_hits = sum((cluster.get('hits') or 0) for cluster in dataSetP.values())
    weightedAlpha = min_alpha + (max_alpha - min_alpha) * min(total_hits, max_hits) / max_hits

    # ----- Weighted Translation -----
    weighted_translations = []
    weights_t = []

    for cid, tvec in translations.items():
        if tvec is None:
            continue
        hits = dataSetP.get(cid, {}).get('hits', 1)
        if hits is None:
            hits = 1
        weight = hits ** weightedAlpha
        tvec = np.asarray(tvec, dtype=float)
        weighted_translations.append(tvec * weight)
        weights_t.append(weight)

    if weighted_translations:
        weighted_translations = np.stack(weighted_translations, axis=0)
        weights_t = np.array(weights_t, dtype=float)
        average_translations = np.sum(weighted_translations, axis=0) / np.sum(weights_t)
    else:
        average_translations = np.zeros(3)

    # ----- Weighted Circular Mean for Rotation -----
    cos_vals = []
    sin_vals = []
    weights_r = []

    for cid, angle in rotations.items():
        if angle is None:
            continue
        hits = dataSetP.get(cid, {}).get('hits', 1)
        if hits is None:
            hits = 1
        weight = hits ** weightedAlpha
        cos_vals.append(np.cos(angle) * weight)
        sin_vals.append(np.sin(angle) * weight)
        weights_r.append(weight)

    if cos_vals and sin_vals:
        cos_vals = np.array(cos_vals, dtype=float)
        sin_vals = np.array(sin_vals, dtype=float)
        weights_r = np.array(weights_r, dtype=float)
        cos_mean = np.sum(cos_vals) / np.sum(weights_r)
        sin_mean = np.sum(sin_vals) / np.sum(weights_r)
        average_rotations = np.arctan2(sin_mean, cos_mean)
    else:
        average_rotations = 0.0

    # Store the last valid transformation
    _last_valid_transformation = {
        'translation_avg': average_translations,
        'rotation_avg': average_rotations
    }

    return _last_valid_transformation

def icp_transformation_matrix(motionVectors):
    """
    Builds the homogeneous 2D transformation matrix from averaged
    translation and rotation.

    This matrix will provide the world motion.

    Args:
      avg: dict with keys 'translation_avg' (array-like [tx, ty, ...])
           and 'rotation_avg' (theta in radians).

    Returns:
      3x3 numpy array:
        [[cosθ, -sinθ, tx],
         [sinθ,  cosθ, ty],
         [0,      0,   1]]
    """
    translations = motionVectors.get('translation_avg')
    rotations = motionVectors.get('rotation_avg')
    if translations is None or rotations is None:
        return None
    tx = float(translations[0])
    ty = float(translations[1])
    cos = np.cos(rotations)
    sine = np.sin(rotations)
    transformation_icp = np.array([[cos, -sine, tx], [sine,  cos, ty], [0,  0,  1]])
    return transformation_icp

def icp_ego_motion_matrix(avg):
    """
    Returns the ego-motion (inverse ICP) matrix:
      T_ego =   [[cosθ, sinθ, -(tx*cosθ + ty*sinθ)],
                 [-sinθ,  cosθ, (tx*sinθ - ty*cosθ)],
                 [0,      0,   1]]
    """
    translations = avg.get('translation_avg')
    rotations = avg.get('rotation_avg')
    if translations is None or rotations is None:
        return None
    tx = float(translations[0])
    ty = float(translations[1])
    cos = np.cos(rotations)
    sine = np.sin(rotations)
    # rotation matrix
    R_rotation = np.array([[cos, sine], 
                   [-sine, cos]])
    R_translation = np.array([-(tx*cos + ty*sine), 
                        (tx*sine + ty*cos)])

    transformation_ego = np.array([
        [cos, sine, -(tx * cos + ty * sine)],
        [-sine, cos, (tx * sine - ty * cos)],
        [0, 0, 1]
    ])

    return transformation_ego, R_rotation, R_translation