import numpy as np

# Global variable to store the last valid transformation
_last_valid_transformation = {
    'translation_avg': np.zeros(3),
    'rotation_avg': 0.0
}

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
    # Normalize inputs
    ptsP = _normalize_icp_input(icpP)
    ptsQ = _normalize_icp_input(icpQ)

    # gather raw cluster data into dictionaries
    dataSetP = {}
    dataSetQ = {}
    for cid, P_data in ptsP:
        if P_data is None:
            continue
        dataSetP[cid] = {
            'points':   P_data.get('points'),
            'centroid': P_data.get('centroid'),
            'doppler':  P_data.get('doppler_avg'),
            'hits':     P_data.get('hits'),
            'missed':   P_data.get('missed'),
            'history':  P_data.get('history')
        }
    for cid, Q_data in ptsQ:
        if Q_data is None:
            continue
        dataSetQ[cid] = {
            'points':   Q_data.get('points'),
            'centroid': Q_data.get('centroid'),
            'doppler':  Q_data.get('doppler_avg'),
            'hits':     Q_data.get('hits'),
            'missed':   Q_data.get('missed'),
            'history':  Q_data.get('history')
        }

    # compute per-cluster translations and rotations
    translations = {}
    rotations    = {}
    for cid in dataSetP:
        if cid not in dataSetQ:
            continue
        P_cent = dataSetP[cid]['centroid']
        Q_cent = dataSetQ[cid]['centroid']
        # Skip clusters without valid centroids
        if P_cent is None or Q_cent is None:
            continue
        P_cent = np.asarray(P_cent)
        Q_cent = np.asarray(Q_cent)
        t_vec  = P_cent - Q_cent
        translations[cid] = t_vec

        # infer 2D rotation angle from the translation vector
        tx, ty = t_vec[0], t_vec[1]
        rotations[cid] = icp_rotation_vector(tx, ty)

    return {
        'dataSetP':    dataSetP,
        'dataSetQ':    dataSetQ,
        'translation': translations,
        'rotation':    rotations
    }


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

    # ----- Weighted Translation -----
    weighted_translations = []
    weights_t = []

    for cid, tvec in translations.items():
        if tvec is None:
            continue
        hits = dataSetP.get(cid, {}).get('hits', 1)
        if hits is None:
            hits = 1
        tvec = np.asarray(tvec, dtype=float)
        weighted_translations.append(tvec * hits)
        weights_t.append(hits)

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
        cos_vals.append(np.cos(angle) * hits)
        sin_vals.append(np.sin(angle) * hits)
        weights_r.append(hits)

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
      T_ego = [[R^T, -R^T * t], [0,0,1]]
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
    R = np.array([[cos, -sine], [sine, cos]])
    Rt = R.T
    t_ego = -Rt.dot(np.array([tx, ty]))

    transformation_ego = np.array([[Rt[0,0], Rt[0,1], t_ego[0]], [Rt[1,0], Rt[1,1], t_ego[1]], [0, 0, 1]])
    
    return transformation_ego