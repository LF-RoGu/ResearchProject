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

def icp_pointCloudeWise_vectors(P, Q):
    """
    Pairwise ICP for 2D radar odometry with KD-tree matching.
    Computes per-point translation and rotation vectors and a global averaged transformation.

    Changes made:
    - Added KD-tree nearest-neighbor matching to handle unequal point counts.
    - Computes translation vector for each matched point (P_i - Q_i).
    - Computes rotation for each match as atan2(ty, tx).
    - Returns both per-point results and a global averaged translation and rotation.
    """

    if P is None or Q is None or len(P) == 0 or len(Q) == 0:
        return {
            'per_point': {'translations': [], 'rotations': []},
            'global': {'translation': (0.0, 0.0), 'rotation': 0.0}
        }
    else:
        P_points = np.array([[p['x'], p['y']] for p in P])
        Q_points = np.array([[q['x'], q['y']] for q in Q])

    # Return empty if no points found in either frame
    if P_points.shape[0] < 1 or Q_points.shape[0] < 1:
        return []

    """
    KD-tree Nearest Neighbor Matching.
    A KD-tree is a spatial data structure that allows efficient nearest-neighbor searches.
    - We build a KD-tree from the 'Q' point set (target frame).
    - For each point in 'P' (source frame), we find the closest point in 'Q'.
    """
    treeQ = cKDTree(Q_points)
    # Find nearest neighbor for each P
    # dists: distance from each P point to its closest Q point
    # indices: indices of the matched Q points
    dists, indices = treeQ.query(P_points, k=1)

    """
    Translation and rotation computation for each point.
    For each matched pair:
    - Translation: tx, ty = P_i - Q_j
    - Rotation: atan2(ty, tx)
    This computes motion estimates for every matched point individually.
    """
    translations = []
    rotations = []

    for i, idx in enumerate(indices):
        tx, ty = P_points[i] - Q_points[idx]
        translations.append((tx, ty))
        rotations.append(np.arctan2(ty, tx))

    """
    Compute global averaged transformation.
    By averaging individual translations and rotations, we obtain a
    robust global transformation for the entire point cloud.
    """
    averageTx = np.mean([t[0] for t in translations])
    averageTy = np.mean([t[1] for t in translations])
    averageTheta = np.mean(rotations)

    return {
        'per_point': {
            'translations': translations,
            'rotations': rotations
        },
        'global': {
            'translation': (averageTx, averageTy),
            'rotation': averageTheta
        }
    }

def icp_clusterWise_vectors(P_clusters, Q_clusters):
    """
    Cluster-wise ICP for 2D radar odometry.
    Uses icp_pointCloudeWise_vectors for each cluster individually.
    Computes translation and rotation per cluster and a global average.
    """

    translations_per_cluster = {}
    rotations_per_cluster = {}

    all_translations = []
    all_rotations = []

    for cid, P_data in P_clusters.items():
        if cid not in Q_clusters:
            continue

        # Prepare inputs for the per-point ICP
        P_points = [{'x': p[0], 'y': p[1]} for p in P_data['points']]
        Q_points = [{'x': q[0], 'y': q[1]} for q in Q_clusters[cid]['points']]

        # Reuse the pairwise ICP function
        cluster_result = icp_pointCloudeWise_vectors(P_points, Q_points)

        avg_tx, avg_ty = cluster_result['global']['translation']
        avg_angle = cluster_result['global']['rotation']

        translations_per_cluster[cid] = (avg_tx, avg_ty)
        rotations_per_cluster[cid] = avg_angle

        all_translations.append((avg_tx, avg_ty))
        all_rotations.append(avg_angle)

    if all_translations:
        global_tx = np.mean([t[0] for t in all_translations])
        global_ty = np.mean([t[1] for t in all_translations])
        global_angle = np.mean(all_rotations)
    else:
        global_tx, global_ty, global_angle = 0.0, 0.0, 0.0

    return {
        'per_cluster': {
            'translations': translations_per_cluster,
            'rotations': rotations_per_cluster
        },
        'global': {
            'translation': (global_tx, global_ty),
            'rotation': global_angle
        }
    }

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