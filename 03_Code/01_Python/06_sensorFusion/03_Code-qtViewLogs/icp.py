import numpy as np
from scipy.spatial import cKDTree

# Global variable to store the last valid transformation
_last_valid_transformation = {
    'translation_avg': np.zeros(3),
    'rotation_avg': 0.0
}

# Uses hits^alpha as the weighting factor to reduce the effect of newly detected clusters with low hit counts.
weightedAlpha = 2.0
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
            'per_cluster': {
                'translations': {},
                'rotations': {}
            },
            'global': {
                'translation': (0.0, 0.0),
                'rotation': 0.0,
                'matched_points': 0,
                'centroid_displacement': [0.0, 0.0]
            }
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

    matched_points = len(indices)  # Number of matched points
    centroid_P = np.mean(P_points, axis=0)
    centroid_Q = np.mean(Q_points, axis=0)
    centroid_displacement = centroid_P - centroid_Q

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

    translations = np.array(translations)
    rotations = np.array(rotations)

    """
    Compute global averaged transformation.
    By averaging individual translations and rotations, we obtain a
    robust global transformation for the entire point cloud.
    """
    averageTx = np.mean(translations[:, 0])
    averageTy = np.mean(translations[:, 1])
    averageTheta = np.mean(rotations)

    return {
        'per_point': {
            'translations': translations,
            'rotations': rotations
        },
        'global': {
            'translation': (averageTx, averageTy),
            'rotation': averageTheta,
            'matched_points': int(matched_points),
            'centroid_displacement': centroid_displacement.tolist()
        }
    }


def icp_clusterWise_vectors(P_clusters, Q_clusters):
    """
    Cluster-wise ICP with ID-based matching.
    Matches clusters only if IDs are present in both P_clusters and Q_clusters.
    Runs point-wise ICP for each matched pair.
    """

    translations_per_cluster = {}
    rotations_per_cluster = {}
    translations = []
    rotations = []

    # Guard for empty inputs
    if not P_clusters or not Q_clusters:
        return {
            'per_cluster': {'translations': {}, 'rotations': {}},
            'global': {'translation': (0.0, 0.0), 'rotation': 0.0}
        }

    for cid in P_clusters:
        if cid not in Q_clusters:
            continue

        P_data = P_clusters[cid]
        Q_data = Q_clusters[cid]

        P_points = P_data.get('points')
        Q_points = Q_data.get('points')

        if P_points is None or len(P_points) == 0 or Q_points is None or len(Q_points) == 0:
            continue

        # Convert to [{'x':, 'y':}, ...] for compatibility with icp_pointCloudeWise_vectors
        P_list = [{'x': float(p[0]), 'y': float(p[1])} for p in P_points]
        Q_list = [{'x': float(q[0]), 'y': float(q[1])} for q in Q_points]

        # Run per-cluster ICP
        cluster_result = icp_pointCloudeWise_vectors(P_list, Q_list)
        avg_tx, avg_ty = cluster_result['global']['translation']
        avg_angle = cluster_result['global']['rotation']

        translations_per_cluster[cid] = (avg_tx, avg_ty)
        rotations_per_cluster[cid] = avg_angle
        translations.append((avg_tx, avg_ty))
        rotations.append(avg_angle)

        # Debugging info
        centroid_P = np.mean(P_points, axis=0)
        centroid_Q = np.mean(Q_points, axis=0)
        centroid_dist = np.linalg.norm(centroid_P - centroid_Q)

        """
        print(
            f"[Cluster P{cid} → Q{cid}] "
            f"centroid_dist={centroid_dist:.3f} | "
            f"nP={len(P_points)}, nQ={len(Q_points)} | "
            f"Δ=({avg_tx:.3f}, {avg_ty:.3f}) θ={avg_angle:.3f}"
        )
        """

    # Compute global average
    if translations:
        global_tx = np.mean([t[0] for t in translations])
        global_ty = np.mean([t[1] for t in translations])
        global_angle = np.mean(rotations)
    else:
        global_tx, global_ty, global_angle = 0.0, 0.0, 0.0

    return {
        'per_cluster': {'translations': translations_per_cluster,
                        'rotations': rotations_per_cluster},
        'global': {'translation': (global_tx, global_ty),
                   'rotation': global_angle}
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

    R_rotation = np.array([[cos, sine],
                           [-sine, cos]])
    R_translation = np.array([-(tx * cos + ty * sine),
                               (tx * sine + ty * cos)])

    transformation_ego = np.array([
        [cos, sine, -(tx * cos + ty * sine)],
        [-sine, cos, (tx * sine - ty * cos)],
        [0, 0, 1]
    ])

    return transformation_ego, R_rotation, R_translation
