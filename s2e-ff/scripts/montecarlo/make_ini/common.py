import numpy as np

def rotation_matrix(angle_deg, axis) -> np.ndarray:
    """
    Generate a 3D rotation matrix for a given angle and axis.
    
    Parameters:
        angle_deg (float): The rotation angle in degrees.
        axis (array-like): The rotation axis (must be a 3-element list or array).
    
    Returns:
        ndarray: A 3x3 rotation matrix.
    """
    # Convert angle from degrees to radians
    angle_rad = np.radians(angle_deg)
    
    # Normalize the axis vector
    axis = np.array(axis)
    axis = axis / np.linalg.norm(axis)
    
    # Rodrigues' rotation formula components
    cos_theta = np.cos(angle_rad)
    sin_theta = np.sin(angle_rad)
    one_minus_cos = 1 - cos_theta
    
    # Cross-product matrix K
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    
    # Rotation matrix using Rodrigues' formula
    R = np.eye(3) * cos_theta + one_minus_cos * np.outer(axis, axis) + sin_theta * K
    
    return R

def random_axis_vector():
    # θは0からπの間、φは0から2πの間のランダムな値
    theta = np.random.uniform(0, np.pi)
    phi = np.random.uniform(0, 2 * np.pi)
    
    # 球面座標からデカルト座標への変換
    x = np.sin(theta) * np.cos(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(theta)
    
    # ベクトルを生成
    axis = np.array([x, y, z])
    
    # 正規化（単位ベクトル化）
    axis_normalized = axis / np.linalg.norm(axis)
    
    return axis_normalized