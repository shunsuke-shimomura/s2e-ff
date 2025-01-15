
import pathlib
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

def echo():
    with open(pathlib.Path(__file__).parent.parent.parent.parent/"data/initialize_files/components/relative_position_sensor.ini", 'w') as f:
        print(
"""[RELATIVE_POSITION_SENSOR]
// Prescaler with respect to the component update period
prescaler = 1

// Target satellite ID
target_sat_id = 1

// When this value is negative, reference_sat_id is automatically set as the mounting satellite ID
reference_sat_id = -1

// Users can choose the frame for error settings
// INERTIAL, RTN, or BODY
error_frame = BODY

[SENSOR_BASE_RELATIVE_POSITION_SENSOR]
// The coordinate of the error is selected by the error_frame
// Scale factor [-]
scale_factor_c(0) = {0};
scale_factor_c(1) = {1};
scale_factor_c(2) = {2};
scale_factor_c(3) = {3};
scale_factor_c(4) = {4};
scale_factor_c(5) = {5};
scale_factor_c(6) = {6};
scale_factor_c(7) = {7};
scale_factor_c(8) = {8};

// Constant bias noise [m]
constant_bias_c_m(0) = 0.0
constant_bias_c_m(1) = 0.0
constant_bias_c_m(2) = 0.0

// Standard deviation of normal random noise [m]
normal_random_standard_deviation_c_m(0) = 0.1
normal_random_standard_deviation_c_m(1) = 0.1
normal_random_standard_deviation_c_m(2) = 0.1

// Standard deviation for random walk noise [m]
random_walk_standard_deviation_c_m(0) = 0.0
random_walk_standard_deviation_c_m(1) = 0.0
random_walk_standard_deviation_c_m(2) = 0.0

// Limit of random walk noise [m]
random_walk_limit_c_m(0) = 0.2
random_walk_limit_c_m(1) = 0.2
random_walk_limit_c_m(2) = 0.2

// Range [m]
range_to_constant_m = 1000000.0  // smaller than range_to_zero_m
range_to_zero_m = 10000000.0
"""
        .format(
            *rotation_matrix(1,random_axis_vector()).flatten()
        ),
        file=f)


if __name__ == "__main__":
    print(*rotation_matrix(1,random_axis_vector()).flatten())
    echo()