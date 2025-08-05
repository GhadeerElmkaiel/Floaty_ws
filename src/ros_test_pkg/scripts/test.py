

from scipy.spatial.transform import Rotation as R
import numpy as np
import time


def rotate_vector(x, y, z, roll, pitch, yaw, inverse=False):
    """
    Rotates a 3D vector (x, y, z) by the specified roll, pitch, and yaw angles.
    If inverse=True, it applies the inverse rotation to rotate back.
    
    Parameters:
    - x, y, z: float, the components of the vector in the original coordinate system.
    - roll: float, roll angle in radians.
    - pitch: float, pitch angle in radians.
    - yaw: float, yaw angle in radians.
    - inverse: bool, if True, applies the inverse rotation.
    
    Returns:
    - rotated_vector: numpy array, the rotated or inverse-rotated vector [x', y', z'].
    """
    # Original vector
    vector = np.array([x, y, z])

    # Create a rotation object from roll, pitch, and yaw using the 'xyz' order
    rotation = R.from_euler('xyz', [roll, pitch, yaw])

    # Apply the inverse rotation if specified
    if inverse:
        rotated_vector = rotation.inv().apply(vector)
    else:
        rotated_vector = rotation.apply(vector)

    return rotated_vector

def transform_euler_angles(original_angles, new_system_angles, sequence='xyz'):
    """
    Transforms a set of Euler angles from one coordinate system to another.
    
    Parameters:
    - original_angles: tuple/list of 3 floats, the roll, pitch, and yaw of the original rotation in radians.
    - new_system_angles: tuple/list of 3 floats, the roll, pitch, and yaw that define the new coordinate system in radians.
    - sequence: str, optional, the rotation order for Euler angles (e.g., 'xyz' or 'zyx').
    
    Returns:
    - transformed_angles: numpy array, Euler angles of the original rotation in the new coordinate system.
    """
    # Convert the original Euler angles to a rotation matrix
    R_orig = R.from_euler(sequence, original_angles)
    
    # Convert the new coordinate system's Euler angles to a rotation matrix
    R_new_basis = R.from_euler(sequence, new_system_angles)
    
    # Combine the rotations: new_basis * original
    R_transformed = R_new_basis * R_orig
    
    # Convert the result back to Euler angles
    transformed_angles = R_transformed.as_euler(sequence)
    
    return transformed_angles

yaw = np.radians(90)
roll = 0.0
pitch = 0.0

x = 1
y = 0.5
z = 2


rotated_vector = rotate_vector(x, y, z, roll, pitch, yaw, inverse=False)

print(rotated_vector)

rotated_vector = rotate_vector(x, y, z, roll, pitch, yaw, inverse=True)

print(rotated_vector)


x_val = x*np.cos(yaw) + y*np.sin(yaw)
y_val = y*np.cos(yaw) - x*np.sin(yaw)
z_val = z

print(np.array([x_val, y_val, z_val]))


# Define original Euler angles (in radians)
original_angles = (np.radians(30), np.radians(45), np.radians(60))

# Define the new coordinate system's Euler angles (in radians)
new_system_angles = (np.radians(10), np.radians(0), np.radians(0))

# Compute the transformed Euler angles in the new coordinate system
transformed_angles = transform_euler_angles(original_angles, new_system_angles, sequence='xyz')

# Convert the transformed angles back to degrees for readability
transformed_angles_degrees = np.degrees(transformed_angles)
print("Transformed Euler Angles in Degrees:", transformed_angles_degrees)