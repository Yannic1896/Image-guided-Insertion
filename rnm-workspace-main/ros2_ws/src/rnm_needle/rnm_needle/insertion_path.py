import math
from geometry_msgs.msg import Pose, Point
import numpy as np
from rnm_kinematics.fk_solver import quaternion_from_rotation_matrix

class InsertionPathCalculator:
    def __init__(self):
        pass

    def rotation_with_z_along(self, needle_dir: np.ndarray) -> np.ndarray:
        z = needle_dir / np.linalg.norm(needle_dir)

        # pick an arbitrary "up" that is not parallel to z
        up = np.array([0.0, 0.0, 1.0])
        if abs(np.dot(up, z)) > 0.9:
            up = np.array([0.0, 1.0, 0.0])

        x = np.cross(up, z)
        x = x / np.linalg.norm(x)

        y = np.cross(z, x)

        R = np.column_stack((x, y, z))  # 3x3 rotation matrix
        return R

    def compute_path(self, entry: Point, target: Point, offset_z: float) -> list[Pose]:
        """Calculates an interpolated linear path from entry to target point.
        Considers the needle offset."""
        poses = []
       
        # calculation of the direction vector from entry to target
        dx = target.x - entry.x
        dy = target.y - entry.y
        dz = target.z - entry.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        if distance < 1e-6:
            return poses
        
        #dynamic step size based on distance
        step_size = 0.01  # 10 mm
        num_steps = math.ceil(distance / step_size)

        # normalized directional vector (unit vector)
        ux = dx / distance
        uy = dy / distance
        uz = dz / distance

        # calculation of the orientation: the needle should be along ux,uy,uz
        needle_dir = np.array([ux, uy, uz])
        R = self.rotation_with_z_along(needle_dir)
        q_x, q_y, q_z, q_w = quaternion_from_rotation_matrix(R)

        # calculate pre-entry pose
        pre_insertion_offset = 0.05
        pre_x = entry.x - needle_dir[0] * pre_insertion_offset
        pre_y = entry.y - needle_dir[1] * pre_insertion_offset
        pre_z = entry.z - needle_dir[2] * pre_insertion_offset

        pre_pose = Pose()
        pre_pose.position.x = pre_x - ux * offset_z
        pre_pose.position.y = pre_y - uy * offset_z
        pre_pose.position.z = pre_z - uz * offset_z
        pre_pose.orientation.x = q_x
        pre_pose.orientation.y = q_y
        pre_pose.orientation.z = q_z
        pre_pose.orientation.w = q_w

        # linear interpolation of points
        for i in range(num_steps + 1):
            alpha = i / num_steps

            tip_x = entry.x + alpha * dx
            tip_y = entry.y + alpha * dy
            tip_z = entry.z + alpha * dz

            flange_x = tip_x - ux * offset_z
            flange_y = tip_y - uy * offset_z
            flange_z = tip_z - uz * offset_z

            pose = Pose()
            pose.position.x = flange_x
            pose.position.y = flange_y
            pose.position.z = flange_z

            pose.orientation.x = q_x
            pose.orientation.y = q_y
            pose.orientation.z = q_z
            pose.orientation.w = q_w

            poses.append(pose)

        poses = [pre_pose] + poses

        return poses

    def _calculate_quaternion_to_vector(self, ux: float, uy: float, uz: float) -> tuple[float, float, float, float]:
        """Helper function to calculate a quaternion rotating the Z-axis to the direction vector."""
        # calculate the axis of rotation (cross product of Z-axis and direction vector) 
        ax = -uy
        ay = ux
        az = 0.0
        # dot product for the angle
        dot = uz

        # transforning the angle to quaternion
        # for stability using the half-way methode
        # standard quaternion: q = [ax * sin(theta/2), ay * sin(theta/2), az * sin(theta/2), cos(theta/2)]- computing cos and sin directly is comutationally expensive and leads to rounding errors 
        # half-way methode: rotation quaternion constructed by adding 2 vectors, the z-axis and the target vector
        qw = dot + 1.0

        if qw < 1e-6:
            # Singularity : direction is opposite to Z-axis
            # spinning around X-axis is a valid solution
            return 1.0, 0.0, 0.0, 0.0
        
        # Normalization of the quaternion
        norm = math.sqrt(ax * ax + ay * ay + az * az + qw * qw)
        return ax / norm, ay / norm, az / norm, qw / norm 