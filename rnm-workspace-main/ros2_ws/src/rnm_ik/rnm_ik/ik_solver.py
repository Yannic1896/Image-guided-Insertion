
import numpy as np
from typing import Optional, Tuple


class IKSolver:
    """
    Base class for Inverse Kinematic solvers.
    """

    def __init__(self, dh_params: Optional[np.ndarray] = None):
        """
        
        Args:
            dh_params: Denavit-Hartenberg parameters for the robot
        """
        self.dh_params = dh_params

    def solve(self, target_pose: np.ndarray) -> Optional[np.ndarray]:
        """
        Solve inverse kinematics for target end-effector pose.
        
        Args:
            target_pose: Target pose (position and orientation)
            
        Returns:
            Joint angles if solution found, None otherwise
        """
        raise NotImplementedError("Subclass must implement solve method")
