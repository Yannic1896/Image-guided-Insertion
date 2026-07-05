import numpy as np

class QuinticTrajectoryGenerator:
    """
    Multi-segment trajectory generator using quintic polynomials
    """
    
    def __init__(self, joint_velocity_limits: np.ndarray, joint_acceleration_limits: np.ndarray, joint_jerk_limits: np.ndarray):
        """
        Initializes the generator with physical robot joint constraints

        Args:
            joint_velocity_limits (np.ndarray): Maximum allowed velocity per joint
            joint_acceleration_limits (np.ndarray): Maximum allowed acceleration per joint
            joint_jerk_limits (np.ndarray): Maximum allowed jerk per joint
        """
        self._velocity_limits = joint_velocity_limits
        self._joint_count = len(self._velocity_limits)
        self._accel_limits = joint_acceleration_limits
        self._jerk_limits = joint_jerk_limits
    
    def _movement_time(self, start_q, goal_q, safety_factor, min_duration):
        """
        Calculates minimum required segment duration based on joint limits

        Args:
            start_q (np.ndarray): Starting joint positions for segment
            goal_q (np.ndarray): Target joint positions for segment
            safety_factor (float): Multiplier scaling safety limits (0.0 to 1.0)
            min_duration (float): Absolute floor constraint for segment time in seconds

        Returns:
            float: Final duration for segment
        """

        times = []
        min_duration = 4.0
    
        for i in range(self._joint_count):

            distance = abs(goal_q[i] - start_q[i])

            # Apply safety factor to limits (% of max. values)
            safe_velocity_limit = self._velocity_limits[i] * safety_factor
            safe_accel_limit = self._accel_limits[i] * safety_factor
            safe_jerk_limit = self._jerk_limits[i] * safety_factor
        
            # Maximum dynamic values always at fixed points
            # Peak velocity when using quintic polynomials: 1.875 * mean velocity
            # Peak acceleration: 5.7735 * mean acceleration
            # Peak jerk: 60 * mean jerk

            t_v = 0.0
            t_a = 0.0
            t_j = 0.0

            if safe_velocity_limit > 0:
                t_v = 1.875 * (distance / safe_velocity_limit)

            if safe_accel_limit > 0:
                t_a = np.sqrt(5.7735 * distance / safe_accel_limit)

            if safe_jerk_limit > 0:
                t_j = np.cbrt(60* distance / safe_jerk_limit)
            
            # Segment duration satisfies strictest constraint
            times.append(max(t_v, t_a, t_j))
            
        
        # Duration as long as slowest joint takes
        T_calculated = max(times)        
        T_final = max(T_calculated, min_duration)
    
        return T_final
    
    def _compute_coefficients(self, start_q, goal_q, T, start_v, end_v, start_a, end_a):
        """
        Solves linear system to find the quintic polynomial coefficients

        Sets up boundary matrix M matching position, velocity, and 
        acceleration states at t=0 and t=T

        Args:
            start_q (np.ndarray): Joint positions at segment start
            goal_q (np.ndarray): Joint positions at segment end
            T (float): Total time duration of the segment
            start_v (np.ndarray): Joint velocities at segment start
            end_v (np.ndarray): Joint velocities at segment end
            start_a (np.ndarray): Joint accelerations at segment start
            end_a (np.ndarray): Joint accelerations at segment end

        Returns:
            np.ndarray: Matrix of coefficients (num_joints, 6)
        """

        # M * coefficients = boundary_conditions
        # matrix results from quintic polynomial and boundary conditions
        # for first point:
        # q(0) = start_q, qdot(0) = 0, qdotdot(0) = 0
        # for last point:
        # q(T) = goal_q, qdot(T) = 0, qdotdot(T) = 0
        # for intermediate points: calculate mean velocity for waypoints
        M = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
            [1.0, T, T**2, T**3, T**4, T**5],
            [0.0, 1.0, 2*T, 3*(T**2), 4*(T**3), 5*(T**4)],
            [0.0, 0.0, 2.0, 6*T, 12*(T**2), 20*(T**3)]
        ])
        coefficients = np.zeros((self._joint_count, 6))

        for i in range(self._joint_count):
            boundary_conditions = np.array([start_q[i], start_v[i], start_a[i], goal_q[i], end_v[i], end_a[i]])
            coefficients_i = np.linalg.solve(M, boundary_conditions)
            coefficients[i, :] = coefficients_i
        return coefficients
    
    def _validate_trajectory(self, trajectory, dt, safety_factor):
        """
        Check to avoid velocity & acceleration discontinuity

        Args:
            trajectory (list): Trajectory
            dt (float): Discretization time-step size
            safety_factor (float): Scaling factor for safety boundaries

        Raises:
            ValueError: If any evaluated point violates velocity or acceleration limits
        """
        max_vel = np.asarray(self._velocity_limits, dtype=float) * safety_factor
        max_acc = np.asarray(self._accel_limits, dtype=float) * safety_factor

        for idx, pt in enumerate(trajectory):
            if np.any(np.abs(pt['velocities']) > max_vel):
                raise ValueError(f"Velocity limit violation detected at step index {idx}.")
            if np.any(np.abs(pt['accelerations']) > max_acc):
                raise ValueError(f"Acceleration limit violation detected at step index {idx}.")
    
    def generate_trajectory(self, path, frequency, safety_factor, min_duration):
        """
        Generates discretized multi-segment trajectory from joint path

        Estimates continuous intermediate waypoint velocities and accelerations

        Args:
            path (np.ndarray): Array of joint waypoints
            frequency (float): Target frequency
            safety_factor (float): Allowed fraction of maximum joint limits
            min_duration (float): Minimum time for each segment

        Returns:
            list: List of dictionaries containing 'time', 'positions', 'velocities',
                  and 'accelerations' arrays per time step
        """
        dt = 1.0 / frequency
        path = np.asarray(path, dtype=float)

        n_segments = len(path) - 1
        
        # Compute duration and coefficients of all segments
        segment_times = []
        segment_coefficients = []
            
        n_segments = len(path) - 1
        
        # Segment durations
        segment_times = []
        for k in range(n_segments):
            T_seg = self._movement_time(path[k], path[k+1], safety_factor, min_duration)
            segment_times.append(T_seg)
            
        # Waypoint velocities & accelerations
        waypoint_velocities = [np.zeros_like(path[0]) for _ in range(len(path))]
        waypoint_accelerations = [np.zeros_like(path[0]) for _ in range(len(path))]
        
        # Start & end velocities & accelerations = 0
        waypoint_velocities[0] = np.zeros_like(path[0])
        waypoint_accelerations[0] = np.zeros_like(path[0])
        waypoint_velocities[n_segments] = np.zeros_like(path[0])
        waypoint_accelerations[n_segments] = np.zeros_like(path[0])
        
        # Intermediate waypoint velocities & accelerations
        for i in range(1, n_segments):
            # Previous segment mean velocities
            v_in = (path[i] - path[i-1]) / segment_times[i-1]
            # Next segment mean velocities
            v_out = (path[i+1] - path[i]) / segment_times[i]
            # Waypoint mean velocities
            mean_vel = 0.5 * (v_in + v_out)
            mean_vel = 0.5 * (v_in + v_out)
            same_direction = (v_in * v_out) >= 0.0
            waypoint_velocities[i] = np.where(same_direction, mean_vel, 0.0)

            # Previous segment mean accelerations
            a_in = (waypoint_velocities[i] - waypoint_velocities[i-1]) / segment_times[i-1]
            # Next segment mean accelerations
            a_out = (waypoint_velocities[i+1] - waypoint_velocities[i]) / segment_times[i]
            # Waypoint mean accelerations
            waypoint_accelerations[i] = 0.5 * (a_in + a_out)

        # Compute polynomial coefficients
        segment_coefficients = []
        for k in range(n_segments):
            q_start = path[k]
            q_end = path[k+1]
            T_seg = segment_times[k]
            
            v_start = waypoint_velocities[k]
            v_end = waypoint_velocities[k+1]

            a_start = waypoint_accelerations[k]
            a_end = waypoint_accelerations[k+1]
            
            coeffs_seg = self._compute_coefficients(q_start, q_end, T_seg, v_start, v_end, a_start, a_end)
            segment_coefficients.append(coeffs_seg)

        # Total movement time
        T = sum(segment_times)
        
        trajectory = []
        t = 0.0

        # Calculate joint p,v,a for each segment & timestep
        while t <= T:
            t_local = t
            current_seg = 0
            
            # Shift local time such that it is 0 at start of each segment
            while current_seg < n_segments and t_local > segment_times[current_seg]:
                t_local -= segment_times[current_seg]
                current_seg += 1
                
            if current_seg >= n_segments:
                current_seg = n_segments - 1
                t_local = segment_times[current_seg]
                
            coefficients = segment_coefficients[current_seg]

            # p,v,a for current timestep
            p_t = []
            v_t = []
            a_t = []
        
            # Loop over joints
            for i in range(self._joint_count):
                a0, a1, a2, a3, a4, a5 = coefficients[i]
            
                # p,v,a for current joint at current timestep
                p = a0 + a1*t_local + a2*(t_local**2) + a3*(t_local**3) + a4*(t_local**4) + a5*(t_local**5)
                v = a1 + 2*a2*t_local + 3*a3*(t_local**2) + 4*a4*(t_local**3) + 5*a5*(t_local**4)
                a = 2*a2 + 6*a3*t_local + 12*a4*(t_local**2) + 20*a5*(t_local**3)
            
                p_t.append(p)
                v_t.append(v)
                a_t.append(a)
            
            point_data = {
                'time': t,
                'positions': p_t,
                'velocities': v_t,
                'accelerations': a_t
            }

            trajectory.append(point_data)
        
            t += dt

        self._validate_trajectory(trajectory, dt, safety_factor)
        
        return trajectory

