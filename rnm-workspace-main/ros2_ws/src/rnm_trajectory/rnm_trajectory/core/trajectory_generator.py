""" Trajectory Generator class using quintic polynomials"""
""" To do: clean up, return message could be float64MultiArray directly"""
""" Supports path with several segments, not needed if we do not use a pathplanner"""

import numpy as np

class QuinticTrajectoryGenerator:
    
    def __init__(self, joint_velocity_limits: np.ndarray, joint_acceleration_limits: np.ndarray, joint_jerk_limits: np.ndarray):
        """
            Args: Joint velocity limits
        """
        self._velocity_limits = joint_velocity_limits
        self._joint_count = len(self._velocity_limits)
        self._accel_limits = joint_acceleration_limits
        self._jerk_limits = joint_jerk_limits
    
    def _movement_time(self, start_q, goal_q, safety_factor):
        times = []
    
        for i in range(self._joint_count):

            distance = abs(goal_q[i] - start_q[i])

            # apply safety factor to limits (5% of max. values)
            safe_velocity_limit = self._velocity_limits[i] * safety_factor
            safe_accel_limit = self._accel_limits[i] * safety_factor
            safe_jerk_limit = self._jerk_limits[i] * safety_factor
        
            # maximum dynamic values always at fixed points
            # peak velocity when using quintic polynomials: 1.875 * mean velocity
            # peak acceleration: 5.7735 * mean acceleration
            # peak jerk: 60 * mean jerk

            t_v = 0.0
            t_a = 0.0
            t_j = 0.0

            if safe_velocity_limit > 0:
                t_v = 1.875 * (distance / safe_velocity_limit)

            if safe_accel_limit > 0:
                t_a = np.sqrt(5.7735 * distance / safe_accel_limit)

            if safe_jerk_limit > 0:
                t_j = np.cbrt(60* distance / safe_jerk_limit)
            
            # segment duration satisfies strictest constraint
            times.append(max(t_v, t_a, t_j))
        
        # duration as long as slowest joint takes (at least 1)
        T_calculated = max(times)
        T_final = max(T_calculated, 1.0)
    
        return T_final
    
    def _compute_coefficients(self,start_q, goal_q, T):
        # M * coefficients = boundary_conditions
        # matrix results from quintic polynomial and boundary conditions
        # q(0) = start_q, qdot(0) = 0, qdotdot(0) = 0
        # q(T) = goal_q, qdot(T) = 0, qdotdot(T) = 0
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
            boundary_conditions = np.array([start_q[i], 0.0, 0.0, goal_q[i], 0.0, 0.0])
            coefficients_i = np.linalg.solve(M, boundary_conditions)
            coefficients[i, :] = coefficients_i
        return coefficients
    
    def generate_trajectory(self, path, frequency, safety_factor):
        dt = 1.0 / frequency

        n_segments = len(path) - 1
        
        # compute duration and coefficients of all segments
        segment_times = []
        segment_coefficients = []
        
        for k in range(n_segments):
            q_start = path[k]
            q_end = path[k+1]

            T_seg = self._movement_time(q_start, q_end, safety_factor)
            segment_times.append(T_seg)
            
            coeffs_seg = self._compute_coefficients(q_start, q_end, T_seg)
            segment_coefficients.append(coeffs_seg)

        # total movement time
        T = sum(segment_times)
        
        trajectory = []
        t = 0.0

        while t <= T:
            t_local = t
            current_seg = 0
            
            # shift local time such that it is 0 at start of each segment
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
        
            # loop over joints
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
        
        return trajectory

