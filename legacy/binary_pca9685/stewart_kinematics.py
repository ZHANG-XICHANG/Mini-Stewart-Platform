"""Shared geometry solver extracted unchanged from the original visualizer.

Preserves the existing rotation convention; physical pose accuracy still requires calibration.
"""
import numpy as np
from math import pi, sin, cos, asin, atan2, sqrt, radians, degrees
from stewart_config import (BASE_RADIUS, PLATFORM_RADIUS, SERVO_ARM_LENGTH,
    CONNECTING_ARM_LENGTH, PLATFORM_HEIGHT, THETA_S, THETA_R, THETA_P)

class StewartKinematics:
    def __init__(self):
        self.RD = BASE_RADIUS
        self.PD = PLATFORM_RADIUS
        self.L1 = SERVO_ARM_LENGTH
        self.L2 = CONNECTING_ARM_LENGTH
        self.platform_height = PLATFORM_HEIGHT
        self.theta_s = np.array(THETA_S)
        self.theta_r = THETA_R
        self.theta_p = THETA_P
        self.DxMultiplier = np.array([1, 1, 1, -1, -1, -1])
        self.AngleMultiplier = np.array([1, -1, 1, 1, -1, 1])
        self.OffsetAngle = np.array([pi/6, pi/6, -pi/2, -pi/2, pi/6, pi/6])
        self.motion_limits = {'max_z': 0, 'min_z': 0, 'max_angle': 0}
        self.calculate_geometry()

    def calculate_geometry(self):
        """Calculate platform geometry using the same method as the C code."""
        # Initialize arrays for platform and base coordinates
        self.platform_coords_x = np.zeros(6)
        self.platform_coords_y = np.zeros(6)
        self.base_coords_x = np.zeros(6)
        self.base_coords_y = np.zeros(6)
        
        # Calculate platform and base coordinates using the algorithm from getAlpha()
        for i in range(6):
            # Platform coordinates calculation
            platform_pd_x = self.DxMultiplier[i] * self.RD
            platform_pd_y = self.RD
            platform_angle = self.OffsetAngle[i] + self.AngleMultiplier[i] * radians(self.theta_r)
            self.platform_coords_x[i] = platform_pd_x * cos(platform_angle)
            self.platform_coords_y[i] = platform_pd_y * sin(platform_angle)
            
            # Base coordinates calculation
            base_pd_x = self.DxMultiplier[i] * self.PD
            base_pd_y = self.PD
            base_angle = self.OffsetAngle[i] + self.AngleMultiplier[i] * radians(self.theta_p)
            self.base_coords_x[i] = base_pd_x * cos(base_angle)
            self.base_coords_y[i] = base_pd_y * sin(base_angle)
        
        # Store the coordinates as points
        self.base_points = np.column_stack((self.base_coords_x, self.base_coords_y, np.zeros(6)))
        self.home_platform_points = np.column_stack((
            self.platform_coords_x, 
            self.platform_coords_y, 
            np.ones(6) * self.platform_height
        ))
        
        # Calculate motor shaft vectors using servo angles
        self.servo_vectors = []
        for i in range(6):
            # Servo angles from THETA_S (degrees to radians)
            theta = radians(self.theta_s[i])
            
            # Unit vector in the direction of the servo angle
            vx = cos(theta)
            vy = sin(theta)
            vz = 0
            self.servo_vectors.append([vx, vy, vz])
        self.servo_vectors = np.array(self.servo_vectors)


    def calculate_servo_angles(self, position, rotation):
        """
        Calculate servo angles using the inverse kinematics from C code.
        
        Args:
            position (array): [x, y, z] position offset in mm
            rotation (array): [roll, pitch, yaw] rotation in radians
            
        Returns:
            tuple: (servo_angles, transformed_platform_points, servo_arm_ends) or None if invalid
        """
        # Reset constraint info
        self.constraint_info = {
            'active': False,
            'type': None,
            'servo_index': -1,
            'value': 0,
            'limit': 0
        }
        
        # Create arrays for storing calculated values
        platform_pivot_x = np.zeros(6)
        platform_pivot_y = np.zeros(6)
        platform_pivot_z = np.zeros(6)
        delta_Lx = np.zeros(6)
        delta_Ly = np.zeros(6)
        delta_Lz = np.zeros(6)
        delta_L2_virtual = np.zeros(6)
        l_values = np.zeros(6)
        m_values = np.zeros(6)
        n_values = np.zeros(6)
        alpha_values = np.zeros(6)
        
        # Rotation order from C code: roll(x), pitch(y), yaw(z)
        # Unpack rotation values
        roll, pitch, yaw = rotation
        
        try:
            for i in range(6):
                # Transform platform coordinates based on position and rotation
                # Formula directly from getAlpha() in helpers.cpp
                platform_pivot_x[i] = (self.platform_coords_x[i] * cos(roll) * cos(yaw) + 
                                    self.platform_coords_y[i] * (sin(pitch) * sin(roll) * cos(yaw) - cos(pitch) * sin(yaw)) + 
                                    position[0])
                
                platform_pivot_y[i] = (self.platform_coords_x[i] * cos(pitch) * sin(yaw) + 
                                    self.platform_coords_y[i] * (cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)) + 
                                    position[1])
                
                platform_pivot_z[i] = (-self.platform_coords_x[i] * sin(roll) + 
                                    self.platform_coords_y[i] * sin(pitch) * cos(roll) + 
                                    self.platform_height + position[2])
                
                # Calculate leg vectors
                delta_Lx[i] = self.base_coords_x[i] - platform_pivot_x[i]
                delta_Ly[i] = self.base_coords_y[i] - platform_pivot_y[i]
                delta_Lz[i] = -platform_pivot_z[i]
                
                # Calculate virtual leg length
                delta_L2_virtual[i] = sqrt(delta_Lx[i]**2 + delta_Ly[i]**2 + delta_Lz[i]**2)
                
                # Check if connecting rod would need to stretch beyond L2 length
                if abs(delta_L2_virtual[i] - self.L1) > self.L2:
                    self.constraint_info = {
                        'active': True,
                        'type': 'length',
                        'servo_index': i,
                        'value': abs(delta_L2_virtual[i] - self.L1),
                        'limit': self.L2
                    }
                    return None
                
                # Calculate intermediate values for servo angle calculation
                l_values[i] = delta_L2_virtual[i]**2 - (self.L2**2 - self.L1**2)
                m_values[i] = 2 * self.L1 * platform_pivot_z[i]
                n_values[i] = 2 * self.L1 * (cos(radians(self.theta_s[i])) * (platform_pivot_x[i] - self.base_coords_x[i]) + 
                                          sin(radians(self.theta_s[i])) * (platform_pivot_y[i] - self.base_coords_y[i]))
                
                # Check if we'll get a valid solution (real numbers)
                discriminant = m_values[i]**2 + n_values[i]**2
                if discriminant <= 0:
                    self.constraint_info = {
                        'active': True,
                        'type': 'math',
                        'servo_index': i,
                        'value': discriminant,
                        'limit': 0
                    }
                    return None
                
                check_val = l_values[i] / sqrt(discriminant)
                if abs(check_val) > 1:  # asin domain error
                    self.constraint_info = {
                        'active': True,
                        'type': 'math',
                        'servo_index': i,
                        'value': abs(check_val),
                        'limit': 1
                    }
                    return None
                
                # Calculate servo angle using the formula from getAlpha()
                alpha_values[i] = asin(check_val) - atan2(n_values[i], m_values[i])
                
                # Check if angle is within limits
                if alpha_values[i] < radians(-60):
                    self.constraint_info = {
                        'active': True,
                        'type': 'angle',
                        'servo_index': i,
                        'value': degrees(alpha_values[i]),
                        'limit': -60
                    }
                    return None
                elif alpha_values[i] > radians(60):
                    self.constraint_info = {
                        'active': True,
                        'type': 'angle',
                        'servo_index': i,
                        'value': degrees(alpha_values[i]),
                        'limit': 60
                    }
                    return None
            
            # Calculate servo arm endpoints
            servo_arm_ends = []
            for i in range(6):
                # Get base point
                base_point = self.base_points[i]
                
                # Get servo angle
                alpha = alpha_values[i]
                
                # Calculate swing arm endpoint
                servo_angle_rad = radians(self.theta_s[i])
                
                # Direction vector of the swing arm
                arm_dir_x = cos(servo_angle_rad) * cos(alpha)
                arm_dir_y = sin(servo_angle_rad) * cos(alpha)
                arm_dir_z = sin(alpha)
                
                # Calculate the endpoint
                endpoint = base_point + self.L1 * np.array([arm_dir_x, arm_dir_y, arm_dir_z])
                servo_arm_ends.append(endpoint)
            
            # Create array of transformed platform points
            platform_points = np.column_stack((platform_pivot_x, platform_pivot_y, platform_pivot_z))
            
            # Calculate motion limits
            self.check_motion_limits(alpha_values, platform_points, servo_arm_ends)
            
            return alpha_values, platform_points, np.array(servo_arm_ends)
            
        except (ValueError, ZeroDivisionError, RuntimeWarning) as e:
            # If no specific constraint was detected but we still got an error
            if not self.constraint_info['active']:
                self.constraint_info = {
                    'active': True,
                    'type': 'math',
                    'servo_index': -1,
                    'value': 0,
                    'limit': 0
                }
            return None


    def check_motion_limits(self, alpha_values, platform_points, servo_arm_ends):
        """Check and record motion limits"""
        # Update maximum height reached
        z_values = platform_points[:, 2]
        if np.max(z_values) > self.motion_limits['max_z']:
            self.motion_limits['max_z'] = np.max(z_values)
        if np.min(z_values) < self.motion_limits['min_z']:
            self.motion_limits['min_z'] = np.min(z_values)
            
        # Update maximum angle reached
        max_angle = np.max(np.abs(alpha_values)) * 180/pi
        if max_angle > self.motion_limits['max_angle']:
            self.motion_limits['max_angle'] = max_angle
