import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as manimation

from utils import endpoints_to_edges, angle_diff, interpolate_angle
from utils import is_in_polygon, is_intersecting


class Robot:
    """A parent class for all robots"""

    def __init__(self, limits):
        """Initialize by providing limits of each degree of freedom"""
        # Limits in each dof, each limit is defined as (lower, upper, name)
        self.limits = limits
        self.dof = len(limits)

    def forward_kinematics(self, config):
        """Compute the endpoints of the robot given a configuration
        The last endpoint would be used for visualization of the sampling
        """
        raise NotImplementedError

    def get_edges(self):
        """Return the edges of the robot for collision checking"""
        raise NotImplementedError

    def distance(self, config1, config2):
        """Compute the distance between two configurations"""
        raise NotImplementedError

    def interpolate(self, config1, config2, num):
        """Interpolate between two configurations"""
        raise NotImplementedError

    def check_collision(
        self, config1, config2, map_corners, obstacles, obstacle_edges
    ):
        """Check colliding with obstacles between two configurations
        First perform an interpolation between the two configurations,
        then check if any of the interpolated configurations hit obstacles.
       
        arguments:
            config1 - configuration 1
            config2 - configuration 2
            map_corners - corners of the map
            obstacles - list of obstacles
            obstacle_edges - list of edges of obstacles, including map edges
        
        return:
            True if colliding with obstacles between the two configurations
        """
        # Get intepolated configurations in between config1 and config2
        configs_between = self.interpolate(config1, config2)

        # check if any of these configurations hit obstacles
        for config in configs_between:
            if self.check_collision_config(
                config, map_corners, obstacles, obstacle_edges
            ):
                return True
        return False

    def check_collision_config(
        self, config, map_corners, obstacles, obstacle_edges
    ):
        """Check if a configuration is colliding with obstacles. Ensure that all  
        cases are checked. Even ones that might not be present in the given map. 
        arguments:
            config - configuration of the robot
            map_corners - corners of the map
            obstacles - list of obstacles
            obstacle_edges - list of edges of obstacles, including map edges
        
        return:
            True if colliding with obstacles
        """
        # Get the edges of the robot for collision checking
        robot_endpoint = self.forward_kinematics(config)[-1]
        robot_edges = self.get_edges(config)

        # Check if the robot endpoint is outside the map
        if not is_in_polygon(robot_endpoint, map_corners):
            return True

        # Check if the robot endpoint is inside any obstacle
        for obstacle in obstacles:
            if is_in_polygon(robot_endpoint, obstacle):
                return True

        ### YOUR CODE HERE ###

        if is_intersecting(robot_edges, obstacle_edges):
            return True

        return False


    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        """Draw the robot given a configuration on a matplotlib axis.
        This is for visualization purpose only.
        """
        raise NotImplementedError


class PointRobot(Robot):
    """2D Point robot class"""

    def __init__(self):
        """Initialize the robot with no limits in x, y (0, 1000))"""
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y")
        ])

    def forward_kinematics(self, config):
        """Simply return the configuration as the endpoint"""
        return [config]

    def get_edges(self, config):
        """Simply return an empty list"""
        return []

    def distance(self, config1, config2):
        """Euclidean distance"""
        x_diff = config1[0] - config2[0]
        y_diff = config1[1] - config2[1]
        return np.sqrt(x_diff**2 + y_diff**2)

    def interpolate(self, config1, config2, num=5):
        """Interpolate between two configurations"""
        configs_between = zip(
            np.linspace(config1[0], config2[0], num),
            np.linspace(config1[1], config2[1], num)
        )
        return configs_between

    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        ax.scatter(config[0], config[1], s=20, c=edgecolor)


class OmnidirectionalRobot(Robot):
    """Omnidirectional navigation robot class
    Its shape is defined as a rectangle with a width and a height.
    The robot could move in any direction with any angle in a 2D plane.
    """

    def __init__(self, width, height):
        """Initialize the robot with a width and height."""
        self.width = width
        self.height = height
        # Limits in each dof: (x, y, theta)
        # x, y have no limits unless bounded by the map (1000 as default)
        # theta range is (-pi, pi)
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y"),
            (-np.pi, np.pi, "r")
        ])

    def forward_kinematics(self, config):
        """Compute the 4 corner coordinates of the robot given a configuration
        Also attach the center of the robot as the last endpoint.
        The last endpoint would be used for visualization of the sampling.
        arguments:
            config: [x, y, theta] of the rectangle

        return:
            endpoints: 4 corner coordinates of the rectangle and its center
                       [corner1, corner2, corner3, corner4, center]
        """
        # Check configuration shape
        assert len(config) == 3, "Configuration should be (x, y, theta)"

        x, y, theta = config
        width_half = self.width/2
        height_half = self.height/2

        local_corners = np.array([[-width_half, -height_half],
        [width_half, -height_half],
        [width_half, height_half],
        [-width_half, height_half]])

        # Rotation matrix
        R = np.array([[np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]])

        world_corners = local_corners.dot(R.T) + np.array([x, y])

        center = np.array([x, y])
        # Add the center of the robot as the last endpoint
        endpoints = np.vstack([world_corners, center])

        # endpoints = np.zeros((5, 2))
        return endpoints

    def get_edges(self, config):
        """Compute the edges of the robot given a configuration"""
        # Get the 4 corner coordinates
        corners = self.forward_kinematics(config)[:4]
        ### YOUR CODE HERE ###
        edges = []
        num_corners = len(corners)
        for ii in range(num_corners):
            edge = (corners[ii], corners[(ii + 1) % num_corners])
            edges.append(edge)
        return edges

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]
        return:
            distance in R^2 x S^1 space
        """
        ### YOUR CODE HERE ###
        x_diff = config1[0] - config2[0]
        y_diff = config1[1] - config2[1]
        euclidean_distance = np.sqrt(x_diff**2 + y_diff**2)
        # Calculate the shortest angular difference
        theta_diff = angle_diff(config1[2], config2[2])
        # Combine the Euclidean distance and angular difference
        # You can weigh the angular difference if needed, depending on the application
        distance = euclidean_distance + theta_diff
        return distance
        
    def interpolate(self, config1, config2, num=5):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]
        return:
            list with num number of configs from linear interploation in R^2 x S^1 space
        """
        # Unpack configurations
        x1, y1, theta1 = config1
        x2, y2, theta2 = config2
        # Initialize list to store interpolated configurations
        configs = []
        for i in range(num):
            t = i / (num - 1)  # Parameter t to interpolate between the configurations
            # Interpolate x and y coordinates
            x = (1 - t) * x1 + t * x2
            y = (1 - t) * y1 + t * y2
            # Interpolating orientation in S^1 space
            delta_theta = (theta2 - theta1 + np.pi) % (2 * np.pi) - np.pi
            theta = (theta1 + t * delta_theta + np.pi) % (2 * np.pi) - np.pi
            # Append configuration to the list
            configs.append([x, y, theta])
        return configs

    def draw_robot(self, ax, config, edgecolor="b", facecolor="pink"):
        # compute corners and draw rectangle
        corners = self.forward_kinematics(config)[:4]
        polygon = Polygon(
            corners, closed=True, edgecolor=edgecolor, facecolor=facecolor
        )
        ax.add_patch(polygon)

class KinematicChain(Robot):
    """Kinematic chain robot class
    A planar robot with a fixed base and pure revolute joints.
    Each link is a line segment.
    """
    def __init__(self, link_lengths, base=[0.1, 0.1]):
        """Initialize with a list of link lengths, and a fixed base."""
        self.base = base
        self.link_lengths = link_lengths
        self.num_joints = len(link_lengths)
        # Limits in each dof
        # assume all to be (-pi, pi)
        super().__init__(limits=[
            (-np.pi, np.pi, "r") for _ in range(self.num_joints)
        ])
        
    def forward_kinematics(self, config):
        """Compute the joint coordinates given a configuration of joint angles.
        The last endpoint would be used for visualization of the sampling
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of joint coordinates.
        """
        # Initialize the starting point as the fixed base
        joint_positions = [self.base]
        curr_position = np.array(self.base)
        curr_angle = 0
        # Compute the end points of each joint based on the configuration
        for joint_angle, link_length in zip(config, self.link_lengths):
            # Update the current angle
            curr_angle += joint_angle
            # Compute the end point of the current link
            end_x = curr_position[0] + link_length * np.cos(curr_angle)
            end_y = curr_position[1] + link_length * np.sin(curr_angle)
            end_position = np.array([end_x, end_y])
            # Add the end point to the list of joint positions
            joint_positions.append(end_position)
            # Update the current position for the next iteration
            curr_position = end_position

        return joint_positions


    def get_edges(self, config):
        """Compute the link line segments of the robot given a configuration.
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of line segments representing the link line segments.
        """
        # Check configuration length
        assert (
            len(config) == self.num_joints
        ), "Configuration should match the number of joints"
        # Computing the joint positions using forward kinematics
        joint_positions = self.forward_kinematics(config)
        # Computing the link line segments
        edges = []
        for i in range(len(joint_positions) - 1):
            edge = (joint_positions[i], joint_positions[i + 1])
            edges.append(edge)
        return edges

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            A Euclidean distance in S^1 x S^1 x ... x S^1 space
        """
        assert len(config1) == len(config2), "Configurations must have the same length"
        # Calculate the squared angular distance for each pair of corresponding joints
        squared_distances = [(angle1 - angle2) ** 2 for angle1, angle2 in zip(config1, config2)]
        # Take the square root of the sum of squared distances to get the Euclidean distance
        distance = np.sqrt(sum(squared_distances))
        return distance

    def interpolate(self, config1, config2, num=10):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            A Euclidean distance in 
            list with num number of configs from linear interploation in S^1 x S^1 x ... x S^1 space.
        """
        assert len(config1) == len(config2), "Configurations must have the same length"
    
        # Normalize the angles to be in the range [0, 2*pi)
        config1 = [angle % (2 * np.pi) for angle in config1]
        config2 = [angle % (2 * np.pi) for angle in config2]

        interpolated_configs = []
        for ii in range(num):
            t = ii / (num - 1)  # Normalized interpolation parameter
            interpolated_config = []
            for angle1, angle2 in zip(config1, config2):
                # Compute the difference in angles, and adjust for circular interpolation
                delta_angle = (angle2 - angle1 + np.pi) % (2 * np.pi) - np.pi
                # Interpolate and wrap the angle back to the range [0, 2*pi)
                new_angle = (angle1 + t * delta_angle) % (2 * np.pi)
                interpolated_config.append(new_angle)
            interpolated_configs.append(interpolated_config)
        
        return interpolated_configs
    
    def draw_robot(self, ax, config, edgecolor="b", facecolor="black"):
        # compute joint positions and draw lines
        positions = self.forward_kinematics(config)
        # Draw lines between each joint
        for i in range(len(positions) - 1):
            line = np.array([positions[i], positions[i + 1]])
            ax.plot(line[:, 0], line[:, 1], color=edgecolor)
        # Draw joint
        for i in range(len(positions)):
            ax.scatter(positions[i][0], positions[i][1], s=5, c=facecolor)
