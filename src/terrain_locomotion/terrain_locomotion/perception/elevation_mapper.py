# File: ~/terrain_locomotion_ws/src/terrain_locomotion/terrain_locomotion/perception/elevation_mapper.py

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import TransformException
import sensor_msgs_py.point_cloud2 as pc2

class ElevationMapper(Node):
    def __init__(self):
        super().__init__('elevation_mapper')
        
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('map_size', 200)  # Grid size (200x200)
        self.declare_parameter('resolution', 0.05)  # 5cm per cell
        self.declare_parameter('max_range', 5.0)  # Maximum depth range
        self.declare_parameter('min_range', 0.1)  # Minimum depth range
        
        map_size = self.get_parameter('map_size').value
        self.resolution = self.get_parameter('resolution').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        
        # Initialize elevation grid
        self.elevation_grid = np.full((map_size, map_size), np.nan)
        self.confidence_grid = np.zeros((map_size, map_size))
        
        # Grid origin (center of the grid)
        self.grid_origin = map_size // 2
        
        # Camera parameters (adjust these based on your camera)
        self.fx = 525.0  # Focal length x
        self.fy = 525.0  # Focal length y
        self.cx = 320.0  # Principal point x
        self.cy = 240.0  # Principal point y
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Publishers
        self.elevation_map_pub = self.create_publisher(
            Image,
            '/terrain/elevation_map',
            10
        )
        
        self.point_cloud_pub = self.create_publisher(
            PointCloud2,
            '/terrain/point_cloud',
            10
        )
        
        # Timer for publishing elevation map
        self.map_timer = self.create_timer(0.2, self.publish_elevation_map)  # 5Hz
        
        self.get_logger().info('Elevation Mapper initialized')
    
    def depth_callback(self, depth_msg):
        """Process depth image and update elevation map"""
        try:
            # Convert depth image
            if depth_msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
            elif depth_msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
                depth_image = depth_image.astype(np.float32) / 1000.0  # Convert mm to m
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {depth_msg.encoding}')
                return
            
            # Get robot transform
            try:
                robot_transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    depth_msg.header.frame_id,
                    depth_msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.1)
                )
            except TransformException as ex:
                self.get_logger().warn(f'Could not get transform: {ex}')
                # Use identity transform as fallback
                robot_transform = TransformStamped()
                robot_transform.transform.translation.x = 0.0
                robot_transform.transform.translation.y = 0.0  
                robot_transform.transform.translation.z = 0.5  # Assume camera 0.5m above ground
                robot_transform.transform.rotation.w = 1.0
            
            # Process depth image
            self.update_elevation_map(depth_image, robot_transform)
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')
    
    def update_elevation_map(self, depth_image, transform):
        """Update elevation map from depth image"""
        height, width = depth_image.shape
        
        # Get robot position
        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        robot_z = transform.transform.translation.z
        
        # Convert depth image to 3D points
        points = []
        
        for v in range(0, height, 4):  # Subsample for performance
            for u in range(0, width, 4):
                depth = depth_image[v, u]
                
                # Skip invalid depths
                if not (self.min_range < depth < self.max_range):
                    continue
                
                # Convert pixel to 3D point (camera frame)
                x_cam = (u - self.cx) * depth / self.fx
                y_cam = (v - self.cy) * depth / self.fy
                z_cam = depth
                
                # Transform to world frame (simplified - assumes camera looking down)
                x_world = robot_x + x_cam
                y_world = robot_y + y_cam  
                z_world = robot_z - z_cam  # Assuming camera points down
                
                points.append([x_world, y_world, z_world])
                
                # Update elevation grid
                self.add_point_to_grid(x_world, y_world, z_world)
        
        # Publish point cloud for visualization
        if points:
            self.publish_point_cloud(points)
    
    def add_point_to_grid(self, x, y, z):
        """Add a 3D point to the elevation grid"""
        # Convert world coordinates to grid indices
        grid_x = int((x / self.resolution) + self.grid_origin)
        grid_y = int((y / self.resolution) + self.grid_origin)
        
        # Check if point is within grid bounds
        if (0 <= grid_x < self.elevation_grid.shape[0] and 
            0 <= grid_y < self.elevation_grid.shape[1]):
            
            # Update elevation (take maximum height)
            if np.isnan(self.elevation_grid[grid_x, grid_y]) or z > self.elevation_grid[grid_x, grid_y]:
                self.elevation_grid[grid_x, grid_y] = z
            
            # Update confidence
            self.confidence_grid[grid_x, grid_y] += 1
    
    def publish_elevation_map(self):
        """Publish elevation map as image"""
        try:
            # Create visualization image
            vis_map = self.create_elevation_visualization()
            
            # Convert to ROS message
            map_msg = self.bridge.cv2_to_imgmsg(vis_map, 'bgr8')
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'base_link'
            
            self.elevation_map_pub.publish(map_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing elevation map: {str(e)}')
    
    def create_elevation_visualization(self):
        """Create colored elevation map for visualization"""
        # Handle NaN values
        vis_grid = self.elevation_grid.copy()
        valid_mask = ~np.isnan(vis_grid)
        
        if not valid_mask.any():
            # Return empty map if no valid points
            return np.zeros((vis_grid.shape[0], vis_grid.shape[1], 3), dtype=np.uint8)
        
        # Normalize elevation values
        min_elevation = np.nanmin(vis_grid)
        max_elevation = np.nanmax(vis_grid)
        
        if max_elevation > min_elevation:
            normalized = (vis_grid - min_elevation) / (max_elevation - min_elevation)
        else:
            normalized = np.zeros_like(vis_grid)
        
        # Create color map (blue = low, red = high)
        colored_map = np.zeros((vis_grid.shape[0], vis_grid.shape[1], 3), dtype=np.uint8)
        
        # Apply colormap where we have valid data
        colored_map[valid_mask, 0] = (normalized[valid_mask] * 255).astype(np.uint8)  # Red channel
        colored_map[valid_mask, 1] = ((1 - normalized[valid_mask]) * 128).astype(np.uint8)  # Green channel  
        colored_map[valid_mask, 2] = ((1 - normalized[valid_mask]) * 255).astype(np.uint8)  # Blue channel
        
        # Mark unknown areas as gray
        colored_map[~valid_mask] = [128, 128, 128]
        
        # Add robot position marker
        robot_grid_x = self.grid_origin
        robot_grid_y = self.grid_origin
        cv2.circle(colored_map, (robot_grid_y, robot_grid_x), 5, (0, 255, 0), -1)
        
        return colored_map
    
    def publish_point_cloud(self, points):
        """Publish 3D points as point cloud"""
        try:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'base_link'
            
            # Convert points to PointCloud2 message
            point_cloud_msg = pc2.create_cloud_xyz32(header, points)
            self.point_cloud_pub.publish(point_cloud_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing point cloud: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ElevationMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()