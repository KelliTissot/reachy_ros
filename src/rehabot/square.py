import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

class SquareTrajectory(Node):
    def __init__(self):
        super().__init__('square_trajectory')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/r_arm/target_pose', 10)
        
        # Parâmetros otimizados
        self.interpolation_points = 50  # Pontos interpolados
        self.gravity_compensation = 0.06  # 3 cm
        self.trajectory_points = self.create_square_points()
        self.current_point = 0
        
        self.timer = self.create_timer(0.05, self.publish_next_pose)  # 20 Hz

    def create_square_points(self):
        base_points = [
            {'position': [0.3, -0.4, -0.3], 'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()},
            {'position': [0.3, -0.4, 0.0], 'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()},
            {'position': [0.3, -0.1, 0.0], 'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()},
            {'position': [0.3, -0.1, -0.3], 'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()}
        ]
        
        for point in base_points:
            point['position'][2] += self.gravity_compensation
        
        return self.generate_interpolated_trajectory(base_points)

    def generate_interpolated_trajectory(self, base_points):
        full_trajectory = []
        for i in range(len(base_points)):
            start = base_points[i]
            end = base_points[(i+1) % len(base_points)]
            interpolated = self.generate_interpolated_points(start, end, self.interpolation_points)
            full_trajectory.extend(interpolated)
        return full_trajectory

    def generate_interpolated_points(self, start, end, num_points):
        points = []
        for i in range(num_points):
            alpha = np.sin((i / (num_points - 1)) * np.pi/2)
            interp_pos = [
                start['position'][0] + alpha*(end['position'][0] - start['position'][0]),
                start['position'][1] + alpha*(end['position'][1] - start['position'][1]),
                start['position'][2] + alpha*(end['position'][2] - start['position'][2])
            ]
            points.append({
                'position': interp_pos,
                'orientation': start['orientation']
            })
        return points

    def publish_next_pose(self):
        point = self.trajectory_points[self.current_point]
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'torso'
        
        msg.pose.position.x = point['position'][0]
        msg.pose.position.y = point['position'][1]
        msg.pose.position.z = point['position'][2] + 0.01  # Compensação adicional
        
        msg.pose.orientation.x = point['orientation'][0]
        msg.pose.orientation.y = point['orientation'][1]
        msg.pose.orientation.z = point['orientation'][2]
        msg.pose.orientation.w = point['orientation'][3]
        
        self.pose_pub.publish(msg)
        self.current_point = (self.current_point + 1) % len(self.trajectory_points)

def main(args=None):
    rclpy.init(args=args)
    node = SquareTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()