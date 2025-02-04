import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

class CircularTrajectory(Node):
    def __init__(self):
        super().__init__('circular_trajectory')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/r_arm/target_pose', 10)
        
        # Parâmetros da trajetória/Divisão do círculo em pontos fixos 
        self.num_points = 100       # Pontos na circunferência/ O círculo é dividido em 100 pontos, quanto menor o num_points, mais "angular" será a trajetória.
        self.radius = 0.15          # 15 cm de raio
        self.center_y = -0.25       # Centro em Y (entre -0.4 e -0.1)
        self.center_z = -0.15       # Centro em Z
        self.gravity_compensation = 0.06  # 6 cm
        self.trajectory_points = self.create_circle_points()
        self.current_point = 0
        
        self.timer = self.create_timer(0.05, self.publish_next_pose)  # 20 Hz

    def create_circle_points(self):
        points = []
        theta = np.linspace(0, 2*np.pi, self.num_points)  # Ângulos igualmente espaçados
        
        for angle in theta:
            # Equações paramétricas do círculo
            y = self.center_y + self.radius * np.cos(angle)
            z = self.center_z + self.radius * np.sin(angle)
            
            points.append({
                'position': [0.3, y, z + self.gravity_compensation],
                'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()
            })
        
        return points

    def publish_next_pose(self):
        point = self.trajectory_points[self.current_point]
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'torso'
        
        msg.pose.position.x = point['position'][0]
        msg.pose.position.y = point['position'][1]
        msg.pose.position.z = point['position'][2] + 0.01  # Compensação final
        
        msg.pose.orientation.x = point['orientation'][0]
        msg.pose.orientation.y = point['orientation'][1]
        msg.pose.orientation.z = point['orientation'][2]
        msg.pose.orientation.w = point['orientation'][3]
        
        self.pose_pub.publish(msg)
        self.current_point = (self.current_point + 1) % len(self.trajectory_points)

def main(args=None):
    rclpy.init(args=args)
    node = CircularTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()