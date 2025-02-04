import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

class PhysioArmMovement(Node):
    def __init__(self):
        super().__init__('physio_arm_movement')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/r_arm/target_pose', 10)
        
        # Parâmetros de movimento
        self.interpolation_points = 40
        self.dwell_time = 2.0  # Segundos em cada posição-chave
        self.gravity_comp = 0.03  # 3 cm
        
        self.trajectory = self.create_physio_trajectory()
        self.current_point = 0
        
        self.timer = self.create_timer(0.05, self.publish_pose)  # 20 Hz

    def create_physio_trajectory(self):
        """Trajetória de abdução + alcance frontal"""
        key_positions = [
            # Posição neutra (braço ao lado do corpo)
            {'position': [0.0, -0.3, -0.2], 'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()},
            
            # Abdução de 90° (braço lateral)
            {'position': [0.0, -0.6, 0.2], 'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()},
            
            # Alcance frontal (braço à frente)
            {'position': [0.4, -0.3, 0.0], 'orientation': Rotation.from_euler('xyz', [0, 0, 0]).as_quat()},
            
            # Retorno à posição neutra
            {'position': [0.0, -0.3, -0.2], 'orientation': Rotation.from_euler('xyz', [0, -np.pi/2, 0]).as_quat()}
        ]
        
        # Aplica compensação de gravidade
        for p in key_positions:
            p['position'][2] += self.gravity_comp
            
        return self.generate_smooth_trajectory(key_positions)

    def generate_smooth_trajectory(self, key_points):
        """Gera trajetória com pausas e suavização"""
        full_path = []
        
        for i in range(len(key_points)):
            start = key_points[i]
            end = key_points[(i+1) % len(key_points)]
            
            # Segmento principal
            segment = self.interpolate_points(start, end, self.interpolation_points)
            full_path.extend(segment)
            
            # Pausa no ponto-chave (corrigido para usar dicionário consistente)
            full_path.extend([{
                'position': end['position'],
                'orientation': end['orientation']
            }] * int(self.dwell_time/0.05))
            
        return full_path

    def interpolate_points(self, start, end, num_points):
        """Interpolação com curva sigmoidal para aceleração suave"""
        points = []
        for i in range(num_points):
            alpha = 0.5 * (1 - np.cos(np.pi * i/(num_points-1)))
            interp_pos = [
                start['position'][0] + alpha*(end['position'][0] - start['position'][0]),
                start['position'][1] + alpha*(end['position'][1] - start['position'][1]),
                start['position'][2] + alpha*(end['position'][2] - start['position'][2])
            ]
            points.append({
                'position': interp_pos,
                'orientation': end['orientation'] if alpha > 0.5 else start['orientation']
            })
        return points

    def publish_pose(self):
        if self.current_point >= len(self.trajectory):
            self.current_point = 0
            
        point = self.trajectory[self.current_point]
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'torso'
        
        # Acesso correto às chaves
        msg.pose.position.x = point['position'][0]
        msg.pose.position.y = point['position'][1]
        msg.pose.position.z = point['position'][2] + 0.01
        
        msg.pose.orientation.x = point['orientation'][0]
        msg.pose.orientation.y = point['orientation'][1]
        msg.pose.orientation.z = point['orientation'][2]
        msg.pose.orientation.w = point['orientation'][3]
        
        self.pose_pub.publish(msg)
        self.current_point += 1

def main(args=None):
    rclpy.init(args=args)
    node = PhysioArmMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando sessão de fisioterapia...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()