"""
Сохранение построенной карты
"""
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
import yaml
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        
        self.client = self.create_client(GetMap, '/map_server/map')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.save_map()
    
    def save_map(self):
        request = GetMap.Request()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            map_msg = future.result().map
            self.get_logger().info('Map received, saving...')
            
            # Создание директории
            os.makedirs('maps', exist_ok=True)
            
            # Сохранение PGm
            import numpy as np
            width = map_msg.info.width
            height = map_msg.info.height
            data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
            
            # Инверсия и сохранение
            pgm_data = 255 - ((data + 1) * 127).clip(0, 255).astype(np.uint8)
            from PIL import Image
            img = Image.fromarray(pgm_data)
            img.save('maps/map.pgm')
            
            # Сохранение YAML
            yaml_data = {
                'image': 'map.pgm',
                'resolution': map_msg.info.resolution,
                'origin': [
                    map_msg.info.origin.position.x,
                    map_msg.info.origin.position.y,
                    0.0
                ],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196
            }
            
            with open('maps/map.yaml', 'w') as f:
                yaml.dump(yaml_data, f)
            
            self.get_logger().info('Map saved to maps/')
        else:
            self.get_logger().error('Failed to get map')

def main():
    rclpy.init()
    node = MapSaver()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
