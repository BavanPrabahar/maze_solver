
#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        
        # Configure depth stream
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(config)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        self.set_distance = 0.3
        self.point_show = [(180, 132), (240, 132), (300, 132), (360, 132), (420, 132), (480, 132), 
                           (180, 176), (240, 176), (300, 176), (360, 176), (420, 176), (480, 176), 
                           (180, 220), (240, 220), (300, 220), (360, 220), (420, 220), (480, 220), 
                           (180, 264), (240, 264), (300, 264), (360, 264), (420, 264), (480, 264),  
                           (180, 308), (240, 308), (300, 308), (360, 308), (420, 308), (480, 308)]
        self.point_depth = [(132, 180), (132, 240), (132, 300), (132, 360), (132, 420), (132, 480), (132, 540),  
                            (176, 180), (176, 240), (176, 300), (176, 360), (176, 420), (176, 480), (176, 540), 
                            (220, 180), (220, 240), (220, 300), (220, 360), (220, 420), (220, 480), (220, 540),  
                            (264, 180), (264, 240), (264, 300), (264, 360), (264, 420), (264, 480), (264, 540),  
                            (308, 180), (308, 240), (308, 300), (308, 360), (308, 420), (308, 480), (308, 540), 
                            (352, 180), (352, 240), (352, 300), (352, 360), (352, 420), (352, 480), (352, 540)]
        
        self.distance1 = []
        self.obstacle_pub = self.create_publisher(Float32, '/obstacle_value', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if not depth_frame:
            return

        for i in range(len(self.point_show)):
            d0 = depth_image[self.point_depth[i]]
            d = round(d0 * self.depth_scale, 2)
            if 3 < d < 15:
                continue
            elif d != 0:
                self.distance1.append(d)
        a=min(self.distance1)
        if any(dist < self.set_distance for dist in self.distance1):
            direction = 1
            # Publish direction
        else:
            direction = 0
        
        self.distance1.clear()
        self.obstacle_pub.publish(a)
        
    def stop(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
