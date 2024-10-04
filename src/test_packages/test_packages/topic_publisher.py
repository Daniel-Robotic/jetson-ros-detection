import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MyPublisherExample(Node):
    
    def __init__(self):
        super().__init__(node_name='my_publisher_example')
        
        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='my_topic',
                                                qos_profile=10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.i = 0
        
    def timer_callback(self):
        msg = String()
        
        msg.data = f'Hello world {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1
        
        
def main(args=None):
    rclpy.init(args=args)
    
    my_pub_ex = MyPublisherExample()
    rclpy.spin(my_pub_ex)
    my_pub_ex.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__ == "__main__":
    main()