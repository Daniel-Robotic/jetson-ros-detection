import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MySubscriberExample(Node):
    
    def __init__(self):
        super().__init__(node_name='my_subscriber_example')
        self.subscription_ = self.create_subscription(msg_type=String,
                                                      topic='my_topic',
                                                      callback=self.listener_callback,
                                                      qos_profile=10)
        self.subscription_
        
        
    def listener_callback(self, msg: String):
        self.get_logger().info(f'I heard: {msg.data}')
        
        
def main(args=None):
    rclpy.init(args=args)
    
    my_sub_ex = MySubscriberExample()
    rclpy.spin(my_sub_ex)
    my_sub_ex.destroy_node()
    rclpy.shutdown()
    
    
    
if __name__ == "__main__":
    main()