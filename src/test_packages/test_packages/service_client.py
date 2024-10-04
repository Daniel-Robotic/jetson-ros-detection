import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts


class MyServiceClient(Node):

    def __init__(self):
        super().__init__('my_service_client')
        self.get_logger().info("Service client started")
        
    
    def call_add_two_ints_server(self, a: int, b: int):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
    
    
    def send_request(self, a, b):
        client = self.create_client(AddTwoInts, 
                                      'add_two_ints')
        while not client.wait_for_service(5):
            self.get_logger().warn("Waiting for Server...")
            
        request = AddTwoInts.Request(a=a, b=b)
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))
        
        
    def callback_call_add_two_ints(self, future, a: int, b: int):
        try:
            response = future.result()
            self.get_logger().info(f"{a} + {b} = {response.sum}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
    
def main(args=None):
    rclpy.init(args=args)
    
    service = MyServiceClient()
    service.send_request(70, 80)
    service.send_request(128, 56473)
    service.send_request(76, 21)
    # rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
