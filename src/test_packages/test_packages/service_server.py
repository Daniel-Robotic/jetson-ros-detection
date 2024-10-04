import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts


class MyServiceServer(Node):

    def __init__(self):
        super().__init__('my_service_server')
        
        self.srv = self.create_service(AddTwoInts, 
                                       'add_two_ints',
                                       self.add_two_ints_callback)
        
        self.get_logger().info("Service server started")
        
    def add_two_ints_callback(self, 
                              request: AddTwoInts.Request, 
                              response: AddTwoInts.Response):
        
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    
    rclpy.init(args=args)
    rclpy.spin(MyServiceServer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
