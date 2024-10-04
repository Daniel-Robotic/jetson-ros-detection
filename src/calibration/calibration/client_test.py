import cv2
import rclpy
from rclpy.node import Node
from functools import partial
from cv_bridge import CvBridge

from calibration_interfaces.srv import CompressedImageService


class ClientTest(Node):

    def __init__(self):
        super().__init__('my_service_client')
        self.get_logger().info("Service client started")
    
    
    def send_request(self):
        client = self.create_client(CompressedImageService, 
                                      'get_calibration_frames')
        while not client.wait_for_service(5):
            self.get_logger().warn("Waiting for Server...")
            
        request = CompressedImageService.Request()
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call))
        
        
    def callback_call(self, future):
        try:
            response = future.result()
            
            # TODO: Сделать if !response
            
            if response.status:
                self.get_logger().info(response.msg)  
                bridge = CvBridge()
                for img in response.images:
                    image = bridge.compressed_imgmsg_to_cv2(img)
                    cv2.imshow("Img", image)
                    cv2.waitKey(1)
                
                cv2.destroyWindow("Img")
            else:
                self.get_logger().error(response.msg)
            
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
    
def main(args=None):
    rclpy.init(args=args)
    
    service = ClientTest()
    service.send_request()
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
