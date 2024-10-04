import rclpy
import rclpy.handle
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from test_interfaces.action import FibAction


class MySimpleActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')

        self._shutdown_requested = False
        
        self.client_action_ = ActionClient(node=self, 
                                           action_type=FibAction,
                                           action_name='fibonacci')
    
    def get_shutdown_request(self):
        return self._shutdown_requested
    
    def send_goal(self, target_number: int, period: float):
        # Wait for the server
        self.client_action_.wait_for_server()
        
        # Create goal
        goal = FibAction.Goal(target_number=target_number,
                              period=period)
        
        # Send the goal
        self.get_logger().info("Sending goal")
        self.client_action_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)
        
        # Send a cancel request 2 second latter
        self.timer_ = self.create_timer(2.0, self.canceled_goal)
        
    def canceled_goal(self):
        self.get_logger().info('Send a cancel request')
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()
        
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        
        if self.goal_handle_.accepted:
            self.get_logger().info('Goal got accepted')
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) 
        else:
            self.get_logger().warn('Goal got rejected')
            
    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        
        
        if status == GoalStatus.STATUS_ACCEPTED:
            self.get_logger().info('Success')
            self._shutdown_requested = True 
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().fatal('ABORTED')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Canceled')
                
        self.get_logger().info(f'Result: {str(result.reached_number)}')
 
    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.current_number
        self.get_logger().info(f'Got feedback: {number}')   


def main(args=None):
    rclpy.init(args=args)
    client = MySimpleActionClient()
    client.send_goal(4, 1.)
    
    try:
        # Основной цикл с проверкой флага завершения
        while rclpy.ok() and not client.get_shutdown_request():
            rclpy.spin_once(client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

