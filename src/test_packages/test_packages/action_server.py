import time
import rclpy

import cv2
import numpy

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from test_interfaces.action import FibAction


class MySimpleActionServer(Node):

    def __init__(self):
        super().__init__('my_simple_action_server')
    
        self.fib_action_ = ActionServer(node=self,
                                        action_type=FibAction,
                                        action_name='fibonacci',
                                        goal_callback=self.goal_callback,
                                        cancel_callback=self.cancel_callback,
                                        execute_callback=self.execute_callback,
                                        callback_group=ReentrantCallbackGroup())
        self.get_logger().info("Action server has been started")
        
    
    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn('Received a cancel request')
        return CancelResponse.ACCEPT  # or REJECT
    
    def goal_callback(self, goal_request: FibAction.Goal):
        if goal_request.target_number <= 0:
            self.get_logger().warn('Rejecting the goal')
            return GoalResponse.REJECT
        
        self.get_logger().info('Accepting the goal')
        return GoalResponse.ACCEPT

        
    def execute_callback(self, goal_handle: ServerGoalHandle):
        
        #  get request from goal
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        
        # Execute the action
        self.get_logger().info("Execute the goal")
        feedback = FibAction.Feedback()
        counter = 0
        result = FibAction.Result()
        
        for i in range(0, target_number):
            
            self.get_logger().warn(str(goal_handle.is_cancel_requested))
            
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Canceling the goal')
                goal_handle.canceled()
                result.reached_number = counter
                return result
            
            counter += 1
            self.get_logger().info(str(i))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)
            
        # Once done, set goal final state
        # goal_handle.abort()
        goal_handle.succeed()
        
        # and send the result
        result.reached_number = counter
        
        return result
        

def main(args=None):
    rclpy.init(args=args)
    action_server = MySimpleActionServer()
    rclpy.spin(action_server, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

