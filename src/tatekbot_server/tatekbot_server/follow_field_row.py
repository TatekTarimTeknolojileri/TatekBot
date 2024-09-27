import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

import sys
import os

# Add the directory containing the Camera, ImageProcessor, and other dependencies to the Python path
script_dir = os.path.dirname(os.path.realpath(__file__))
field_row_detection_dir = os.path.join(script_dir, 'field_row_detection')
sys.path.append(field_row_detection_dir)

from tatekbot_action_interfaces.action import FollowFieldRow
from field_row_detection.ImageProcessor import CropRowImageProcessingNode



class VisualNavigationServer(Node):

    def __init__(self):
        super().__init__('follow_field_row_action_server')
        self._action_server = ActionServer(
            self,
            FollowFieldRow,
            'follow_field_row',
            self.execute_callback)
        self.get_logger().info('follow_field_row_action_server has been started.')

        self.P = None
        self.ang = None

        

    
    
    def execute_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info('Executing goal...')
        request = goal_handle.request.request
        print(request)
        if request == 'start':
            self.get_logger().info('Following field row server has been started.')
            self.get_logger().info('Robot is now following field.')
            # Implement segmentation logic and make neseccary calculations
            self.rowDetector = CropRowImageProcessingNode()
            
            self.timer = self.create_timer(1.0, self.main_line_status)
            #add condition later to stop image processing
            if True:
                # self.P, self.ang = rowDetector.getMainLineFeatures()
                # print(f"P value: {self.P}")
                # print(f"ang value: {self.ang}")    
                # Think about it for later
                #if self.p !=None and self.ang != None:
                rclpy.spin(self.rowDetector)

                
                # For now its capable to detect rows and give a path to navigate trough crops


            
        self.rowDetector.destroy_node()
        goal_handle.succeed()
        
        return "4"
    
    def main_line_status(self):
        # Fetch P and ang values every time this callback is executed
        self.P, self.ang = self.rowDetector.getMainLineFeatures()
        print(f"P value: {self.P}")
        print(f"ang value: {self.ang}")


def main(args=None):
    rclpy.init(args=args)

    action_server = VisualNavigationServer()

    rclpy.spin(action_server)

    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()