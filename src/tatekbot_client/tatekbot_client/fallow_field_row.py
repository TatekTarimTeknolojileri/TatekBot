import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tatekbot_action_interfaces.action import FollowFieldRow

class FollowFieldClient(Node):

    def __init__(self):
        super().__init__('follow_field_row_client')
        self._action_client = ActionClient(self, FollowFieldRow, 'follow_field_row')

    def send_goal(self, request):
        goal_msg = FollowFieldRow.Goal()
        goal_msg.request = request

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FollowFieldClient()

    future = action_client.send_goal('start')

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
