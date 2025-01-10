import rclpy # type: ignore
from rclpy.node import Node # type: ignore

from std_msgs.msg import String # type: ignore


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('listener_node_2')
        self.subscription = self.create_subscription(
            String,
            'robot_status',
            self.listener_callback,10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('I heard %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()