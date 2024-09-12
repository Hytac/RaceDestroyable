
import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
import asyncio


class NodeServer(Node):

    def __init__(self):
        super().__init__('node_server', start_parameter_services=True)
        self.cb_group = ReentrantCallbackGroup()

        self.Service = self.create_service(
            Trigger, '~/Service', self.ConnectCallback)

        self.get_logger().info(f"Initialized {self.get_name()}")

    def ConnectCallback(self, request, response: Trigger.Response):
        self.get_logger().warning("Request handler")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_client = NodeServer()

    executor = MultiThreadedExecutor()
    executor.add_node(minimal_client)
    executor.spin()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
