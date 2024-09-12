
import rclpy
from rclpy.node import Node
import rclpy.time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
import asyncio
from threading import Thread
import time


class NodeClient(Node):

    def __init__(self):
        super().__init__('node_client', start_parameter_services=True)
        self.cb_group = ReentrantCallbackGroup()

        self.max = 0
        self.total = 0
        for x in range(10):
            thread = Thread(target=self.threaded_function, args=(x, ))
            thread.start()

        self.get_logger().info(f"Initialized {self.get_name()}")

    def threaded_function(self, index):
        while True:
            client = self.create_client(Trigger, 'node_server/Service')
            try:
                start_time = time.time()

                client.wait_for_service()
                client.service_is_ready()

                response: Trigger.Response = client.call(Trigger.Request())
                self.total += 1

                elapsed_time = time.time() - start_time

                if elapsed_time * 1000 > self.max:
                    self.max = elapsed_time * 1000

                self.get_logger().warning(
                    f"Service response recieved in {elapsed_time * 1000:.2f} ms Max {self.max:.2f} Total {self.total}")

                if response.success is False:
                    self.get_logger().error(response.message)
            finally:
                self.destroy_client(client)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = NodeClient()

    executor = MultiThreadedExecutor()
    executor.add_node(minimal_client)
    executor.spin()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
