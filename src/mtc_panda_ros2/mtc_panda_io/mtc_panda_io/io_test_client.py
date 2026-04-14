import time

import rclpy
from rclpy.node import Node

from mtc_panda_interfaces.srv import SetIO


class MinimalClientAsync(Node):

    def __init__(self):
        # set up service
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetIO, 'update_io')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetIO.Request()

    def send_request(self, vacuum: bool = False):
        # call service (io 5 = True for vac cup on)
        self.req.data = [False]*12
        self.req.data[4] = vacuum
        self.req.data[5] = vacuum
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    # connect to service
    minimal_client = MinimalClientAsync()

    for n in range(12):
        print(n)
        # turn vac on for 3 secs then off again
        time.sleep(3)
        minimal_client.send_request(True)
        time.sleep(3)
        minimal_client.send_request(False)

        while rclpy.ok():
            rclpy.spin_once(minimal_client)
            if minimal_client.future.done():
                try:
                    response = minimal_client.future.result()
                except Exception as e:
                    minimal_client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    minimal_client.get_logger().info(f'result: {response.success}')  # CHANGE
                    break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
