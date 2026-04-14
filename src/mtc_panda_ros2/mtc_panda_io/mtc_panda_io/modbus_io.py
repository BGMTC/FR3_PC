import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mtc_panda_interfaces.srv import SetIO
from mtc_panda_io.beckhoff import ModbusIO


class IOService(Node):
    def __init__(self, ip:str, num_coils:int=12, start_coil:int=0, loop_speed:float=0.1) -> None:
        super().__init__('Beckhoff_IO_service')
        # create modbus connection
        self.ip = ip
        self.start_coil = start_coil
        self.num_coils = num_coils
        self.io_connection = ModbusIO(self.ip)
        # create service to externally update list of io_state
        self.srv = self.create_service(SetIO, 'update_io', self.update_io_callback)
        self.io_state = [False]*self.num_coils # default to all off
        # create publisher which will broadcast current state on each loop (0.1 secs default)
        #loop_speed = self.get_parameter('loop_speed').get_parameter_value()
        self.pub = self.create_publisher(String, 'io_state',10)
        # keep io set with timer loop
        self.timer = self.create_timer(loop_speed, self.set_io)

    def set_io(self):
        # update io
        self.io_connection.reset_connection()
        self.io_connection.write_many(self.start_coil, self.io_state)
        msg = String()
        msg.data = str(self.io_state)
        self.pub.publish(msg)

    def update_io_callback(self, request, response):
        # update internal list for io state
        self.io_state = request.data 
        # trigger update
        self.set_io()
        # check that we updated successfully
        print(self.io_connection.read_many(self.start_coil, self.num_coils))
        if np.all(self.io_connection.read_many(self.start_coil, self.num_coils)[:self.num_coils] == request.data):
            response.success = True
        else:
            response.success = False
        return response

def main():
    print('Initialising mtc_beckhoff_modbus_io IOService node.')
    rclpy.init()
    node = IOService('172.20.9.175') #set ip address as needed
    #run node
    rclpy.spin(node)
    #tidy up
    node.io_connection.close_interface()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
