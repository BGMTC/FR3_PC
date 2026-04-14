from typing import List

from pymodbus.client import ModbusTcpClient


class ModbusIO:
    '''Creates connection to Beckhoff Modbus IO module
    Connection setup
    - With switches set to 0,1 default IP address of device is 172.16.18.1
    - To get device mac address use default IP connection, in terminal:
        $ arp -a
    - IP address can be set using beckhoff BootP tool (windows), switches F,1
        (this is only valid until device re-boot)
    - To set IP using DHCP change switches to F,0
    - alternatively via TwinCAT system manager

    docs: https://download.beckhoff.com/download/Document/io/fieldbus-box/fbb-x900en.pdf

    note that the client will timeout (10 seconds by default) so must be kept
    alive within a loop.
    '''
    def __init__(self, ip_address:str = '172.16.18.1'):
        self.client = ModbusTcpClient(ip_address, 502, timeout=10, slave=1, auto_open=True, auto_close=True)
        self.client.connect()

    def reset_connection(self):
        rq = self.client.write_register(0x1121, 48847)
        rq = self.client.write_register(0x1121, 45054)
        if rq.isError():
            raise ConnectionError('failed to write reset registers')

    def close_interface(self):
        """try to kill th socket connection gracefully
        """
        self.client.close()
    
    def read_many(self,start_address:int, number_of_coils:int) -> List[bool]:
        """reads set of coil states
        Args:
            start_address (int): first coil to read
            number_of_coils (int)
        Raises:
            ConnectionError
        Returns:
            List[bool]: list of coil values
        """
        res = self.client.read_coils(start_address, number_of_coils, slave=1)
        if res.isError():
            raise ConnectionError(f'could not read coils {start_address} to {start_address+number_of_coils}')
        return res.bits
    
    def read_one(self,coil_address:int) -> bool:
        """reads state of one coil
        Args:
            coil_address (int): zero indexed coil number
        Raises:
            ConnectionError
        Returns:
            bool: coil value
        """
        res = self.client.read_coils(coil_address,1)
        if res.isError():
            raise ConnectionError(f'could not read coil {coil_address}')
        return res.bits[0]

    def write_one(self,coil_address:int,value:bool) -> bool:
        """Write a single coil output
        Args:
            coil_address (int): zero indexed coil number
            value (bool): target value
        Raises:
            ConnectionError
        Returns:
            bool: True if successful
        """
        r = self.client.write_coil(coil_address, value)
        print(r)
        if r.isError():
            raise ConnectionError(f'could not write coil {coil_address}')
        return True

    def write_many(self,start_coil:int,values:List[bool]) -> bool:
        r = self.client.write_coils(start_coil,values)
        if r.isError():
            raise ConnectionError(f'could not write multiple coils')
        return True

def test():
    number_of_coils = 12
    start_address = 0
    coil_state = []
    test_coil = 1
    test_value = True

    M = ModbusIO('172.20.9.175')
    M.reset_connection()
    coil_state = M.read_many(start_address, number_of_coils)
    print(coil_state)
    M.write_one(test_coil,test_value)
    r = M.read_one(test_coil)
    print(r)
    res = M.write_one(test_coil,test_value)
    print(res)
    coil_state = M.read_many(start_address, number_of_coils)
    print(coil_state)
    # example loop to hold IO state
    vals = [False,False,True,True,True,False,False,False,False,False,False,False]
    n = 0
    while n<100:
        M.write_many(0,vals)
        print(M.read_many(0,12))
        n+=1

if __name__ == "__main__":
    test()

