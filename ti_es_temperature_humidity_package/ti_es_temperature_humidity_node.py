from smbus import SMBus
import lgpio
import rclpy
from rclpy.node import Node
import time

addr = 0x38

class temperatureHumidityNode(Node):
    def __init__(self):
        super().__init__('temperature_humidity_topic')
        print("start")
        self.bus = SMBus(1)
        time.sleep(0.1)
        self.bus.write_byte_data(addr,0,0x71)
        # print(self.bus.read_byte_data(addr,0))
        if(self.bus.read_byte_data(addr,0)!=0x18):
            print("error")
            return
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        sendData = [0xAC,0x33,0x00]
        self.bus.write_i2c_block_data(addr,0,sendData)
        time.sleep(0.08)
        self.bus.write_byte_data(addr,0,0x71)
        sensorData = self.bus.read_i2c_block_data(addr,0,6)
        # print(sensorData)

        srh = (sensorData[1] << 12) | (sensorData[2] << 4) | (sensorData[3] >> 4)
        humidity = srh/(1<<20) * 100
        print("Humidity: ",humidity)

        st = ((sensorData[3] & 0x0f) << 16) | (sensorData[4] << 8) | sensorData[5]
        temperature = st/(1<<20) * 200.0 - 50
        print("Temperature: ",temperature)

def main(args=None):
    rclpy.init(args=args)
    thn = temperatureHumidityNode()
    rclpy.spin(thn)


if __name__ == '__main__':
    main()
