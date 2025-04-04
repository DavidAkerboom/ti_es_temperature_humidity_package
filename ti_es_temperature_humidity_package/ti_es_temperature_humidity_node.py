import smbus
import lgpio
import rclpy
from rclpy.node import Node
import time
from enum import Enum
from dataclasses import dataclass

from std_msgs.msg import String

# Settings
ADDR = 0x38
I2C_TIMER = 1.0

# Setup i2c
i2c = smbus.SMBus(1)

@dataclass
class sensorData:
    temperature: float
    humidity: float


class temperatureHumidityNode(Node):
    def __init__(self):
        super().__init__('temperature_humidity_topic')

        time.sleep(0.1)
        while True:
            try:
                i2c.write_byte_data(ADDR, 0, 0x71)  # Check status word
                if i2c.read_byte_data(ADDR, 0) == 0x18:
                    break  # Sensor is available
            except OSError:
                print("Error: Sensor not found. Retrying...")
                time.sleep(1)  # Wait before retrying

        self.log_publisher = self.create_publisher(String, "ti/es/log_data", 10)
        self.timer = self.create_timer(I2C_TIMER, self.timer_callback)


    # Timer callback that repeatedly checks measurements from the sensor
    def timer_callback(self):
        try:
            i2c.write_i2c_block_data(ADDR,0,[0xAC,0x33,0x00]) # trigger measurement
            time.sleep(0.08)
            i2c.write_byte_data(ADDR,0,0x71)
        except:
            print("Error: Failed to receive sensor data")
            return

        try:
            data = self.get_data_from_sensor()

            log_msg = String()
            log_msg.data = f"temperature/humidity data: {data}"
            self.log_publisher.publish(log_msg)

            print(f"temperature/humidity data: {data}")
        except:
            print("Error: Failed to process data")


    # Method that reads the data from the sensor and calculates the proper values
    def get_data_from_sensor(self):
        data = i2c.read_i2c_block_data(ADDR,0,6)
        sd = sensorData(0.0,0.0)

        srh = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4) # relative humidity signal
        sd.humidity = srh/(1<<20) * 100

        st = ((data[3] & 0x0f) << 16) | (data[4] << 8) | data[5] # temperature signal
        sd.temperature = st/(1<<20) * 200.0 - 50

        return sd


def main(args=None):
    rclpy.init(args=args)
    thn = temperatureHumidityNode()

    print('Temperature/humidity Package has booted!')
    try:
        rclpy.spin(thn)
    except KeyboardInterrupt:
        pass
    finally:
        thn.destroy_node()


if __name__ == '__main__':
    main()
