import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class TFLunaPublisher(Node):
    def __init__(self):
        super().__init__('tf_luna_publisher')
        self.publisher_ = self.create_publisher(Float32, 'tf_luna_distance', 10)
        self.create_timer(0.3, self.timer_callback)
        
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.ser = self.connect_to_sensor()

    def connect_to_sensor(self):
        try:
            ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f'Erro ao conectar ao TF Luna: {e}')
            return None

    def timer_callback(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(9)
            self.ser.reset_input_buffer()
            if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                distance = data[2] + data[3] * 256
                checksum = sum(data[:8]) & 0xFF
                if checksum == data[8]:
                    self.publish_distance(distance)
        except Exception as e:
            self.get_logger().error(f'Erro ao ler dados do TF Luna: {e}')

    def publish_distance(self, distance):
        distance_m = distance / 100.0
        msg = Float32()
        msg.data = distance_m
        self.publisher_.publish(msg)

    def close_serial_connection(self):
        if self.ser:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = TFLunaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial_connection()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
