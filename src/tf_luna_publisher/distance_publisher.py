import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class TFLunaPublisher(Node):
    def __init__(self):
        super().__init__('tf_luna_publisher')
        self.publisher_ = self.create_publisher(Float32, 'tf_luna_distance', 10)
        self.timer = self.create_timer(0.3, self.timer_callback)  
        self.serial_port = '/dev/ttyUSB0'  # Porta serial do TF Luna
        self.baud_rate = 115200  # Baud rate configurado
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info('TF Luna conectado com sucesso!')
        except serial.SerialException as e:
            self.get_logger().error(f'Erro ao conectar ao TF Luna: {e}')
            self.ser = None

    def timer_callback(self):
        if self.ser is None:
            return
        try:
            data = self.ser.read(9)  # Ler 9 bytes do TF Luna
            self.ser.reset_input_buffer()  # Reseta o buffer do serial
            if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
                distance = data[2] + data[3] * 256  # Calcular a distância
                checksum = sum(data[:8]) & 0xFF
                if checksum == data[8]:  # Verificar checks um
                    distance_m = distance / 100.0 # Converter para metros
                    msg = Float32() 
                    msg.data = distance_m
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Publicando distância: {distance_m:.2f} m') 
                else:
                    self.get_logger().error('Checksum inválido!')
        except Exception as e:
            self.get_logger().error(f'Erro ao ler dados do TF Luna: {e}')

    def close_serial_connection(self):  
        self.ser.close()  # Fecha a conexão serial quando o nó for desligado

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
