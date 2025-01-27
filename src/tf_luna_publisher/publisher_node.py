import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class TfLunaPublisher(Node):
    def __init__(self):
        super().__init__('tf_luna_publisher')
        
        # Defina o serial port do TF Luna
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)

        # Verifica se o dispositivo está corretamente conectado
        if not self.ser.isOpen():
            self.ser.open()

        self.publisher_ = self.create_publisher(String, 'tf_luna_data', 10)
        self.timer = self.create_timer(0.2, self.timer_callback)  # Publica a cada 100ms

    def read_tfluna_data(self):
        while True:
            counter = self.ser.in_waiting  # Conta o número de bytes esperando para serem lidos
            bytes_to_read = 9  # O pacote de dados do TF Luna tem 9 bytes
            if counter > bytes_to_read - 1:
                bytes_serial = self.ser.read(bytes_to_read)  # Lê os 9 bytes
                self.ser.reset_input_buffer()  # Reseta o buffer do serial

                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:  # Verifica os primeiros dois bytes
                    distance = bytes_serial[2] + bytes_serial[3] * 256  # Distância
                    strength = bytes_serial[4] + bytes_serial[5] * 256  # Intensidade do sinal
                    temperature = bytes_serial[6] + bytes_serial[7] * 256  # Temperatura
                    temperature = (temperature / 8) - 256  # Escala e offset para temperatura
                    return distance, strength, temperature  # Distância em metros, força do sinal e temperatura

    def timer_callback(self):
        distance, strength, temperature = self.read_tfluna_data()  # Lê os dados do TF Luna
        
        # Publica os dados em um formato string (poderia ser um tipo customizado, mas String é suficiente aqui)
        msg = String()
        msg.data = f"Distância: {distance:.2f} cm, Temperatura: {temperature:.2f} °C"
        
        self.publisher_.publish(msg)  # Publica os dados
        self.get_logger().info(f'Publicando: {msg.data}')

    def close_serial_connection(self):
        self.ser.close()  # Fecha a conexão serial quando o nó for desligado

def main(args=None):
    rclpy.init(args=args)
    node = TfLunaPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial_connection()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
