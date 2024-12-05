import socket
import serial
import rclpy
from libnmea_navsat_driver.driver import Ros2NMEADriver
# import pyrtcm
from threading import Thread

class SerialDriverWithSocket:
    def __init__(self, driver):
        self.driver = driver
        self.frame_id = driver.get_frame_id()
        self.serial_port = driver.declare_parameter('port', '/dev/ttyUSB0').value
        self.serial_baud = driver.declare_parameter('baud', 4800).value
        self.socket_host = driver.declare_parameter('socket_host', 'localhost').value
        self.socket_port = driver.declare_parameter('socket_port', 2789).value
        self.GPS = None
        self.socket = None
        self.socket_thread = None

    def init_serial(self):
        # Open the GPS serial port for reading and writing
        self.GPS = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
        self.driver.get_logger().info(f"Successfully connected to {self.serial_port} at {self.serial_baud}.")

    def init_socket(self):
        # Set up socket to read RTCM data
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.socket_host, self.socket_port))
        self.driver.get_logger().info(f"Connected to RTCM stream at {self.socket_host}:{self.socket_port}")

    def read_serial(self):
        # Continuously read from the GPS serial port
        try:
            while rclpy.ok():
                data = self.GPS.readline().strip()

                self.driver.get_logger().info(f"Got RTCM")
                
                if data:
                    # rclpy.spin_once(self.driver)  # Process ROS2 callbacks
                    try:
                        if isinstance(data, bytes):
                            data = data.decode("utf-8")
                        self.driver.add_sentence(data, self.frame_id)
                    except ValueError as e:
                        self.driver.get_logger().warn(
                            f"Value error, likely due to missing fields in the NMEA message. Error was: {e}. "
                            "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
                            "with the NMEA sentences that caused it.")
        except Exception as e:
            self.driver.get_logger().error(f"Error while reading from serial port: {e}")
            self.GPS.close()
            # rclpy.shutdown()

    def read_socket(self):
        # Continuously read RTCM data from the socket
        try:
            while rclpy.ok():
                rtcm_data = self.socket.recv(1024)  # Adjust the buffer size if needed
                if rtcm_data:
                    # Use pyrtcm to decode the RTCM data (optional step, depending on whether you want to process it)
                    # try:
                    #     # Decode the RTCM data
                    #     messages = pyrtcm.decode(rtcm_data)
                    #     for msg in messages:
                    #         # Log the message if needed (optional)
                    #         self.driver.get_logger().info(f"Decoded RTCM message: {msg}")
                    # except Exception as e:
                    #     self.driver.get_logger().error(f"Error decoding RTCM data: {e}")
                    
                    # Write the raw RTCM data directly to the GPS serial port
                    self.GPS.write(rtcm_data)
                    self.driver.get_logger().info("Written RTCM data to GPS.")
        except Exception as e:
            self.driver.get_logger().error(f"Error while reading from socket: {e}")
            self.socket.close()
            rclpy.shutdown()

    def start_threads(self):
        # Start separate threads for serial and socket reading
        serial_thread = Thread(target=self.read_serial)
        socket_thread = Thread(target=self.read_socket)
        
        serial_thread.start()
        socket_thread.start()

        serial_thread.join()
        socket_thread.join()


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    serial_driver_with_socket = SerialDriverWithSocket(driver)

    try:
        serial_driver_with_socket.init_serial()
        serial_driver_with_socket.init_socket()
        serial_driver_with_socket.start_threads()
    except Exception as e:
        driver.get_logger().error(f"Initialization error: {e}")
        rclpy.shutdown()
