import socket
import serial
import rclpy
from libnmea_navsat_driver.driver import Ros2NMEADriver
# import pyrtcm
from threading import Thread
import time
import select

class SerialDriverWithSocket:
    def __init__(self, driver):
        self.driver = driver
        self.frame_id = driver.get_frame_id()
        self.serial_port = driver.declare_parameter('port', '/dev/ttyUSB0').value
        self.serial_baud = driver.declare_parameter('baud', 4800).value
        self.socket_host = driver.declare_parameter('socket_host', 'localhost').value
        self.socket_port = driver.declare_parameter('socket_port', 2789).value
        
        self.gps = None
        self.socket = None
        self.socket_thread = None
        
        self.socket_connected = False
        self.gps_connected = False

    def init_serial(self):
        # Open the GPS serial port for reading and writing
        self.gps = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, timeout=2)
        self.driver.get_logger().info(f"Successfully connected to {self.serial_port} at {self.serial_baud}.")

    def init_socket(self):
        # Set up socket to read RTCM data
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.socket_host, self.socket_port))
        self.driver.get_logger().info(f"Connected to RTCM stream at {self.socket_host}:{self.socket_port}")

    def read_serial(self):
        # Continuously read from the GPS serial port
        # try:
        #     while rclpy.ok():
        #         data = self.gps.readline().strip()

        #         if data:
        #             # rclpy.spin_once(self.driver)  # Process ROS2 callbacks
        #             try:
        #                 if isinstance(data, bytes):
        #                     data = data.decode("utf-8")
        #                 self.driver.add_sentence(data, self.frame_id)
        #             except ValueError as e:
        #                 self.driver.get_logger().warn(
        #                     f"Value error, likely due to missing fields in the NMEA message. Error was: {e}. "
        #                     "Please report this issue at github.com/ros-drivers/nmea_navsat_driver, including a bag file "
        #                     "with the NMEA sentences that caused it.")
        # except Exception as e:
        #     self.driver.get_logger().error(f"Error while reading from serial port: {e}")
        #     self.gps.close()
        #     # rclpy.shutdown()

        while True:
            # retry_count = 100  # Retry every 2 seconds
            if not self.gps_connected:
                try:
                    # Try to establish the serial connection
                    self.gps = serial.Serial(self.serial_port, baudrate=self.serial_baud, timeout=1)
                    self.driver.get_logger().info(f'Serial connection established successfully on {self.serial_port}.')
                    # retry_count += 1
                    self.gps_connected = True
                    
                except Exception as e:
                    # Log the error and wait for the next retry attempt
                    self.driver.get_logger().error(f'Failed to open serial connection on {self.serial_port}: {e}')
                    self.gps_connected = False

                    # Wait for retry interval before trying again
                    # self.get_logger().info(f'Retrying in {(100-retry_count)/1000} seconds...')
                    # time.sleep(retry_interval)  # Sleep for 2 seconds before trying again
                    time.sleep(2)  # Retry every 2 seconds

            if self.gps_connected:
                retry_count = 0
            # self.get_logger().info(f"Reading serial")
                # Check if data is available to read from serial
                try:
                    if self.gps.in_waiting > 0:
                        data = self.gps.readline().decode('utf-8').strip()  # Read line and strip whitespace
                        # self.driver.get_logger().info(f"Data read from serial: {data}")
                        self.driver.add_sentence(data, self.frame_id)
                        # self.gps_queue.put(data)

                except Exception as e:
                    self.driver.get_logger().error(f"Data read failed: {e}")
                    self.gps_connected = False

    def read_socket(self):
        # Continuously read RTCM data from the socket
        try:
            while rclpy.ok():
                if not self.socket_connected:
                    try:
                        # Try to connect to the socket
                        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.socket.connect((self.socket_host, self.socket_port))  # Example IP and port
                        self.driver.get_logger().info(f"Socket connected to {self.socket_host}:{self.socket_port}")
                        self.socket_connected = True
                    except Exception as e:
                        self.driver.get_logger().error(f"Socket connection failed: {e}")
                        self.socket_connected = False
                        time.sleep(2)  # Retry every 2 seconds

                # Once connected, start receiving data
                if self.socket_connected and rclpy.ok():
                    try:
                        # Use select for non-blocking socket read
                        ready_to_read, _, _ = select.select([self.socket], [], [], 1)  # 1-second timeout
                        if ready_to_read:
                            rtcm_data = self.socket.recv(2048)  # Adjust the buffer size if needed
                            if rtcm_data:
                                # self.get_logger().info(f"Received RTCM from socket: {len(rtcm_data)}")
                                # self.rtcm_queue.put(rtcm_data)  # Put the message in the queue
                                if self.gps_connected:
                                    self.gps.write(rtcm_data)
                                else:
                                    self.driver.get_logger().error(f"GPS not connected - abandoning RTCM")

                            else:
                                # If data is empty, that means the socket was closed
                                self.driver.get_logger().info("Socket closed, reconnecting...")
                                self.socket_connected = False  # Set it to False to trigger reconnection
                        else:
                            pass
                    except Exception as e:
                        self.driver.get_logger().error(f"Socket read error: {e}")
                        self.socket_connected = False  # Set it to False to trigger reconnection
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
