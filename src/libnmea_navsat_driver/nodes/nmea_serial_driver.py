import socket
import serial
import rclpy
from libnmea_navsat_driver.driver import Ros2NMEADriver
# import pyrtcm
from threading import Thread
import time
import select
import signal

class SerialDriverWithSocket:
    def __init__(self, driver):
        self.driver = driver
        self.frame_id = driver.get_frame_id()
        self.serial_port = driver.declare_parameter('port', '/dev/ttyUSB0').value
        self.serial_baud = driver.declare_parameter('baud', 4800).value
        self.socket_host = driver.declare_parameter('socket_host', 'localhost').value
        self.socket_port = driver.declare_parameter('socket_port', 2789).value
        
        self.serial_thread = None
        self.socket_thread = None
        
        self.gps = None
        self.socket = None
        self.socket_thread = None
        
        self.socket_connected = False
        self.gps_connected = False

        self.shutdown_flag = False  # Flag to signal shutdown


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
        while not self.shutdown_flag:
            if not self.gps_connected:
                try:
                    # Try to establish the serial connection
                    self.gps = serial.Serial(self.serial_port, baudrate=self.serial_baud, timeout=1)
                    self.driver.get_logger().info(f'Serial connection established successfully on {self.serial_port}.')
                    self.gps_connected = True
                    
                except Exception as e:
                    # Log the error and wait for the next retry attempt
                    self.driver.get_logger().error(f'Failed to open serial connection on {self.serial_port}: {e}')
                    self.gps_connected = False

                    # Wait for retry interval before trying again
                    time.sleep(2)  # Retry every 2 seconds

            if self.gps_connected:
                retry_count = 0
                try:
                    if self.gps.in_waiting > 0:
                        data = self.gps.readline().decode('utf-8').strip()  # Read line and strip whitespace
                        self.driver.add_sentence(data, self.frame_id)
                        
                except Exception as e:
                    self.driver.get_logger().error(f"Data read failed: {e}")
                    self.gps_connected = False

    def read_socket(self):
        while not self.shutdown_flag:
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

    def start_threads(self):
        # Start separate threads for serial and socket reading
        self.serial_thread = Thread(target=self.read_serial)
        self.socket_thread = Thread(target=self.read_socket)
        
        self.serial_thread.start()
        self.socket_thread.start()

        self.serial_thread.join()
        self.socket_thread.join()

    def stop_threads(self):
        # Set shutdown flag to True, which will cause threads to exit
        self.shutdown_flag = True
        # Join the threads to wait for them to finish gracefully
        if self.serial_thread is not None:
            self.serial_thread.join()
        if self.socket_thread is not None:
            self.socket_thread.join()
    
    def signal_handler(self, signum, frame):
        self.driver.get_logger().info("Received shutdown signal, stopping threads.")
        self.stop_threads()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    driver = Ros2NMEADriver()
    serial_driver_with_socket = SerialDriverWithSocket(driver)

    signal.signal(signal.SIGINT, serial_driver_with_socket.signal_handler)


    try:
        serial_driver_with_socket.init_serial()
        serial_driver_with_socket.init_socket()
        serial_driver_with_socket.start_threads()

    except Exception as e:
        driver.get_logger().error(f"Initialization error: {e}")
        rclpy.shutdown()

    finally:
        # Make sure to stop threads and shut down ROS2 properly
        serial_driver_with_socket.stop_threads()
        driver.get_logger().info("Shutdown complete.")
