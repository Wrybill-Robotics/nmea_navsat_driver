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
        self.serial_baud = driver.declare_parameter('baud', 115200).value
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
        # From m2
        #BESTPOSA,   COM3,0,92.5,FINESTEERING,2343,94540.400,00000000,0000,781;SOL_COMPUTED,NARROW_INT,-40.37993097965,175.61293101574,28.5369,15.0493,WGS84,0.0156,0.0163,0.0301,"1195",0.400,0.017,31,22,22,22,00,00,30,37*1d21563a

        # M1 Outside
        #BESTNAVA,      88,GPS,FINE,2380,440535600,0,0,18,9;SOL_COMPUTED,PSRDIFF,-40.38011892865,175.61256910407,28.3449,14.6861,WGS84,0.4778,0.4548,1.1723,"1195",61.600,58.000,42,27,27,0,1,18,11,51,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0015,0.941811,-0.0045,0.0160,0.0110*bc89ad07
        #UNIHEADINGA,   88,GPS,FINE,2380,440535600,0,0,18,14;SOL_COMPUTED,L1_INT,0.8176,210.8297,0.3650,0.0000,0.4080,0.8552,"999",42,30,30,0,3,01,1,51*69ac1b05
        #BESTNAVA,      88,GPS,FINE,2380,440535800,0,0,18,10;SOL_COMPUTED,PSRDIFF,-40.38011891726,175.61256911644,28.3447,14.6861,WGS84,0.4783,0.4551,1.1738,"1195",61.800,58.000,42,27,27,0,1,18,11,51,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0019,350.223961,-0.0044,0.0160,0.0109*6d3d8b32
        #UNIHEADINGA,   88,GPS,FINE,2380,440535800,0,0,18,14;SOL_COMPUTED,L1_INT,0.8211,211.0314,0.7240,0.0000,0.4060,0.8522,"999",42,30,30,0,3,01,1,51*35134843
        #BESTNAVA,      88,GPS,FINE,2380,440536000,0,0,18,11;SOL_COMPUTED,PSRDIFF,-40.38011890692,175.61256912540,28.3443,14.6861,WGS84,0.4782,0.4555,1.1737,"1195",62.000,58.000,42,27,27,0,1,18,11,51,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0019,344.258914,-0.0043,0.0160,0.0109*d4ff2939
        #UNIHEADINGA,   88,GPS,FINE,2380,440536000,0,0,18,16;SOL_COMPUTED,L1_INT,0.8224,210.8856,0.5414,0.0000,0.4055,0.8504,"999",42,30,30,0,3,01,1,51*8bf17726
        #BESTNAVA,      88,GPS,FINE,2380,440536200,0,0,18,10;SOL_COMPUTED,PSRDIFF,-40.38011888799,175.61256912895,28.3439,14.6861,WGS84,0.4789,0.4560,1.1757,"1195",62.200,58.000,42,27,27,0,1,18,11,51,SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.0021,331.251882,-0.0045,0.0160,0.0109*2414d69d

        # M1 Inside
        #BESTNAVA,   95,GPS,FINE,2380,442844200,0,0,18,3;INSUFFICIENT_OBS,NONE,-40.37981427103,175.61286287752,-32.8624,14.6875,WGS84,204.9449,180.4498,221.7988,"0",0.000,1798.000,2,2,0,0,161,02,00,00,INSUFFICIENT_OBS,NONE,0.000,0.000,0.0000,0.000000,0.0000,14.7774,11.0171*5a8112b0
        #UNIHEADINGA,95,GPS,FINE,2380,442844200,0,0,18,5;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*69dc1b69
        #BESTNAVA,   95,GPS,FINE,2380,442844400,0,0,18,3;INSUFFICIENT_OBS,NONE,-40.37981427103,175.61286287752,-32.8624,14.6875,WGS84,204.9449,180.4498,221.7988,"0",0.000,1798.200,2,2,0,0,161,02,00,00,INSUFFICIENT_OBS,NONE,0.000,0.000,0.0000,0.000000,0.0000,14.7774,11.0171*77734a67
        #UNIHEADINGA,95,GPS,FINE,2380,442844400,0,0,18,5;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*c6583d26
        #BESTNAVA,   95,GPS,FINE,2380,442844600,0,0,18,3;INSUFFICIENT_OBS,NONE,-40.37981427103,175.61286287752,-32.8624,14.6875,WGS84,204.9449,180.4498,221.7988,"0",0.000,1798.400,2,2,0,0,161,02,00,00,INSUFFICIENT_OBS,NONE,0.000,0.000,0.0000,0.000000,0.0000,14.7774,11.0171*d3c89c13
        #UNIHEADINGA,95,GPS,FINE,2380,442844600,0,0,18,5;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*a32420e3
        #BESTNAVA,   95,GPS,FINE,2380,442844800,0,0,18,3;INSUFFICIENT_OBS,NONE,-40.37981427103,175.61286287752,-32.8624,14.6875,WGS84,204.9449,180.4498,221.7988,"0",0.000,1798.600,2,2,0,0,161,02,00,00,INSUFFICIENT_OBS,NONE,0.000,0.000,0.0000,0.000000,0.0000,14.7774,11.0171*2c97fbc9
        #UNIHEADINGA,95,GPS,FINE,2380,442844800,0,0,18,5;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*422177f9
        #BESTNAVA,   95,GPS,FINE,2380,442845000,0,0,18,3;INSUFFICIENT_OBS,NONE,-40.37981427103,175.61286287752,-32.8624,14.6875,WGS84,204.9449,180.4498,221.7988,"0",0.000,1798.800,2

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
                            # self.driver.get_logger().info(f"Received RTCM from socket: {len(rtcm_data)}")
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
