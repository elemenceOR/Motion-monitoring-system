from machine import Pin, I2C, reset
import time
from time import sleep
import network
from umqtt.simple import MQTTClient
import config
from sh1106 import SH1106_I2C  
from imu import MPU6050

# Constants
WIDTH = 128
HEIGHT = 64
MQTT_SERVER = config.mqtt_server
MQTT_PORT = 0
MQTT_USER = config.mqtt_username
MQTT_PASSWORD = config.mqtt_password
MQTT_CLIENT_ID = b"raspberrypi_picow"
MQTT_KEEPALIVE = 7200
MQTT_SSL = True
MQTT_SSL_PARAMS = {'server_hostname': MQTT_SERVER}
SEND_DELAY = 1
MAX_RETRIES = 3
RETRY_DELAY = 5


class DeviceInit:
    """Initialize and manage hardware devices (OLED display and MPU6050 sensor)"""
    
    def __init__(self, oled_sda=4, oled_scl=5, imu_sda=14, imu_scl=15):
        self.oled = None
        self.imu = None
        self.offsets = None
        self.width = WIDTH
        self.height = HEIGHT
        self.display_enabled = True
        self.display_error_count = 0
        self.i2c_oled = None
        self.oled_sda = oled_sda
        self.oled_scl = oled_scl
        self.last_reset_time = 0
        
        try:
            # Initialize OLED display
            self.i2c_oled = I2C(0, sda=Pin(oled_sda), scl=Pin(oled_scl), freq=200000)
            self.oled = SH1106_I2C(self.width, self.height, self.i2c_oled)
            self._reset_display()
            self.display_message("Initializing...")
            print("OLED initialized successfully")
            
            # Initialize IMU sensor
            i2c_imu = I2C(1, sda=Pin(imu_sda), scl=Pin(imu_scl), freq=400000)
            self.imu = MPU6050(i2c_imu)
            self.display_message("IMU initialized")
            print("IMU initialized successfully")
            
        except Exception as e:
            print(f"Device initialization error: {e}")
            self.display_message("Init Error!")
            raise
    
    def _reset_display(self):
        """Reset the OLED display to clear corruption"""
        try:
            if self.oled:
                # Power cycle the display
                self.oled.poweroff()
                time.sleep(0.1)
                self.oled.poweron()
                time.sleep(0.1)
                # Clear display buffer
                self.oled.fill(0)
                self.oled.show()
                print("Display reset performed")
        except Exception as e:
            print(f"Display reset error: {e}")
    
    def calibrate_sensor(self, cal_time=10, sample_delay=0.01):
        """Calibrate the IMU sensor by collecting baseline readings"""
        if not self.imu:
            raise RuntimeError("IMU not initialized")
        
        try:
            print(f"Calibrating for {cal_time}s - keep sensor still and flat")
            self.display_message("Calibrating...\nKeep still!")
            
            sums = [0.0] * 6
            count = 0
            end_time = time.time() + cal_time
            
            while time.time() < end_time:
                sums[0] += self.imu.accel.x
                sums[1] += self.imu.accel.y
                sums[2] += self.imu.accel.z - 9.8  # Gravity adjustment
                sums[3] += self.imu.gyro.x
                sums[4] += self.imu.gyro.y
                sums[5] += self.imu.gyro.z
                
                count += 1
                
                if count % 100 == 0:
                    print(f"Calibration points: {count}")
                
                time.sleep(sample_delay)
            
            self.offsets = [round(s / count, 2) for s in sums]
            
            self.display_message("Calibration\nComplete!")
            print(f"Calibration complete: {count} samples")
            print(f"Offsets: {self.offsets}")
            
            return self.offsets
            
        except Exception as e:
            print(f"Calibration error: {e}")
            self.display_message("Calib Error!")
            raise
    
    def display_message(self, msg, line=0, clear=True, retry=True):
        """Display message on OLED screen"""
        if not self.oled or not self.display_enabled:
            return       
        try:
            if clear:
                self.oled.fill(0)
            
            if isinstance(msg, str):
                lines = msg.split('\n')
                for i, text in enumerate(lines):
                    y_pos = line + (i * 12)
                    if y_pos < self.height:
                        self.oled.text(text[:16], 0, y_pos)
            
            self.oled.show()
          
        except OSError as e:
            
            if retry and e.errno == 110: 
                try:
                    time.sleep(0.01)
                    self.oled.show()
                except:
                    pass
            

class DataCollector:
    """Collect and process sensor data"""
    
    def __init__(self, imu, offsets):
        if not imu:
            raise ValueError("IMU sensor not provided")
        if not offsets or len(offsets) != 6:
            raise ValueError("Invalid calibration offsets")
        
        self.imu = imu
        self.offsets = offsets
    
    def read_sensor(self):
        """Read calibrated sensor values"""
        try:
            ax = round(self.imu.accel.x - self.offsets[0], 2)
            ay = round(self.imu.accel.y - self.offsets[1], 2)
            az = round(self.imu.accel.z - self.offsets[2], 2)
            gx = round(self.imu.gyro.x - self.offsets[3], 2)
            gy = round(self.imu.gyro.y - self.offsets[4], 2)
            gz = round(self.imu.gyro.z - self.offsets[5], 2)
            temp = round(self.imu.temperature, 1)
            
            return {
                'ax': ax, 'ay': ay, 'az': az,
                'gx': gx, 'gy': gy, 'gz': gz,
                'temperature': temp
            }
        except Exception as e:
            print(f"Sensor reading error: {e}")
            raise
    
    def get_formatted_data(self):
        """Get sensor data formatted for display and transmission"""
        data = self.read_sensor()
        return data, f"T:{data['temperature']}C\nAX:{data['ax']} AY:{data['ay']}\nAZ:{data['az']}"


class MQTTInit:
    """Initialize and manage WiFi and MQTT connections"""
    
    def __init__(self, display_callback=None):
        self.wlan = None
        self.client = None
        self.connected = False
        self.ip_address = None
        self.display_callback = display_callback
    
    def connect_wifi(self, ssid, password, timeout=20, max_attempts=3):
        """Connect to WiFi network with retry logic"""
        for attempt in range(max_attempts):
            try:
                print(f"\n=== WiFi Connection Attempt {attempt + 1}/{max_attempts} ===")
                
                # Reset WiFi interface
                self.wlan = network.WLAN(network.STA_IF)
                self.wlan.active(False)
                sleep(0.5)
                self.wlan.active(True)
                sleep(0.5)
                
                # Check if already connected
                if self.wlan.isconnected():
                    print("Already connected to WiFi")
                    network_info = self.wlan.ifconfig()
                    self.ip_address = network_info[0]
                    print(f'IP: {self.ip_address}')
                    self._display(f"WiFi OK\n{self.ip_address}")
                    return True
                
                print(f"Connecting to: {ssid}")
                self._display(f"WiFi Try {attempt+1}")
                
                # Disconnect from any previous connection
                self.wlan.disconnect()
                sleep(0.5)
                
                # Connect to network
                self.wlan.connect(ssid, password)
                
                # Wait for connection with status monitoring
                connection_timeout = timeout
                while connection_timeout > 0:
                    status = self.wlan.status()
                    
                    if status == 3:  # Connected (STAT_GOT_IP)
                        network_info = self.wlan.ifconfig()
                        self.ip_address = network_info[0]
                        print(f'WiFi connected! IP: {self.ip_address}')
                        self._display(f"WiFi OK\n{self.ip_address}")
                        return True
                    
                    elif status == -1:  # STAT_CONNECT_FAIL
                        print("Connection failed - wrong password or network issue")
                        break
                    elif status == -2:  # STAT_NO_AP_FOUND
                        print("Network not found - check SSID")
                        break
                    elif status == -3:  # STAT_WRONG_PASSWORD
                        print("Wrong password")
                        break
                    
                    # Status codes: 0=idle, 1=connecting, 2=wrong password
                    if connection_timeout % 5 == 0:
                        print(f'Status: {status} | Waiting... ({connection_timeout}s)')
                    
                    connection_timeout -= 1
                    sleep(1)
                
                print(f"Attempt {attempt + 1} failed")
                
            except Exception as e:
                print(f"WiFi error on attempt {attempt + 1}: {e}")
            
            if attempt < max_attempts - 1:
                print(f"Retrying in {RETRY_DELAY} seconds...")
                self._display("Retry WiFi...")
                sleep(RETRY_DELAY)
        
        print("All WiFi connection attempts failed")
        self._display("WiFi Failed!")
        return False
    
    def connect_mqtt(self, use_hivemq=True):
        """Connect to MQTT broker with error handling"""
        try:
            print("Connecting to MQTT broker...")
            self._display("MQTT Connect...")
            
            if use_hivemq:
                # Using public HiveMQ broker (as in original code)
                self.client = MQTTClient('bigles', 'broker.hivemq.com', keepalive=60)
            else:
                # Using configured MQTT broker with SSL
                self.client = MQTTClient(
                    client_id=MQTT_CLIENT_ID,
                    server=MQTT_SERVER,
                    port=MQTT_PORT,
                    user=MQTT_USER,
                    password=MQTT_PASSWORD,
                    keepalive=MQTT_KEEPALIVE,
                    ssl=MQTT_SSL,
                    ssl_params=MQTT_SSL_PARAMS
                )
            
            self.client.connect()
            self.connected = True
            print("MQTT connected successfully")
            self._display("MQTT OK!")
            return True
            
        except Exception as e:
            print(f"MQTT connection error: {e}")
            self._display("MQTT Error!")
            self.connected = False
            return False
    
    def reconnect_mqtt(self):
        """Attempt to reconnect to MQTT broker"""
        print("Attempting MQTT reconnection...")
        self._display("MQTT Reconnect...")
        
        for attempt in range(MAX_RETRIES):
            try:
                if self.client:
                    self.client.disconnect()
            except:
                pass
            
            if self.connect_mqtt():
                return True
            
            print(f"Reconnection attempt {attempt + 1} failed")
            sleep(RETRY_DELAY)
        
        return False
    
    def check_connection(self):
        """Check if WiFi and MQTT are still connected"""
        wifi_ok = self.wlan and self.wlan.isconnected()
        mqtt_ok = self.connected
        return wifi_ok and mqtt_ok
    
    def _display(self, msg):
        """Helper to display messages if callback is available"""
        if self.display_callback:
            self.display_callback(msg)


class Transmission:
    """Handle MQTT data transmission with error recovery"""
    
    def __init__(self, mqtt_client, display_callback=None):
        self.mqtt_client = mqtt_client
        self.display_callback = display_callback
        self.message_count = 0
        self.error_count = 0
    
    def publish(self, topic, value):
        """Publish single value to MQTT topic"""
        try:
            if not self.mqtt_client.connected:
                raise RuntimeError("MQTT not connected")
            
            self.mqtt_client.client.publish(topic, str(value))
            return True
            
        except Exception as e:
            print(f"Publish error on {topic}: {e}")
            self.error_count += 1
            return False
    
    def publish_sensor_data(self, data):
        """Publish all sensor data to respective topics"""
        topics = {
            'pico/temperature': data['temperature'],
            'pico/ax': data['ax'],
            'pico/ay': data['ay'],
            'pico/az': data['az'],
            'pico/gx': data['gx'],
            'pico/gy': data['gy'],
            'pico/gz': data['gz']
        }
        
        success = True
        for topic, value in topics.items():
            if not self.publish(topic, value):
                success = False
        
        if success:
            self.message_count += 1
            self._display(f"Sent #{self.message_count}")
            print(f"Message {self.message_count} sent successfully")
        else:
            self._display("Send Error!")
        
        return success
    
    def _display(self, msg):
        """Helper to display messages if callback is available"""
        if self.display_callback:
            self.display_callback(msg)


def main():
    """Main execution loop with comprehensive error handling"""
    device = None
    data_collector = None
    mqtt = None
    transmitter = None
    
    try:
        # Initialize hardware
        print("=== Starting IoT Sensor System ===")
        device = DeviceInit()
        
        # Calibrate sensor
        offsets = device.calibrate_sensor(cal_time=10)
        
        # Initialize data collector
        data_collector = DataCollector(device.imu, offsets)
        
        # Test WiFi credentials
        print("\n=== Checking Configuration ===")
        print(f"WiFi SSID: {config.wifi_ssid}")
        print(f"WiFi Password: {'*' * len(config.wifi_password)}")
        
        # Initialize WiFi and MQTT
        mqtt = MQTTInit(display_callback=device.display_message)
        
        if not mqtt.connect_wifi(config.wifi_ssid, config.wifi_password):
            device.display_message("WiFi Failed!\nCheck Config")
            print("\n!!! WiFi Connection Failed !!!")
            print("Please verify:")
            print("1. WiFi SSID is correct")
            print("2. WiFi password is correct")
            print("3. WiFi is 2.4GHz (Pico W doesn't support 5GHz)")
            print("4. Router is powered on and in range")
            # Don't raise error immediately - allow manual debugging
            sleep(30)
            raise RuntimeError("WiFi connection failed")
        
        sleep(1)
        
        if not mqtt.connect_mqtt(use_hivemq=True):
            device.display_message("MQTT Failed!")
            print("\n!!! MQTT Connection Failed !!!")
            sleep(10)
            raise RuntimeError("MQTT connection failed")
        
        sleep(1)
        
        # Initialize transmitter
        transmitter = Transmission(mqtt, display_callback=device.display_message)
        
        device.display_message("System Ready!")
        print("=== System Ready - Starting Data Loop ===")
        sleep(2)
        
        # Main data collection and transmission loop
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        while True:
            try:               
                # Check connections periodically
                if transmitter.message_count % 10 == 0:
                    if not mqtt.check_connection():
                        print("Connection lost, attempting reconnection...")
                        device.display_message("Reconnecting...")
                        
                        if not mqtt.reconnect_mqtt():
                            raise RuntimeError("Failed to reconnect")
                        
                        consecutive_errors = 0
                
                # Collect sensor data
                data, display_str = data_collector.get_formatted_data()
                print(f"\n{display_str}\nGX:{data['gx']} GY:{data['gy']} GZ:{data['gz']}")
                
                # Transmit data
                if transmitter.publish_sensor_data(data):
                    consecutive_errors = 0
                else:
                    consecutive_errors += 1
                
                # If too many consecutive errors, attempt recovery
                if consecutive_errors >= max_consecutive_errors:
                    print("Too many errors, attempting system recovery...")
                    device.display_message("Recovery...")
                    
                    if not mqtt.reconnect_mqtt():
                        raise RuntimeError("Recovery failed")
                    
                    consecutive_errors = 0
                
                sleep(SEND_DELAY)
                
            except KeyboardInterrupt:
                raise
            
            except Exception as e:
                print(f"Loop error: {e}")
                consecutive_errors += 1
                
                if consecutive_errors >= max_consecutive_errors:
                    raise RuntimeError("Too many consecutive errors")
                
                sleep(RETRY_DELAY)
    
    except KeyboardInterrupt:
        print("\n=== Shutdown requested ===")
        device.display_message("Shutdown...")
        
    except Exception as e:
        print(f"\n=== FATAL ERROR: {e} ===")
        if device:
            device.display_message("Fatal Error!\nCheck logs")
        print("\nSystem will remain halted for debugging.")
        print("Disconnect power to exit.")
        while True:
            sleep(60)
    
    finally:
        # Cleanup
        print("Cleaning up...")
        if mqtt and mqtt.client:
            try:
                mqtt.client.disconnect()
            except:
                pass
        
        if device:
            device.display_message("Stopped")
        
        print("=== System Stopped ===")


if __name__ == "__main__":
    main()