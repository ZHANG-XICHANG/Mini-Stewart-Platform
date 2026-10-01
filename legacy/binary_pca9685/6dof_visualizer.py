"""
6DOF Rotary Stewart Platform Visualizer

A professional-grade visualization tool for simulating a 6-DOF Stewart Platform
based on the C++ implementation from the physical controller.

Features:
- Accurate inverse kinematics matching the C++ controller implementation
- Real-time visualization of platform movement and servo angles
- Interactive position and orientation control
- Configurable arm lengths (L1, L2) for testing different geometry
- Detailed motion analytics and visualization
- Clean, modern interface with informative feedback

This visualizer serves as both a development tool and educational reference
for understanding the geometric principles of Stewart platforms.
"""

from stewart_kinematics import StewartKinematics
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, TextBox, Button, AxesWidget
from matplotlib.widgets import RadioButtons
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches
from math import pi, sin, cos, tan, asin, acos, atan2, sqrt, radians, degrees
import matplotlib.animation as animation
from matplotlib.lines import Line2D
import threading
import time
import struct

# Optional import of serial - won't crash if not available
HAS_SERIAL = False
try:
    import serial
    import serial.tools.list_ports  # For enumerating available COM ports
    HAS_SERIAL = True
except ImportError:
    print("PySerial not found. Install with 'pip install pyserial' for ESP32 communication.")

# Import configuration from stewart_config.py
from stewart_config import (
    BASE_RADIUS, PLATFORM_RADIUS, SERVO_ARM_LENGTH, CONNECTING_ARM_LENGTH,
    PLATFORM_HEIGHT, THETA_S, THETA_R, THETA_P, BASE_ANGLES, CCW_MOTORS
)

# ======================================================================
# ADJUSTABLE PARAMETERS - Modify these values to change arm lengths
# ======================================================================
# L1 - Servo arm length (mm)
L1 = SERVO_ARM_LENGTH  # Default is imported from config, change this value as needed

# L2 - Connecting arm length (mm)
L2 = CONNECTING_ARM_LENGTH  # Default is imported from config, change this value as needed
# ======================================================================

# Simple port selector using RadioButtons
class PortSelector:
    def __init__(self, ax, labels, title="COM Port", initial=0, callback=None):
        self.labels = labels if labels else ['None']
        self.callback = callback
        self.index = initial if initial < len(self.labels) else 0
        
        # Title for the selector
        if title:
            ax.text(0.5, 1.2, title, ha='center', va='center', transform=ax.transAxes)
        
        # Create the radio buttons
        self.radio = RadioButtons(ax, self.labels, active=self.index)
        
        # Connect callback if provided
        if callback:
            self.radio.on_clicked(callback)
        
    def set_labels(self, labels):
        """Update the available options"""
        if not labels:
            labels = ['None']
        
        # Store the active selection index for new list
        curr_selection = self.get_value()
        new_index = 0
        if curr_selection in labels:
            new_index = labels.index(curr_selection)
        
        # Update the labels and recreate the radio buttons
        self.labels = labels
        self.radio.labels = labels
        self.radio.circles = [plt.Circle((0.15, y), 0.05) for y in np.linspace(0.8, 0.2, len(self.labels))]
        
        # Set the active selection
        self.index = new_index
        self.radio.activeindex = new_index
        
        # Redraw
        self.radio.ax.figure.canvas.draw_idle()
    
    def get_value(self):
        """Get the currently selected value"""
        if self.radio.value_selected:
            return self.radio.value_selected
        return self.labels[self.index] if self.labels else 'None'
        

class SerialThread(threading.Thread):
    """Thread to send/receive platform position data to/from ESP32 over serial"""
    def __init__(self):
        threading.Thread.__init__(self)
        self.port = None
        self.baud_rate = 115200
        self.running = True
        self.ser = None
        self.daemon = True  # Thread will exit when main program exits
        self.connected = False
        self.lock = threading.Lock()  # Thread safety for serial operations
        
        # Data to be received from ESP debug output
        self.debug_data = None
        self.new_data_available = False
        self.data_callback = None  # Callback for when new data is received
        
        # Data to be sent to ESP32
        self.servo_angles_to_send = [0.0] * 6  # Store servo angles in degrees
        self.send_enabled = True  # Flag to enable/disable sending
        
        # Check if PySerial is actually available
        if not HAS_SERIAL:
            raise ImportError("PySerial module not available")

    def connect(self, port):
        """Connect to the specified port"""
        with self.lock:
            try:
                if self.connected:
                    self.disconnect()
                
                self.port = port
                # Configure serial port
                # - timeout=0: non-blocking read
                # - rtscts=True: hardware flow control if available
                # - dsrdtr=True: hardware flow control if available
                self.ser = serial.Serial(
                    port=self.port, 
                    baudrate=self.baud_rate, 
                    timeout=0,  # Non-blocking read
                    write_timeout=1  # 1 second write timeout
                )
                # Clear any existing data in the buffer
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.connected = True
                print(f"Connected to {self.port} at {self.baud_rate} baud")
                print("Waiting for ESP debug data...")
                return True
            except Exception as e:
                print(f"Failed to connect to {port}: {e}")
                self.connected = False
                return False
                
    def disconnect(self):
        """Disconnect from the current port"""
        with self.lock:
            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
                    print(f"Disconnected from {self.port}")
                self.connected = False
                return True
            except Exception as e:
                print(f"Error disconnecting: {e}")
                return False

    def set_data_callback(self, callback):
        """Set a callback function to be called when new data is received"""
        self.data_callback = callback
    
    def set_servo_angles(self, angles_deg):
        """Set servo angles to send to ESP32 (in degrees)"""
        with self.lock:
            self.servo_angles_to_send = angles_deg.copy() if len(angles_deg) == 6 else [0.0] * 6
    
    def enable_sending(self, enable):
        """Enable or disable sending data to ESP32"""
        self.send_enabled = enable
        
    def parse_debug_data(self, line):
        """Parse ESP debug output format
        Example: DEBUG,2678438784,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00
        """
        try:
            line = line.strip()
            if line.startswith('DEBUG'):
                parts = line.split(',')
                print(f"DEBUG parts count: {len(parts)}")
                if len(parts) >= 13:  # DEBUG + timestamp + at least 12 values
                    # Extract timestamp and values
                    timestamp = parts[1]
                    
                    # Parse all available values and log them individually
                    values = []
                    for i, val in enumerate(parts[2:14]):
                        try:
                            fval = float(val)
                            values.append(fval)
                            print(f"Value[{i}] = {fval}")
                        except ValueError as ve:
                            print(f"Could not parse value at index {i}: '{val}', error: {ve}")
                            values.append(0.0)
                    
                    # Make sure we have 12 values
                    while len(values) < 12:
                        values.append(0.0)
                    
                    # Store the parsed data
                    self.debug_data = {
                        'timestamp': timestamp,
                        'values': values,
                        'received_time': time.time()
                    }
                    self.new_data_available = True
                    
                    # Call the callback if set, but only store the data
                    # Don't manipulate UI from here - we're in a non-main thread
                    if self.data_callback:
                        self.data_callback(self.debug_data)
                    
                    return True
                else:
                    print(f"Not enough parts in DEBUG message: {line}")
            return False
        except Exception as e:
            print(f"Error parsing debug data: {e}")
            return False
            
    def get_latest_data(self):
        """Get the latest data received from ESP"""
        self.new_data_available = False
        return self.debug_data
            
    def run(self):
        try:
            # Main loop - keep sending/receiving data while running
            while self.running:
                try:
                    # Check for incoming data if connected
                    if self.connected and self.ser and self.ser.is_open:
                        # Check if data is available to read
                        with self.lock:
                            if self.ser.in_waiting > 0:
                                # Read a line from the serial port
                                line = self.ser.readline().decode('utf-8', errors='ignore')
                                if line:
                                    # Print the raw line to see what we're getting
                                    print(f"RAW DATA: {line.strip()}")
                                    # Parse the debug data
                                    if self.parse_debug_data(line):
                                        print(f"PARSED: Timestamp={self.debug_data['timestamp']}, Values={self.debug_data['values']}")
                                    
                        # Send servo angle data to ESP32
                        if self.send_enabled:
                            with self.lock:
                                # Convert angles from degrees to 0-4095 range
                                # Assuming ±60° maps to 0-4095, with 2047 as center (0°)
                                # Formula: value = 2047 + (angle * 4095 / 120)
                                data = []
                                for angle in self.servo_angles_to_send:
                                    # Clamp angle to ±60°
                                    clamped_angle = max(-60.0, min(60.0, angle))
                                    # Convert to 0-4095 range
                                    value = int(2047 + (clamped_angle * 4095 / 120.0))
                                    # Ensure within valid range
                                    value = max(0, min(4095, value))
                                    data.append(value)
                                
                                packed_data = struct.pack('>6H', *data)
                                self.ser.write(packed_data)
                    
                    # Sleep to maintain desired frequency
                    time.sleep(0.02)  # ~50Hz
                    
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    with self.lock:
                        self.connected = False
                    time.sleep(1)  # Wait before retrying
                    
                except Exception as e:
                    print(f"Error in serial thread loop: {e}")
                    time.sleep(1)  # Wait before retrying
                    
        except Exception as e:
            print(f"Unexpected error in serial thread: {e}")
        finally:
            # Always ensure the serial port is closed
            self.disconnect()

    def is_connected(self):
        """Check if currently connected"""
        return self.connected

    def get_available_ports(self):
        """Get a list of available serial ports"""
        if not HAS_SERIAL:
            return []
            
        ports = []
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
        except Exception as e:
            print(f"Error listing serial ports: {e}")
        return ports

    def stop(self):
        """Stop the thread and close the serial connection"""
        self.running = False
        self.disconnect()

class StewartPlatformVisualizer(StewartKinematics):
    def __init__(self):
        # Constants from config file - same as C code
        self.RD = BASE_RADIUS
        self.PD = PLATFORM_RADIUS
        self.L1 = L1  # Use the adjustable parameter
        self.L2 = L2  # Use the adjustable parameter
        
        # Initialize serial communication thread if PySerial is available
        self.serial_thread = None
        self.port_list = ['None']
        
        # Flag to indicate if we're in ESP debug data mode
        self.esp_data_mode = False
        # Last received ESP debug data
        self.last_esp_data = None
        # Flag to indicate new ESP data is available
        self.new_esp_data = False
        
        if HAS_SERIAL:
            try:
                # Create the serial thread but don't connect automatically
                self.serial_thread = SerialThread()
                # Set callback to update visualization when new data is received
                self.serial_thread.set_data_callback(self.on_esp_data_received)
                self.serial_thread.start()
                print("Serial thread started - waiting for connection")
                
                # Get available COM ports
                self.port_list = self.serial_thread.get_available_ports()
                if not self.port_list:
                    self.port_list = ['None']
            except Exception as e:
                print(f"Could not initialize serial thread: {e}")
                print("Visualizer will run without serial capability")
        else:
            print("PySerial not available - install with 'pip install pyserial' to enable ESP32 communication")
        self.platform_height = PLATFORM_HEIGHT
        self.theta_s = np.array(THETA_S)  # Servo angles in degrees
        self.theta_r = THETA_R  # Base rotation angle
        self.theta_p = THETA_P  # Platform rotation angle
        
        # Arrays directly from C code
        self.DxMultiplier = np.array([1, 1, 1, -1, -1, -1])
        self.AngleMultiplier = np.array([1, -1, 1, 1, -1, 1])
        
        # Convert pi/6, pi/6, -pi/2, -pi/2, pi/6, pi/6 to degrees for better readability
        self.OffsetAngle = np.array([
            pi/6,    # 30 degrees
            pi/6,    # 30 degrees
            -pi/2,   # -90 degrees
            -pi/2,   # -90 degrees
            pi/6,    # 30 degrees
            pi/6     # 30 degrees
        ])
        
        # Track movement boundaries
        self.motion_limits = {
            'max_z': 0,
            'min_z': 0,
            'max_angle': 0
        }
        
        # Track constraint information
        self.constraint_info = {
            'active': False,
            'type': None,  # 'angle', 'length', or 'math'
            'servo_index': -1,
            'value': 0,
            'limit': 0
        }
        
        # Track the last valid slider values
        self.last_valid_pos = [0, 0, 0]
        self.last_valid_rot = [0, 0, 0]
        
        # Flag to prevent recursive slider updates
        self.is_updating = False
        
        # Setup the figure and 3D axis with a more sophisticated layout
        self.fig = plt.figure(figsize=(14, 10))
        self.setup_layout()
        
        # Calculate the platform geometry
        self.calculate_geometry()
        
        # Create controls for position and orientation with more info display
        self.create_controls()
        
        # Create serial controls
        self.create_serial_controls()
        
        # Initial plot
        self.update(None)
    
    def setup_layout(self):
        """Create a professional layout with main view and info panels"""
        # Create grid for layout - with left servo angles panel
        from matplotlib.gridspec import GridSpec
        grid = GridSpec(3, 10, height_ratios=[6, 0.8, 1.2], width_ratios=[2.0, 0.1, 1, 1, 1, 1, 1, 1, 1, 1])
        
        # Servo angles table on the left
        self.servo_table_ax = self.fig.add_subplot(grid[0, 0])
        self.servo_table_ax.axis('off')
        
        # Main 3D view (spans remaining columns)
        self.ax = self.fig.add_subplot(grid[0, 2:], projection='3d')
        
        # Serial controls area (hidden subplot for text display)
        self.serial_controls_ax = self.fig.add_subplot(grid[1, :])
        self.serial_controls_ax.axis('off')
        
        # Info display area
        self.info_ax = self.fig.add_subplot(grid[2, :])
        self.info_ax.axis('off')
        
        # Adjust spacing
        self.fig.subplots_adjust(left=0.05, right=0.95, top=0.92, bottom=0.25, wspace=0.3)
        
        # Title
        self.fig.suptitle('6DOF Stewart Platform Visualizer', 
                        fontsize=14, fontweight='bold', y=0.98)
        
        # Config info (compact)
        config_text = f"R={self.RD}mm | L1={self.L1}mm | L2={self.L2}mm | H={self.platform_height:.1f}mm"
        self.fig.text(0.5, 0.94, config_text, ha='center', fontsize=9, color='gray')
    
    def create_serial_controls(self):
        """Create controls for serial port connection"""
        # Only create serial controls if PySerial is available
        if not HAS_SERIAL or not self.serial_thread:
            self.serial_controls_ax.text(0.5, 0.5, "PySerial not available - pip install pyserial", 
                                      ha='center', va='center', color='gray', fontsize=9)
            return
        
        # Status indicator (initially not connected)
        self.status_text = self.serial_controls_ax.text(0.5, 0.5, "● Disconnected", 
                                                       ha='center', va='center', color='red', fontsize=10)
        
        # All serial buttons on one row (y=0.21)
        btn_y = 0.21
        btn_h = 0.03
        
        # Port selection (wider with more vertical space)
        port_height = max(0.05, len(self.port_list) * 0.025)  # Dynamic height based on number of ports
        self.port_combo_ax = self.fig.add_axes([0.05, btn_y, 0.15, port_height])
        if not self.port_list or len(self.port_list) == 0:
            self.port_list = ['None']
        self.port_combo = RadioButtons(self.port_combo_ax, self.port_list)
        # Increase label spacing to prevent overlap
        for label in self.port_combo.labels:
            label.set_fontsize(9)
        
        # Refresh button (positioned below port list)
        refresh_y = btn_y - 0.01  # Slightly lower
        self.refresh_button_ax = self.fig.add_axes([0.21, refresh_y, 0.08, btn_h])
        self.refresh_button = Button(self.refresh_button_ax, '↻ Refresh')
        self.refresh_button.on_clicked(self.refresh_ports)
        
        # Connect button
        self.connect_button_ax = self.fig.add_axes([0.30, refresh_y, 0.08, btn_h])
        self.connect_button = Button(self.connect_button_ax, 'Connect')
        self.connect_button.on_clicked(self.toggle_connection)
        
        # Mode switch button (moved to bottom left corner)
        self.mode_button_ax = self.fig.add_axes([0.05, 0.02, 0.12, 0.03])
        self.mode_button = Button(self.mode_button_ax, 'Mode: Control')
        self.mode_button.on_clicked(self.toggle_mode)
        self.mode_button.ax.set_visible(False)
        
        # Mode info text (next to mode button)
        self.mode_info_text = self.serial_controls_ax.text(0.10, 0.05, "", 
                                    ha='left', va='center', fontsize=8, style='italic', color='blue')
    
    def center_all_sliders(self, event=None):
        """將所有滑桿重置到中心位置"""
        # 暫時停止更新以避免多次觸發
        old_updating = self.is_updating
        self.is_updating = True
        
        # 將所有滑桿設置為 0
        self.sliders['x'].set_val(0)
        self.sliders['y'].set_val(0)
        self.sliders['z'].set_val(0)
        self.sliders['roll'].set_val(0)
        self.sliders['pitch'].set_val(0)
        self.sliders['yaw'].set_val(0)
        
        # 恢復更新狀態並觸發一次更新
        self.is_updating = old_updating
        self.update(None)
        
        print("\n[Reset] All sliders centered to 0")
    
    def refresh_ports(self, event=None):
        """Refresh the list of available COM ports"""
        if self.serial_thread:
            # Get available ports
            ports = self.serial_thread.get_available_ports()
            if not ports:
                ports = ['None']
            
            # Save the current port list
            self.port_list = ports
            
            # Rebuild the RadioButtons widget with new ports
            self.port_combo_ax.clear()
            self.port_combo_ax.set_title("Port")
            self.port_combo = RadioButtons(self.port_combo_ax, ports)
            
            # Redraw
            self.fig.canvas.draw_idle()
            
    def toggle_mode(self, event=None):
        """切換發送/接收模式"""
        if not self.serial_thread or not self.serial_thread.is_connected():
            return
        
        if self.esp_data_mode:
            # 切換到控制模式（發送）
            self.esp_data_mode = False
            self.serial_thread.enable_sending(True)
            self.mode_button.label.set_text('Mode: Control')
            self.status_text.set_text("● Control Mode")
            self.status_text.set_color('blue')
            self.mode_info_text.set_text("Sending servo commands")
            self.mode_info_text.set_color('blue')
            print("\n[Mode] CONTROL - Sending servo commands")
        else:
            # 切換到監控模式（接收）
            self.esp_data_mode = True
            self.serial_thread.enable_sending(False)
            self.mode_button.label.set_text('Mode: Monitor')
            self.status_text.set_text("● Monitor Mode")
            self.status_text.set_color('green')
            self.mode_info_text.set_text("Receiving ESP32 data")
            self.mode_info_text.set_color('green')
            print("\n[Mode] MONITOR - Receiving ESP32 data")
        
        self.fig.canvas.draw_idle()
    
    def toggle_connection(self, event=None):
        """Toggle connection to the selected COM port"""
        if not self.serial_thread:
            return
            
        if self.serial_thread.is_connected():
            # Disconnect
            self.serial_thread.disconnect()
            self.connect_button.label.set_text('Connect')
            self.status_text.set_text("● Disconnected")
            self.status_text.set_color('red')
            self.mode_info_text.set_text("")
            self.mode_button.ax.set_visible(False)
            self.esp_data_mode = False
            print("\n✗ Disconnected")
        else:
            # Connect to selected port
            port = self.port_combo.value_selected
            if not port and len(self.port_list) > 0:
                port = self.port_list[0]  # Default to first port if none selected
                
            if port and port != 'None':
                if self.serial_thread.connect(port):
                    self.connect_button.label.set_text('Disconnect')
                    self.status_text.set_text("● Control Mode")
                    self.status_text.set_color('blue')
                    # 顯示模式切換按鈕
                    self.mode_button.ax.set_visible(True)
                    self.mode_button.label.set_text('Mode: Control')
                    # 顯示模式信息
                    self.mode_info_text.set_text("Sending servo commands")
                    self.mode_info_text.set_color('blue')
                    # 預設為發送模式
                    self.esp_data_mode = False
                    self.serial_thread.enable_sending(True)
                    print(f"\n✓ Connected to {port} | Control Mode")
                else:
                    self.status_text.set_text("● Connection failed")
                    self.status_text.set_color('red')
            else:
                self.status_text.set_text("Select a port first")
                self.status_text.set_color('orange')
                
        # Redraw the canvas
        self.fig.canvas.draw_idle()
    
    def create_controls(self):
        """Create sliders for controlling the platform position and orientation."""
        slider_color = 'lightgoldenrodyellow'
        self.axes = {}
        
        # Layout parameters
        slider_width = 0.20
        slider_height = 0.025
        left_x = 0.42  # XYZ sliders on left side with gap
        right_x = 0.70  # Roll/Pitch/Yaw on right side
        
        # Sliders positioned in two columns (y positions)
        y_top = 0.17
        y_mid = 0.13
        y_bot = 0.09
        
        # Left column: X, Y, Z
        self.axes['x'] = plt.axes([left_x, y_top, slider_width, slider_height], facecolor=slider_color)
        self.axes['y'] = plt.axes([left_x, y_mid, slider_width, slider_height], facecolor=slider_color)
        self.axes['z'] = plt.axes([left_x, y_bot, slider_width, slider_height], facecolor=slider_color)
        
        # Right column: Roll, Pitch, Yaw
        self.axes['roll'] = plt.axes([right_x, y_top, slider_width, slider_height], facecolor=slider_color)
        self.axes['pitch'] = plt.axes([right_x, y_mid, slider_width, slider_height], facecolor=slider_color)
        self.axes['yaw'] = plt.axes([right_x, y_bot, slider_width, slider_height], facecolor=slider_color)
        
        # Create sliders
        self.sliders = {}
        pos_range = 50  # Range in mm (reduced for practical use)
        self.sliders['x'] = Slider(self.axes['x'], 'X', -pos_range, pos_range, valinit=0)
        self.sliders['y'] = Slider(self.axes['y'], 'Y', -pos_range, pos_range, valinit=0)
        self.sliders['z'] = Slider(self.axes['z'], 'Z', -pos_range/2, pos_range/2, valinit=0)
        
        angle_range = 25  # Range in degrees
        self.sliders['roll'] = Slider(self.axes['roll'], 'Roll', -angle_range, angle_range, valinit=0)
        self.sliders['pitch'] = Slider(self.axes['pitch'], 'Pitch', -angle_range, angle_range, valinit=0)
        self.sliders['yaw'] = Slider(self.axes['yaw'], 'Yaw', -angle_range, angle_range, valinit=0)
        
        # Center button (positioned at top right of sliders)
        self.center_button_ax = self.fig.add_axes([0.42, 0.21, 0.10, 0.03])
        self.center_button = Button(self.center_button_ax, '↺ Center')
        self.center_button.on_clicked(self.center_all_sliders)
        
        # Set up the update function for all sliders
        for slider in self.sliders.values():
            slider.on_changed(self.update)
    
    
    
    
    def on_esp_data_received(self, data):
        """Callback when new ESP debug data is received"""
        if data and self.esp_data_mode:
            # Just store the data - we'll use it in the animation loop
            # which runs in the main thread
            self.last_esp_data = data
            # Flag that new data is available
            self.new_esp_data = True
            
    def update_from_esp_data(self):
        """Update the visualization based on ESP debug data"""
        if not self.last_esp_data:
            return False
            
        # Get the 12 values from ESP debug data
        # Format expected: DEBUG,timestamp,x,y,z,roll,pitch,yaw,s1,s2,s3,s4,s5,s6
        esp_values = self.last_esp_data['values']
        if len(esp_values) < 12:
            return False
            
        # First 6 values are position and rotation
        # Extract x, y, z, roll, pitch, yaw from ESP data
        pos = esp_values[0:3]  # First 3 values: x, y, z
        rot_deg = esp_values[3:6]  # Next 3 values: roll, pitch, yaw (in degrees)
        
        # Convert rotation from degrees to radians
        rot = [angle * pi/180 for angle in rot_deg]
        
        # Update sliders to match ESP values without triggering update cycle
        old_updating = self.is_updating
        self.is_updating = True
        self.sliders['x'].set_val(pos[0])
        self.sliders['y'].set_val(pos[1])
        self.sliders['z'].set_val(pos[2])
        self.sliders['roll'].set_val(rot_deg[0])
        self.sliders['pitch'].set_val(rot_deg[1])
        self.sliders['yaw'].set_val(rot_deg[2])
        self.is_updating = old_updating
        
        # Mark that data was used
        self.last_valid_pos = pos.copy()
        self.last_valid_rot = rot.copy()
        
        # Return success
        return True

    def update(self, val):
        """Update the visualization based on slider values or ESP data."""
        # Check if we're already in an update cycle to prevent recursion
        if self.is_updating:
            return
            
        # Set the update flag
        self.is_updating = True
        
        # If in ESP data mode, try to update from ESP data first
        if self.esp_data_mode and self.serial_thread and self.serial_thread.is_connected():
            esp_update_success = self.update_from_esp_data()
            # Only proceed with slider-based update if ESP update failed
            if esp_update_success:
                # Skip the rest of the update process
                result = self.calculate_servo_angles(self.last_valid_pos, self.last_valid_rot)
                # Reset the update flag at the end
                self.is_updating = False
                return
        
        # Get position and orientation values from sliders
        pos = [
            self.sliders['x'].val,
            self.sliders['y'].val,
            self.sliders['z'].val
        ]
        
        # Convert degrees to radians for rotation
        rot = [
            self.sliders['roll'].val * pi/180,
            self.sliders['pitch'].val * pi/180,
            self.sliders['yaw'].val * pi/180
        ]
        
        # Calculate servo angles and positions
        result = self.calculate_servo_angles(pos, rot)
        
        # If result is None, the position/orientation is invalid
        if result is None:
            # Restore sliders to last valid values
            self.sliders['x'].set_val(self.last_valid_pos[0])
            self.sliders['y'].set_val(self.last_valid_pos[1])
            self.sliders['z'].set_val(self.last_valid_pos[2])
            self.sliders['roll'].set_val(self.last_valid_rot[0] * 180/pi)
            self.sliders['pitch'].set_val(self.last_valid_rot[1] * 180/pi)
            self.sliders['yaw'].set_val(self.last_valid_rot[2] * 180/pi)
            
            # Get the values again (now should be valid)
            pos = self.last_valid_pos
            rot = self.last_valid_rot
            result = self.calculate_servo_angles(pos, rot)
        else:
            # Update the last valid position/orientation
            self.last_valid_pos = pos.copy()
            self.last_valid_rot = rot.copy()
        
        # Unpack the result
        servo_angles, platform_points, servo_arm_ends = result
        
        # Send servo angles to ESP32 if connected
        if self.serial_thread and self.serial_thread.is_connected() and not self.esp_data_mode:
            # Convert servo angles from radians to degrees
            servo_angles_deg = servo_angles * 180 / pi
            self.serial_thread.set_servo_angles(servo_angles_deg)
        
        # Clear previous plot
        self.ax.clear()
        self.info_ax.clear()
        self.info_ax.axis('off')
        
        # Convert servo angles to degrees for display
        servo_angles_deg = servo_angles * 180/pi
        
        # Draw the base plate (circular outline)
        theta = np.linspace(0, 2*pi, 60)
        base_x = self.RD * np.cos(theta)
        base_y = self.RD * np.sin(theta)
        base_z = np.zeros_like(theta)
        self.ax.plot(base_x, base_y, base_z, 'gray', linewidth=1)
        
        # Draw a simpler base plate (filled)
        base_x = np.append(base_x, 0)
        base_y = np.append(base_y, 0)
        base_z = np.append(base_z, 0)
        self.ax.plot_trisurf(base_x, base_y, base_z, color='lightgray', alpha=0.3)
        
        # Mark base points
        self.ax.scatter(self.base_coords_x, self.base_coords_y, np.zeros(6), 
                      color='blue', s=30, marker='o')
        
        # Draw the platform (hexagon)
        # Connect platform points in a single loop
        p_x = np.append(platform_points[:, 0], platform_points[0, 0])
        p_y = np.append(platform_points[:, 1], platform_points[0, 1])
        p_z = np.append(platform_points[:, 2], platform_points[0, 2])
        self.ax.plot(p_x, p_y, p_z, 'red', linewidth=1.5)
        
        # Create a filled platform surface
        platform_poly = Poly3DCollection([list(zip(platform_points[:, 0], 
                                               platform_points[:, 1], 
                                               platform_points[:, 2]))],
                                     alpha=0.3)
        platform_poly.set_facecolor('red')
        self.ax.add_collection3d(platform_poly)
        
        # Mark platform points
        self.ax.scatter(platform_points[:, 0], platform_points[:, 1], platform_points[:, 2], 
                      color='red', s=30, marker='o')
        
        # Plot servo arms and connecting rods
        connecting_rod_color = 'gold'
        for i in range(6):
            base_point = self.base_points[i]
            arm_end = servo_arm_ends[i]
            plat_point = platform_points[i]
            
            # Servo arm (green)
            self.ax.plot([base_point[0], arm_end[0]], 
                       [base_point[1], arm_end[1]], 
                       [base_point[2], arm_end[2]], 
                       'green', linewidth=2)
            
            # Connecting rod (gold)
            self.ax.plot([arm_end[0], plat_point[0]], 
                       [arm_end[1], plat_point[1]], 
                       [arm_end[2], plat_point[2]], 
                       color=connecting_rod_color, linewidth=1.5)
            
            # Display servo angle
            self.ax.text(base_point[0], base_point[1], base_point[2] + 5,
                       f"{servo_angles_deg[i]:.1f}°", 
                       color='black', size=8)
            
            # Add small coordinate markers at servo positions
            # X arrow
            self.ax.quiver(base_point[0], base_point[1], base_point[2],
                         3, 0, 0, color='red', arrow_length_ratio=0.1)
            # Y arrow
            self.ax.quiver(base_point[0], base_point[1], base_point[2],
                         0, 3, 0, color='green', arrow_length_ratio=0.1)
            
        # Add a coordinate system at the center of the platform
        center = np.mean(platform_points, axis=0)
        scale = min(self.RD, self.PD) * 0.2
        # X axis (red)
        self.ax.quiver(center[0], center[1], center[2], 
                     scale, 0, 0, color='red', arrow_length_ratio=0.15)
        # Y axis (green)
        self.ax.quiver(center[0], center[1], center[2], 
                     0, scale, 0, color='green', arrow_length_ratio=0.15)
        # Z axis (blue)
        self.ax.quiver(center[0], center[1], center[2], 
                     0, 0, scale, color='blue', arrow_length_ratio=0.15)
        
        # Coordinate labels
        self.ax.text(center[0] + scale*1.1, center[1], center[2], "X", color='red')
        self.ax.text(center[0], center[1] + scale*1.1, center[2], "Y", color='green')
        self.ax.text(center[0], center[1], center[2] + scale*1.1, "Z", color='blue')
        
        # Set up a better camera angle
        self.ax.view_init(elev=30, azim=45)
        
        # Set labels and title
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        
        # Show platform position and orientation
        platform_title = f"Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) mm"
        self.ax.set_title(platform_title)
        
        # Set axis limits with some padding
        max_range = max(self.RD, self.PD) * 1.5
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        self.ax.set_zlim(0, self.platform_height * 1.5)
        
        # Set aspect ratio to be equal
        self.ax.set_box_aspect([1, 1, 0.7])
        
        # Add detailed information in the info panel
        self.update_info_panel(servo_angles_deg)
        
        # Draw the figure
        self.fig.canvas.draw_idle()
        
        # Reset the update flag
        self.is_updating = False
    
    def update_info_panel(self, servo_angles_deg):
        """Update the information panel with servo angles and stats"""
        # Clear and update servo angles table on the left
        self.servo_table_ax.clear()
        self.servo_table_ax.axis('off')
        
        # Create servo angles table header
        table_title = "Servo Angles"
        if self.esp_data_mode and self.serial_thread and self.serial_thread.is_connected():
            if self.last_esp_data:
                time_diff = time.time() - self.last_esp_data['received_time']
                if time_diff < 1.0:
                    table_title = "[ESP] " + table_title
        
        self.servo_table_ax.text(0.5, 1.0, table_title, ha='center', va='top', 
                                fontsize=11, fontweight='bold')
        
        # Draw table with servo angles
        y_start = 0.90
        y_step = 0.14
        for i, angle in enumerate(servo_angles_deg):
            y_pos = y_start - i * y_step
            # Servo number (moved slightly inward)
            self.servo_table_ax.text(0.35, y_pos, f"S{i+1}:", ha='right', va='center', 
                                    fontsize=10, fontweight='bold')
            # Angle value (moved slightly inward)
            self.servo_table_ax.text(0.80, y_pos, f"{angle:.1f}°", ha='right', va='center', 
                                    fontsize=10, family='monospace')
            # Separator line
            if i < 5:
                self.servo_table_ax.plot([0.10, 0.90], [y_pos - y_step/2, y_pos - y_step/2], 
                                        'k-', alpha=0.2, linewidth=0.5)
        
        # Box around table
        self.servo_table_ax.plot([0.10, 0.90, 0.90, 0.10, 0.10], 
                                [0.02, 0.02, 0.95, 0.95, 0.02], 
                                'k-', linewidth=1.5)
        
        self.servo_table_ax.set_xlim(0, 1)
        self.servo_table_ax.set_ylim(0, 1)
                
        # Add platform orientation
        orientation_text = f"Roll: {self.sliders['roll'].val:.1f}°  Pitch: {self.sliders['pitch'].val:.1f}°  Yaw: {self.sliders['yaw'].val:.1f}°"
        self.info_ax.text(0.01, 0.7, orientation_text, fontsize=10)
        
        # Add motion limits information
        max_height = self.motion_limits['max_z']
        min_height = self.motion_limits['min_z']
        max_angle = self.motion_limits['max_angle']
        
        limits_text = f"Recorded Limits - Max Height: {max_height:.1f} mm  Min Height: {min_height:.1f} mm  Max Servo Angle: {max_angle:.1f}°"
        self.info_ax.text(0.5, 0.7, limits_text, fontsize=10, ha='center')
        
        # Add L1/L2 information
        geometry_text = f"Geometry: L1 (Servo Arm): {self.L1} mm  L2 (Connecting Rod): {self.L2} mm"
        self.info_ax.text(0.01, 0.4, geometry_text, fontsize=10)
        
        # Initialize constraint variables with default values
        constraint_text = ""
        constraint_color = 'green'
        
        # Add constraint information if a constraint was hit
        if self.constraint_info['active']:
            constraint_type = self.constraint_info['type']
            servo_index = self.constraint_info['servo_index']
            value = self.constraint_info['value']
            limit = self.constraint_info['limit']
            
            if constraint_type == 'angle':
                constraint_text = f"CONSTRAINT HIT: Servo {servo_index+1} angle ({value:.1f}°) exceeds limit (±{abs(limit)}°)"
                constraint_color = 'red'
            elif constraint_type == 'length':
                constraint_text = f"CONSTRAINT HIT: Servo {servo_index+1} connecting rod length ({value:.1f} mm) exceeds L2 ({limit} mm)"
                constraint_color = 'red'
            else:  # math error
                if servo_index >= 0:
                    constraint_text = f"CONSTRAINT HIT: Servo {servo_index+1} has no valid mathematical solution"
                else:
                    constraint_text = "CONSTRAINT HIT: No valid mathematical solution exists"
                constraint_color = 'red'
        
        # Only display constraint text if there is a constraint
        if constraint_text:
            self.info_ax.text(0.5, 0.1, constraint_text, fontsize=10, ha='center', color=constraint_color, 
                             weight='bold', bbox=dict(facecolor='yellow', alpha=0.3))

    def cleanup(self):
        """Stop the serial thread and cleanup resources."""
        if hasattr(self, 'serial_thread') and self.serial_thread is not None:
            print("Stopping serial thread...")
            self.serial_thread.stop()
            
    def show(self):
        """Show the visualizer and start the interaction."""
        try:
            # Set up animation to allow dynamic updates from ESP
            # This will create a smooth animation even when receiving data from ESP
            # Fix the warning by setting cache_frame_data=False and frames=None
            self.ani = animation.FuncAnimation(self.fig, self.animation_update, 
                                             frames=None, interval=50, blit=False,
                                             cache_frame_data=False)
            plt.show()
        finally:
            self.cleanup()
            
    def animation_update(self, frame):
        """Update function for animation - checks for new ESP data"""
        if self.esp_data_mode and self.serial_thread and self.serial_thread.is_connected():
            # Only trigger update if not already updating and we have new data
            if not self.is_updating and self.new_esp_data:
                self.new_esp_data = False  # Reset the flag
                self.update(None)

if __name__ == "__main__":
    # Create and show the 6DOF Stewart Platform visualizer
    visualizer = StewartPlatformVisualizer()
    visualizer.show()
