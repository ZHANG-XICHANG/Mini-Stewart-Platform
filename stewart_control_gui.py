"""
Stewart 平台控制 GUI
功能：
- 手動控制 6 軸 (X, Y, Z, Roll, Pitch, Yaw)
- 每個軸獨立正弦波測試
- 實時狀態監控
- by Darcy 2026-03-28
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import time
import math
import os
import random
import glob
from PIL import Image, ImageTk
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import collections

class StewartControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Stewart 平台控制面板")
        self.root.geometry("1600x900")  # 增加視窗寬度以容納波形顯示
        
        # 串口連接
        self.ser = None
        self.connected = False
        
        # 當前位置
        self.current_pos = {
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
        }
        
        # 正弦波參數
        self.sine_params = {
            'x': {'enabled': False, 'amplitude': 10.0, 'frequency': 0.5, 'phase': 0.0},
            'y': {'enabled': False, 'amplitude': 10.0, 'frequency': 0.5, 'phase': 0.0},
            'z': {'enabled': False, 'amplitude': 10.0, 'frequency': 0.5, 'phase': 0.0},
            'roll': {'enabled': False, 'amplitude': 10.0, 'frequency': 0.5, 'phase': 0.0},
            'pitch': {'enabled': False, 'amplitude': 10.0, 'frequency': 0.5, 'phase': 0.0},
            'yaw': {'enabled': False, 'amplitude': 10.0, 'frequency': 0.5, 'phase': 0.0}
        }
        
        # 正弦波執行緒
        self.sine_thread = None
        self.sine_running = False
        self.sine_start_time = 0
        
        # 實時發送設定
        self.live_send_enabled = False
        self.last_send_time = 0
        self.send_interval = 0.05  # 最小發送間隔 50ms (20Hz)
        self.live_send_before_sine = False  # 記住正弦波測試前的實時發送狀態
        
        # 正弦波測試基準位置（測試開始時的位置）
        self.sine_base_pos = {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0}
        
        # GIF 播放狀態
        self.gif_folder = os.path.join(os.path.dirname(__file__), 'GIF')  # GIF 資料夾路徑
        self.gif_files = []  # 所有可用的 GIF 檔案
        self.current_gif_frames = []  # 當前載入的 GIF 幀
        self.gif_frame_index = 0
        self.gif_playing = False
        self.gif_animation_id = None
        self.gif_delay = 100  # 預設延遲 100ms
        self.current_gif_name = ""  # 當前播放的 GIF 名稱
        
        # Logo 顯示
        self.logo_path = os.path.join(self.gif_folder, 'Logo.gif')  # Logo 檔案路徑（GIF 資料夾中）
        self.logo_frames = []  # Logo 的所有幀（支援動態 GIF）
        self.logo_frame_index = 0
        
        # 波形顯示數據（儲存最近5秒的數據）
        # 1500點可支援 300Hz × 5秒，向下相容所有頻率
        self.wave_data = {
            'time': collections.deque(maxlen=1500),  # 時間軸
            'x': collections.deque(maxlen=1500),
            'y': collections.deque(maxlen=1500),
            'z': collections.deque(maxlen=1500),
            'roll': collections.deque(maxlen=1500),
            'pitch': collections.deque(maxlen=1500),
            'yaw': collections.deque(maxlen=1500)
        }
        self.wave_data_lock = threading.Lock()  # 線程鎖保護波形數據
        self.wave_update_timer = None  # 波形更新定時器
        self.wave_needs_update = False  # 標記是否需要更新波形
        self.logo_animation_id = None
        self.logo_delay = 100
        self.is_showing_logo = False  # 是否正在顯示 Logo
        
        # 建立 UI
        self.create_widgets()
        
        # 更新可用 COM port
        self.refresh_ports()
        
        # 載入 Logo 並顯示
        self.load_logo()
        
        # 自動掃描 GIF 資料夾
        self.scan_gif_folder()
        
    def create_widgets(self):
        """建立所有 UI 元件"""
        
        # ====================
        # 連接區域
        # ====================
        conn_frame = ttk.LabelFrame(self.root, text="連接設定", padding=10)
        conn_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky="ew")
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(conn_frame, width=15, state='readonly')
        self.port_combo.grid(row=0, column=1, padx=5)
        
        ttk.Button(conn_frame, text="重新掃描", command=self.refresh_ports).grid(row=0, column=2, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="連接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        self.status_label = ttk.Label(conn_frame, text="未連接", foreground="red")
        self.status_label.grid(row=0, column=4, padx=10)
        
        # ====================
        # 手動控制區域
        # ====================
        manual_frame = ttk.LabelFrame(self.root, text="手動控制", padding=10)
        manual_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        
        self.sliders = {}
        self.value_labels = {}
        
        # 定義軸參數 (名稱, 最小值, 最大值, 初始值)
        axes = [
            ('x', -50, 50, 0, 'mm'),
            ('y', -50, 50, 0, 'mm'),
            ('z', -50, 50, 0, 'mm'),
            ('roll', -30, 30, 0, '°'),
            ('pitch', -30, 30, 0, '°'),
            ('yaw', -30, 30, 0, '°')
        ]
        
        for i, (axis, min_val, max_val, init_val, unit) in enumerate(axes):
            # 軸名稱
            ttk.Label(manual_frame, text=f"{axis.upper()}:", width=6).grid(row=i, column=0, padx=5, pady=5)
            
            # 滑桿
            slider = tk.Scale(manual_frame, from_=min_val, to=max_val, 
                            orient=tk.HORIZONTAL, length=350, resolution=0.1,
                            command=lambda val, a=axis: self.on_slider_change(a, val))
            slider.set(init_val)
            slider.grid(row=i, column=1, padx=5, pady=5)
            self.sliders[axis] = slider
            
            # 數值顯示
            value_label = ttk.Label(manual_frame, text=f"{init_val:.1f} {unit}", width=10)
            value_label.grid(row=i, column=2, padx=5, pady=5)
            self.value_labels[axis] = (value_label, unit)
            
        # 實時發送核取方塊
        live_send_frame = ttk.Frame(manual_frame)
        live_send_frame.grid(row=6, column=0, columnspan=3, pady=5)
        
        self.live_send_var = tk.BooleanVar()
        live_send_check = ttk.Checkbutton(live_send_frame, text="實時發送（拖動滑桿即時更新）",
                                         variable=self.live_send_var,
                                         command=self.on_live_send_toggle)
        live_send_check.pack()
        
        # 控制按鈕
        btn_frame = ttk.Frame(manual_frame)
        btn_frame.grid(row=7, column=0, columnspan=3, pady=10)
        
        self.reset_btn = ttk.Button(btn_frame, text="回中心", command=self.reset_position)
        self.reset_btn.pack(side=tk.LEFT, padx=5)
        self.manual_send_btn = ttk.Button(btn_frame, text="發送位置", command=self.send_current_position)
        self.manual_send_btn.pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="緊急停止", command=self.emergency_stop, 
                  style='Danger.TButton').pack(side=tk.LEFT, padx=5)
        
        # ====================
        # 正弦波測試區域
        # ====================
        sine_frame = ttk.LabelFrame(self.root, text="正弦波測試", padding=10)
        sine_frame.grid(row=1, column=1, padx=10, pady=5, sticky="nsew")
        
        # 表頭
        ttk.Label(sine_frame, text="軸", font=('Arial', 9, 'bold')).grid(row=0, column=0, padx=5)
        ttk.Label(sine_frame, text="啟用", font=('Arial', 9, 'bold')).grid(row=0, column=1, padx=5)
        ttk.Label(sine_frame, text="振幅", font=('Arial', 9, 'bold')).grid(row=0, column=2, padx=5)
        ttk.Label(sine_frame, text="頻率(Hz)", font=('Arial', 9, 'bold')).grid(row=0, column=3, padx=5)
        ttk.Label(sine_frame, text="相位(°)", font=('Arial', 9, 'bold')).grid(row=0, column=4, padx=5)
        
        self.sine_checks = {}
        self.sine_checkbuttons = {}  # 保存复选框对象用于禁用/启用
        self.sine_amp_entries = {}
        self.sine_freq_entries = {}
        self.sine_phase_entries = {}
        
        for i, axis in enumerate(['x', 'y', 'z', 'roll', 'pitch', 'yaw']):
            row = i + 1
            
            # 軸名稱
            ttk.Label(sine_frame, text=axis.upper()).grid(row=row, column=0, padx=5, pady=3)
            
            # 啟用核取方塊
            var = tk.BooleanVar()
            check = ttk.Checkbutton(sine_frame, variable=var, 
                                   command=lambda a=axis: self.on_sine_toggle(a))
            check.grid(row=row, column=1, padx=5, pady=3)
            self.sine_checks[axis] = var
            self.sine_checkbuttons[axis] = check
            
            # 振幅輸入
            amp_entry = ttk.Entry(sine_frame, width=8)
            amp_entry.insert(0, "10.0")
            amp_entry.grid(row=row, column=2, padx=5, pady=3)
            self.sine_amp_entries[axis] = amp_entry
            
            # 頻率輸入
            freq_entry = ttk.Entry(sine_frame, width=8)
            freq_entry.insert(0, "0.5")
            freq_entry.grid(row=row, column=3, padx=5, pady=3)
            self.sine_freq_entries[axis] = freq_entry
            
            # 相位輸入
            phase_entry = ttk.Entry(sine_frame, width=8)
            phase_entry.insert(0, "0")
            phase_entry.grid(row=row, column=4, padx=5, pady=3)
            self.sine_phase_entries[axis] = phase_entry
        
        # 正弦波控制按鈕
        sine_btn_frame = ttk.Frame(sine_frame)
        sine_btn_frame.grid(row=7, column=0, columnspan=5, pady=10)
        
        self.sine_start_btn = ttk.Button(sine_btn_frame, text="開始測試", 
                                        command=self.start_sine_test)
        self.sine_start_btn.pack(side=tk.LEFT, padx=5)
        
        self.sine_stop_btn = ttk.Button(sine_btn_frame, text="停止測試", 
                                       command=self.stop_sine_test, state=tk.DISABLED)
        self.sine_stop_btn.pack(side=tk.LEFT, padx=5)
        
        # 更新頻率設定
        ttk.Label(sine_frame, text="更新頻率:").grid(row=8, column=0, columnspan=2, padx=5, pady=5)
        self.update_freq_combo = ttk.Combobox(sine_frame, width=10, state='readonly',
                                             values=['10 Hz', '20 Hz', '50 Hz', '100 Hz', '200 Hz', '250 Hz', '300 Hz'])
        self.update_freq_combo.set('50 Hz')
        self.update_freq_combo.grid(row=8, column=2, columnspan=3, padx=5, pady=5)
        
        # ====================
        # 狀態監控區域（左下角）
        # ====================
        status_frame = ttk.LabelFrame(self.root, text="狀態監控", padding=10)
        status_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")
        
        # 滾動條（與 Text widget 並排）
        scrollbar = ttk.Scrollbar(status_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Text widget 固定高度，不使用 expand
        self.status_text = tk.Text(status_frame, height=15, width=70, state=tk.DISABLED, 
                                   yscrollcommand=scrollbar.set, wrap=tk.WORD)
        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH)
        scrollbar.config(command=self.status_text.yview)
        
        # ====================
        # GIF 播放區域（右下角）
        # ====================
        gif_frame = ttk.LabelFrame(self.root, text="狀態", padding=10)
        gif_frame.grid(row=2, column=1, padx=10, pady=5, sticky="nsew")
        
        # GIF/Logo 顯示標籤
        self.gif_label = tk.Label(gif_frame, text="載入中...", anchor=tk.CENTER, 
                                  background='#f0f0f0', relief=tk.SUNKEN, font=('Arial', 12))
        self.gif_label.pack(expand=True, fill=tk.BOTH, padx=5, pady=5)
        
        # GIF 狀態顯示（已隱藏）
        # self.gif_status_label = ttk.Label(gif_frame, text="GIF 資料夾：掃描中...", 
        #                                  font=('Arial', 9), foreground='gray')
        # self.gif_status_label.pack(pady=3)
        
        # ====================
        # 波形顯示區域
        # ====================
        wave_frame = ttk.LabelFrame(self.root, text="波形顯示", padding=10)
        wave_frame.grid(row=1, column=2, rowspan=2, padx=10, pady=5, sticky="nsew")
        
        # 創建 matplotlib 圖形
        self.fig = Figure(figsize=(6, 6), dpi=80, facecolor='white')
        self.ax = self.fig.add_subplot(111)
        
        # 設定中文字體（如果可用）
        try:
            import matplotlib.pyplot as plt
            plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei', 'SimHei', 'Arial']
            plt.rcParams['axes.unicode_minus'] = False
        except:
            pass
        
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position/Angle')
        self.ax.set_title('Waveform Display')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, 5)
        self.ax.set_ylim(-50, 50)
        
        # 顯示初始提示
        self.ax.text(2.5, 0, 'Waiting for test...', 
                    ha='center', va='center', fontsize=14, color='gray', alpha=0.5)
        
        # 創建畫布
        self.canvas = FigureCanvasTkAgg(self.fig, master=wave_frame)
        self.canvas.draw()
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.pack(fill=tk.BOTH, expand=True)
        
        # 記錄狀態
        self.log_status("波形顯示區域已初始化")
        
        # 波形線條（初始化為空）
        self.wave_lines = {}
        self.wave_colors = {
            'x': '#FF0000',      # 紅
            'y': '#00AA00',      # 綠
            'z': '#0000FF',      # 藍
            'roll': '#FF8800',   # 橙
            'pitch': '#AA00AA',  # 紫
            'yaw': '#00AAAA'     # 青
        }
        
        # 配置網格權重
        self.root.grid_rowconfigure(0, weight=0)  # 連接區域：固定高度
        self.root.grid_rowconfigure(1, weight=2)  # 主控制區域：較大權重
        self.root.grid_rowconfigure(2, weight=2)  # 狀態監控：增大權重
        self.root.grid_columnconfigure(0, weight=2)  # 手動控制：較大（滑桿需要空間）
        self.root.grid_columnconfigure(1, weight=1)  # 正弦波測試：較小
        self.root.grid_columnconfigure(2, weight=2)  # 波形顯示：較大
        
    def log_status(self, message):
        """記錄狀態訊息（自動限制最多 200 條）"""
        self.status_text.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.status_text.insert(tk.END, f"[{timestamp}] {message}\n")
        
        # 自動刪除舊訊息，只保留最近 200 條
        lines = int(self.status_text.index('end-1c').split('.')[0])
        if lines > 200:
            self.status_text.delete('1.0', f'{lines - 200}.0')
        
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
        
    def refresh_ports(self):
        """重新掃描可用的 COM ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
            self.log_status(f"找到 {len(ports)} 個 COM port")
        else:
            self.log_status("未找到可用的 COM port")
            
    def toggle_connection(self):
        """切換連接狀態"""
        if not self.connected:
            self.connect()
        else:
            self.disconnect()
            
    def connect(self):
        """連接到 ESP32"""
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("錯誤", "請選擇 COM port")
            return
            
        try:
            self.ser = serial.Serial(port, 460800, timeout=0.1)
            time.sleep(2)  # 等待 ESP32 重啟
            
            # 設定為靜默模式
            self.ser.write(b"$V,0\n")
            time.sleep(0.1)
            self.ser.reset_input_buffer()
            
            self.connected = True
            self.status_label.config(text="已連接", foreground="green")
            self.connect_btn.config(text="斷開")
            self.log_status(f"已連接到 {port}")
            
        except Exception as e:
            messagebox.showerror("連接錯誤", f"無法連接到 {port}\n{str(e)}")
            self.log_status(f"連接失敗: {str(e)}")
            
    def disconnect(self):
        """斷開連接"""
        if self.ser:
            # 停止正弦波測試
            if self.sine_running:
                self.stop_sine_test()
            
            # 停用實時發送
            if self.live_send_enabled:
                self.live_send_var.set(False)
                self.live_send_enabled = False
                self.manual_send_btn.config(state=tk.NORMAL)
            
            # 回到中心位置
            self.send_position(0, 0, 0, 0, 0, 0)
            time.sleep(0.1)
            
            self.ser.close()
            self.ser = None
            
        self.connected = False
        self.status_label.config(text="未連接", foreground="red")
        self.connect_btn.config(text="連接")
        self.log_status("已斷開連接")
        
    def calculate_checksum(self, values):
        """計算 checksum"""
        formatted_values = [round(v, 2) for v in values]
        checksum = sum(int(v * 10) for v in formatted_values) & 0xFF
        return checksum
        
    def send_position(self, x, y, z, roll, pitch, yaw):
        """發送位置到 ESP32"""
        if not self.connected or not self.ser:
            return False
            
        try:
            # 四捨五入到兩位小數
            x, y, z = round(x, 2), round(y, 2), round(z, 2)
            roll, pitch, yaw = round(roll, 2), round(pitch, 2), round(yaw, 2)
            
            # 計算 checksum
            values = [x, y, z, roll, pitch, yaw]
            checksum = self.calculate_checksum(values)
            
            # 建立封包
            packet = f"$P,{x:.2f},{y:.2f},{z:.2f},{roll:.2f},{pitch:.2f},{yaw:.2f},{checksum}\n"
            
            # 發送
            self.ser.write(packet.encode('utf-8'))
            self.ser.flush()
            
            # 清空接收緩衝區（靜默模式下應該沒有回應）
            if self.ser.in_waiting > 0:
                self.ser.reset_input_buffer()
            
            return True
            
        except Exception as e:
            self.log_status(f"發送錯誤: {str(e)}")
            return False
    
    def send_position_silent(self):
        """靜默發送位置（不記錄日誌）"""
        if not self.connected or not self.ser:
            return False
        
        return self.send_position(
            self.current_pos['x'],
            self.current_pos['y'],
            self.current_pos['z'],
            self.current_pos['roll'],
            self.current_pos['pitch'],
            self.current_pos['yaw']
        )
    
    def on_live_send_toggle(self):
        """實時發送開關切換"""
        self.live_send_enabled = self.live_send_var.get()
        
        if self.live_send_enabled:
            if not self.connected:
                messagebox.showwarning("警告", "請先連接設備")
                self.live_send_var.set(False)
                self.live_send_enabled = False
                return
            
            # 啟用實時發送時，發送一次當前位置
            self.send_position_silent()
            self.last_send_time = time.time()
            self.manual_send_btn.config(state=tk.DISABLED)
            self.log_status("✅ 實時發送已啟用（20Hz，拖動滑桿即時更新）")
        else:
            self.manual_send_btn.config(state=tk.NORMAL)
            self.log_status("實時發送已停用")
            
    def on_slider_change(self, axis, value):
        """滑桿值改變時"""
        value = float(value)
        self.current_pos[axis] = value
        
        # 更新顯示
        label, unit = self.value_labels[axis]
        label.config(text=f"{value:.1f} {unit}")
        
        # 實時發送（帶節流）
        if self.live_send_enabled and self.connected:
            current_time = time.time()
            if current_time - self.last_send_time >= self.send_interval:
                self.send_position_silent()
                self.last_send_time = current_time
        
    def send_current_position(self):
        """發送當前滑桿位置"""
        if not self.connected:
            messagebox.showwarning("警告", "請先連接設備")
            return
            
        success = self.send_position(
            self.current_pos['x'],
            self.current_pos['y'],
            self.current_pos['z'],
            self.current_pos['roll'],
            self.current_pos['pitch'],
            self.current_pos['yaw']
        )
        
        if success:
            self.log_status(f"已發送位置: X={self.current_pos['x']:.1f}, Y={self.current_pos['y']:.1f}, "
                          f"Z={self.current_pos['z']:.1f}, R={self.current_pos['roll']:.1f}, "
                          f"P={self.current_pos['pitch']:.1f}, Y={self.current_pos['yaw']:.1f}")
        
    def reset_position(self):
        """回到中心位置"""
        for axis, slider in self.sliders.items():
            slider.set(0)
            self.current_pos[axis] = 0.0
            label, unit = self.value_labels[axis]
            label.config(text=f"0.0 {unit}")
            
        self.send_current_position()
        self.log_status("已重置到中心位置")
        
    def emergency_stop(self):
        """緊急停止"""
        # 停止正弦波
        if self.sine_running:
            self.stop_sine_test()
        
        # 停用實時發送
        if self.live_send_enabled:
            self.live_send_var.set(False)
            self.live_send_enabled = False
            self.manual_send_btn.config(state=tk.NORMAL)
        
        # 回中心
        self.reset_position()
        
        self.log_status("⚠️ 緊急停止！已回到中心位置")
        
    def on_sine_toggle(self, axis):
        """正弦波核取方塊切換"""
        enabled = self.sine_checks[axis].get()
        self.sine_params[axis]['enabled'] = enabled
        
    def start_sine_test(self):
        """開始正弦波測試"""
        if not self.connected:
            messagebox.showwarning("警告", "請先連接設備")
            return
            
        # 檢查是否有啟用的軸
        enabled_axes = [axis for axis in self.sine_params if self.sine_params[axis]['enabled']]
        if not enabled_axes:
            messagebox.showwarning("警告", "請至少啟用一個軸")
            return
            
        # 讀取參數
        try:
            for axis in enabled_axes:
                amp = float(self.sine_amp_entries[axis].get())
                freq = float(self.sine_freq_entries[axis].get())
                phase = float(self.sine_phase_entries[axis].get())
                self.sine_params[axis]['amplitude'] = amp
                self.sine_params[axis]['frequency'] = freq
                self.sine_params[axis]['phase'] = phase
        except ValueError:
            messagebox.showerror("錯誤", "振幅、頻率和相位必須是數字")
            return
            
        # 暫時停用實時發送（避免衝突）
        self.live_send_before_sine = self.live_send_enabled
        if self.live_send_enabled:
            self.live_send_var.set(False)
            self.live_send_enabled = False
            self.manual_send_btn.config(state=tk.NORMAL)
        
        # 禁用所有手動控制（防止與正弦波衝突）
        for slider in self.sliders.values():
            slider.config(state=tk.DISABLED)
        self.manual_send_btn.config(state=tk.DISABLED)
        if hasattr(self, 'reset_btn'):
            self.reset_btn.config(state=tk.DISABLED)
        
        # 禁用所有正弦波參數控制（防止測試中修改參數）
        for axis in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
            self.sine_checkbuttons[axis].config(state=tk.DISABLED)
            self.sine_amp_entries[axis].config(state=tk.DISABLED)
            self.sine_freq_entries[axis].config(state=tk.DISABLED)
            self.sine_phase_entries[axis].config(state=tk.DISABLED)
        self.update_freq_combo.config(state=tk.DISABLED)
        
        # 保存當前位置作為正弦波基準位置
        self.sine_base_pos = self.current_pos.copy()
        
        # 清空波形數據（使用線程鎖保護）
        with self.wave_data_lock:
            for key in self.wave_data:
                self.wave_data[key].clear()
            self.wave_needs_update = False
        
        # 初始化波形線條
        self.ax.clear()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position/Angle')
        self.ax.set_title('Sine Wave Test')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, 5)
        self.ax.set_ylim(-50, 50)
        
        # 創建波形線條
        self.wave_lines = {}
        for axis in enabled_axes:
            line, = self.ax.plot([], [], label=axis.upper(), 
                                color=self.wave_colors[axis], linewidth=2)
            self.wave_lines[axis] = line
        
        self.ax.legend(loc='upper right')
        self.canvas.draw()
        
        self.log_status(f"波形顯示已啟動，監控軸: {', '.join([a.upper() for a in enabled_axes])}")
        
        # 開始測試（必須在啟動定時器之前設置！）
        self.sine_running = True
        self.sine_start_time = time.time()
        
        # 啟動波形更新定時器（每 100ms 更新一次）
        self.start_waveform_timer()
        
        self.sine_start_btn.config(state=tk.DISABLED)
        self.sine_stop_btn.config(state=tk.NORMAL)
        
        # 随機播放 GIF（如果有可用的 GIF）
        if self.gif_files:
            self.play_random_gif()
        
        # 啟動執行緒
        self.sine_thread = threading.Thread(target=self.sine_test_loop, daemon=True)
        self.sine_thread.start()
        
        axes_str = ", ".join([a.upper() for a in enabled_axes])
        self.log_status(f"開始正弦波測試: {axes_str}")
        
    def stop_sine_test(self):
        """停止正弦波測試"""
        self.sine_running = False
        
        # 停止波形更新定時器
        self.stop_waveform_timer()
        
        if self.sine_thread:
            self.sine_thread.join(timeout=1.0)
            
        self.sine_start_btn.config(state=tk.NORMAL)
        self.sine_stop_btn.config(state=tk.DISABLED)
        
        # 重新啟用所有手動控制
        for slider in self.sliders.values():
            slider.config(state=tk.NORMAL)
        if hasattr(self, 'reset_btn'):
            self.reset_btn.config(state=tk.NORMAL)
        
        # 重新啟用所有正弦波參數控制
        for axis in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
            self.sine_checkbuttons[axis].config(state=tk.NORMAL)
            self.sine_amp_entries[axis].config(state=tk.NORMAL)
            self.sine_freq_entries[axis].config(state=tk.NORMAL)
            self.sine_phase_entries[axis].config(state=tk.NORMAL)
        self.update_freq_combo.config(state='readonly')
        
        # 清空波形顯示
        self.ax.clear()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Position/Angle')
        self.ax.set_title('Waveform Display')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlim(0, 5)
        self.ax.set_ylim(-50, 50)
        self.ax.text(0.5, 0.5, 'Test Stopped', 
                    transform=self.ax.transAxes, 
                    ha='center', va='center', 
                    fontsize=20, color='gray', alpha=0.5)
        self.canvas.draw()
        
        # 回到滑桿位置
        self.send_current_position()
        
        # 停止 GIF 播放並恢復 Logo
        if self.gif_playing:
            self.stop_gif()
            self.show_logo()  # 恢復顯示 Logo
        
        # 恢復實時發送狀態
        if self.live_send_before_sine and self.connected:
            self.live_send_var.set(True)
            self.live_send_enabled = True
            self.manual_send_btn.config(state=tk.DISABLED)
            self.log_status("正弦波測試已停止，已恢復手動控制和實時發送")
        else:
            self.manual_send_btn.config(state=tk.NORMAL)
            self.log_status("正弦波測試已停止，已恢復手動控制")
        
        self.live_send_before_sine = False
        
    def sine_test_loop(self):
        """正弦波測試循環（在獨立執行緒中運行）"""
        # 取得更新頻率
        freq_str = self.update_freq_combo.get()
        hz = int(freq_str.split()[0])
        interval = 1.0 / hz
        
        packet_count = 0
        
        while self.sine_running:
            loop_start = time.time()
            elapsed = time.time() - self.sine_start_time
            
            # 使用測試開始時的基準位置（忽略手動控制的變化）
            x = self.sine_base_pos['x']
            y = self.sine_base_pos['y']
            z = self.sine_base_pos['z']
            roll = self.sine_base_pos['roll']
            pitch = self.sine_base_pos['pitch']
            yaw = self.sine_base_pos['yaw']
            
            # 應用正弦波（啟用的軸會被正弦波覆蓋）
            for axis in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                if self.sine_params[axis]['enabled']:
                    amp = self.sine_params[axis]['amplitude']
                    freq = self.sine_params[axis]['frequency']
                    phase = self.sine_params[axis]['phase']
                    # 將相位從度轉換為弧度
                    phase_rad = phase * math.pi / 180.0
                    value = amp * math.sin(2 * math.pi * freq * elapsed + phase_rad)
                    
                    if axis == 'x':
                        x = value
                    elif axis == 'y':
                        y = value
                    elif axis == 'z':
                        z = value
                    elif axis == 'roll':
                        roll = value
                    elif axis == 'pitch':
                        pitch = value
                    elif axis == 'yaw':
                        yaw = value
            
            # 發送
            self.send_position(x, y, z, roll, pitch, yaw)
            packet_count += 1
            
            # 收集波形數據（使用線程鎖保護）
            with self.wave_data_lock:
                self.wave_data['time'].append(elapsed)
                self.wave_data['x'].append(x)
                self.wave_data['y'].append(y)
                self.wave_data['z'].append(z)
                self.wave_data['roll'].append(roll)
                self.wave_data['pitch'].append(pitch)
                self.wave_data['yaw'].append(yaw)
                self.wave_needs_update = True  # 標記需要更新
            
            # 每 50 個封包記錄一次
            if packet_count % 50 == 0:
                self.log_status(f"正弦波運行中... 已發送 {packet_count} 個封包 ({hz} Hz)")
            
            # 控制頻率
            loop_time = time.time() - loop_start
            sleep_time = max(0, interval - loop_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def start_waveform_timer(self):
        """啟動波形更新定時器"""
        if self.sine_running:
            self.update_waveform()
            # 每 100ms 更新一次
            self.wave_update_timer = self.root.after(100, self.start_waveform_timer)
        else:
            # 測試停止，取消定時器
            self.wave_update_timer = None
    
    def stop_waveform_timer(self):
        """停止波形更新定時器"""
        if self.wave_update_timer:
            self.root.after_cancel(self.wave_update_timer)
            self.wave_update_timer = None
    
    def update_waveform(self):
        """更新波形顯示"""
        if not self.sine_running:
            return
        
        # 如果沒有新數據，跳過更新
        if not self.wave_needs_update:
            return
        
        try:
            # 使用線程鎖讀取數據
            with self.wave_data_lock:
                data_len = len(self.wave_data['time'])
                if data_len == 0:
                    return
                time_data = list(self.wave_data['time'])
                wave_data_copy = {axis: list(self.wave_data[axis]) 
                                 for axis in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']}
                self.wave_needs_update = False
            
            # 每100個數據點記錄一次（用於調試）
            if data_len % 100 == 0:
                self.log_status(f"波形更新：已收集 {data_len} 個數據點")
            
            # 更新每條波形線
            for axis, line in self.wave_lines.items():
                if self.sine_params[axis]['enabled']:
                    line.set_data(time_data, wave_data_copy[axis])
            
            # 動態調整座標範圍
            if len(time_data) > 0:
                # X 軸：顯示最近 5 秒
                max_time = max(time_data)
                self.ax.set_xlim(max(0, max_time - 5), max_time + 0.5)
                
                # Y 軸：根據實際數據自動調整
                all_values = []
                for axis in self.wave_lines.keys():
                    if self.sine_params[axis]['enabled']:
                        all_values.extend(wave_data_copy[axis])
                
                if all_values:
                    min_val = min(all_values)
                    max_val = max(all_values)
                    margin = (max_val - min_val) * 0.1 if max_val > min_val else 10
                    self.ax.set_ylim(min_val - margin, max_val + margin)
            
            # 更新顯示
            self.canvas.draw_idle()
            
        except Exception as e:
            # 忽略更新錯誤，避免影響測試
            pass
    
    # ====================
    # Logo 和 GIF 顯示功能
    # ====================
    def load_logo(self):
        """載入並顯示 Logo（支援動態 GIF）"""
        # 檢查 Logo 檔案是否存在
        if os.path.exists(self.logo_path):
            try:
                # 載入 Logo GIF
                gif = Image.open(self.logo_path)
                self.logo_frames = []
                
                # 取得延遲時間
                try:
                    self.logo_delay = gif.info.get('duration', 100)
                except:
                    self.logo_delay = 100
                
                # 提取所有幀
                try:
                    while True:
                        frame = gif.copy()
                        frame.thumbnail((400, 300), Image.Resampling.LANCZOS)
                        photo = ImageTk.PhotoImage(frame)
                        self.logo_frames.append(photo)
                        gif.seek(len(self.logo_frames))
                except EOFError:
                    pass  # 已讀取所有幀
                
                if self.logo_frames:
                    # 顯示 Logo
                    self.show_logo()
                    frame_count = len(self.logo_frames)
                    # if frame_count > 1:
                    #     self.log_status(f"已載入動態 Logo：{os.path.basename(self.logo_path)} ({frame_count} 幀)")
                    # else:
                    #     self.log_status(f"已載入 Logo：{os.path.basename(self.logo_path)}")
                else:
                    raise Exception("無法讀取 Logo 幀")
                
            except Exception as e:
                self.log_status(f"載入 Logo 失敗：{str(e)}")
                self.show_default_text()
        else:
            # 如果沒有 Logo 檔案，顯示預設文字
            self.show_default_text()
            self.log_status("未找到 Logo.gif，顯示預設畫面（請在 GIF 資料夾放入 Logo.gif）")
    
    def show_default_text(self):
        """顯示預設文字（當沒有 Logo 時）"""
        self.gif_label.config(
            text="🎮 Stewart Platform 🎮\n\n控制系統\n\n請在 GIF 資料夾放入 Logo.gif\n正弦波測試時會隨機播放其他 GIF 🎁", 
            font=('Arial', 12, 'bold'),
            foreground='#3498db',
            image=''
        )
    
    def show_logo(self):
        """顯示 Logo（測試結束後恢復）"""
        if self.logo_frames:
            self.is_showing_logo = True
            self.logo_frame_index = 0
            self.gif_label.config(image=self.logo_frames[0], text='')
            
            # 如果是動態 Logo，啟動動畫
            if len(self.logo_frames) > 1:
                self.animate_logo()
        else:
            self.show_default_text()
    
    def animate_logo(self):
        """Logo 動畫循環（如果 Logo 是動態 GIF）"""
        if not self.is_showing_logo or not self.logo_frames:
            return
        
        # 更新顯示幀
        self.gif_label.config(image=self.logo_frames[self.logo_frame_index], text='')
        
        # 移至下一幀
        self.logo_frame_index = (self.logo_frame_index + 1) % len(self.logo_frames)
        
        # 排程下一次更新
        self.logo_animation_id = self.root.after(self.logo_delay, self.animate_logo)
    
    def stop_logo_animation(self):
        """停止 Logo 動畫"""
        self.is_showing_logo = False
        if self.logo_animation_id:
            self.root.after_cancel(self.logo_animation_id)
            self.logo_animation_id = None
    
    def scan_gif_folder(self):
        """掃描 GIF 資料夾（排除 Logo.gif）"""
        # 確保資料夾存在
        if not os.path.exists(self.gif_folder):
            try:
                os.makedirs(self.gif_folder)
                self.log_status(f"已建立 GIF 資料夾：{self.gif_folder}")
            except Exception as e:
                self.log_status(f"無法建立 GIF 資料夾：{str(e)}")
                # self.gif_status_label.config(text="GIF 資料夾不存在", foreground='red')
                return
        
        # 掃描所有 GIF 檔案，但排除 Logo.gif
        all_gifs = glob.glob(os.path.join(self.gif_folder, "*.gif"))
        self.gif_files = [f for f in all_gifs if os.path.basename(f).lower() != 'logo.gif']
        
        if self.gif_files:
            count = len(self.gif_files)
            # self.log_status(f"找到 {count} 個 GIF 彩蛋檔案（不含 Logo.gif）")
            # self.gif_status_label.config(
            #     text=f"已載入 {count} 個 GIF 彩蛋，等待測試...", 
            #     foreground='green'
            # )
            pass
        else:
            # self.log_status(f"GIF 資料夾中沒有找到彩蛋 GIF（Logo.gif 除外）")
            # self.gif_status_label.config(
            #     text=f"未找到彩蛋 GIF（可放入更多 GIF 檔案）", 
            #     foreground='orange'
            # )
            pass
    
    def play_random_gif(self):
        """隨機選擇並播放 GIF（彩蛋效果）"""
        if not self.gif_files:
            # self.log_status("沒有可用的彩蛋 GIF 檔案，繼續顯示 Logo")
            return
        
        # 停止 Logo 動畫
        self.stop_logo_animation()
        
        # 停止當前 GIF 播放
        if self.gif_playing:
            self.stop_gif()
        
        # 隨機選擇一個 GIF（已排除 Logo.gif）
        gif_path = random.choice(self.gif_files)
        self.current_gif_name = os.path.basename(gif_path)
        
        try:
            self.load_gif_file(gif_path)
            if self.current_gif_frames:
                self.start_gif_animation()
                # self.log_status(f"正在播放：{self.current_gif_name}")
        except Exception as e:
            self.log_status(f"播放 GIF 失敗：{str(e)}")
    
    def load_gif_file(self, file_path):
        """載入指定的 GIF 檔案"""
        gif = Image.open(file_path)
        self.current_gif_frames = []
        
        # 取得延遲時間
        try:
            self.gif_delay = gif.info.get('duration', 100)
        except:
            self.gif_delay = 100
        
        # 提取所有幀
        try:
            while True:
                # 調整大小以適應顯示區域（保持比例）
                frame = gif.copy()
                frame.thumbnail((400, 300), Image.Resampling.LANCZOS)
                photo = ImageTk.PhotoImage(frame)
                self.current_gif_frames.append(photo)
                gif.seek(len(self.current_gif_frames))
        except EOFError:
            pass  # 已讀取所有幀
    
    def start_gif_animation(self):
        """開始 GIF 動畫"""
        if not self.current_gif_frames:
            return
        
        self.gif_playing = True
        self.gif_frame_index = 0
        # self.gif_status_label.config(
        #     text=f"播放中：{self.current_gif_name}", 
        #     foreground='blue'
        # )
        self.animate_gif()
    
    def stop_gif(self):
        """停止播放 GIF"""
        self.gif_playing = False
        if self.gif_animation_id:
            self.root.after_cancel(self.gif_animation_id)
            self.gif_animation_id = None
        
        # 更新狀態（不清空顯示，因為會由 show_logo() 處理）
        # if self.gif_files:
        #     self.gif_status_label.config(
        #         text=f"已載入 {len(self.gif_files)} 個 GIF，等待測試...", 
        #         foreground='green'
        #     )
        pass
    
    def animate_gif(self):
        """GIF 動畫循環"""
        if not self.gif_playing or not self.current_gif_frames:
            return
        
        # 更新顯示幀
        self.gif_label.config(image=self.current_gif_frames[self.gif_frame_index], text='')
        
        # 移至下一幀
        self.gif_frame_index = (self.gif_frame_index + 1) % len(self.current_gif_frames)
        
        # 排程下一次更新
        self.gif_animation_id = self.root.after(self.gif_delay, self.animate_gif)
                
    def on_closing(self):
        """視窗關閉時"""
        # 停止所有動畫
        if self.gif_playing:
            self.stop_gif()
        self.stop_logo_animation()
        
        if self.connected:
            self.disconnect()
        self.root.destroy()

def main():
    root = tk.Tk()
    
    # 設定樣式
    style = ttk.Style()
    style.theme_use('clam')
    
    app = StewartControlGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()
