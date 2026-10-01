# 舊版二進位控制路徑

此目錄的兩個 PC 介面都搭配 `esp32c6_pca9685_receiver/esp32c6_pca9685_receiver.ino`，使用 115200 baud、12-byte 大端序角度封包。逆運動學在電腦執行。

從此目錄啟動 `6dof_visualizer.py`（3D 視覺化）或 `stewart_control_gui.py`（本次整理的相容面板），一次只使用一個串列連線。先安裝根目錄的 requirements.txt。

- `stewart_kinematics.py`：從原視覺化抽出的共用計算，三個數學方法內容保持一致。
- `stewart_config.py`：舊版幾何，高度 91.92 mm。
- `pca9685_transport.py`：相容面板使用；拒絕不可達或非有限姿態，只重送最新目標，最多約 50 Hz。
- 接收器：PWM 50 Hz，SDA22／SCL23，通道 0～5，CH0、2、4 反轉。

與主版本的 460800 baud `$P` 協定、330 Hz PWM、96.04 mm 平台高度不同。相容面板尚未實機驗證。舊接收器無封包標頭或校驗碼；接收器的 DEBUG 資料不是實際姿態量測。

## 軟體測試

在本目錄執行：

```powershell
python -m unittest discover -s tests -v
```

測試使用假串列埠，不會驅動硬體。
