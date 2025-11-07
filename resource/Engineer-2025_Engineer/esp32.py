import sys
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QFrame, QSlider, QTextEdit, QGridLayout, QLineEdit, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QBrush, QFont

#define ARM_YAW_PHYSICAL_RANGE_MIN -97.5f
#define ARM_YAW_PHYSICAL_RANGE_MAX 97.5f
#define ARM_PITCH1_PHYSICAL_RANGE_MIN 4.0f
#define ARM_PITCH1_PHYSICAL_RANGE_MAX 118.0f
#define ARM_PITCH2_PHYSICAL_RANGE_MIN 11.0f
#define ARM_PITCH2_PHYSICAL_RANGE_MAX 118.0f
#define ARM_ROLL_PHYSICAL_RANGE_MIN -169.0f
#define ARM_ROLL_PHYSICAL_RANGE_MAX 180.0f
#define ARM_END_PITCH_PHYSICAL_RANGE_MIN -145.0f
#define ARM_END_PITCH_PHYSICAL_RANGE_MAX 60.0f


class LEDIndicator(QFrame):
    def __init__(self, color="gray"):
        super().__init__()
        self.color = color
        self.setFixedSize(16, 16)

    def set_color(self, color):
        self.color = color
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(QBrush(QColor(self.color)))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(0, 0, 16, 16)


class BluetoothControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 蓝牙控制面板")
        self.setMinimumSize(850, 750)
        self.setFont(QFont("Microsoft YaHei", 10))  # 使用微软雅黑字体

        self.serial = None
        self.leds = {}
        self.sliders = []
        self.slider_labels = []
        self.slider_names = ["Yaw", "Pitch1", "Pitch2", "Roll", "End_Pitch", "End_Roll"]
        self.slider_mode_open = False
        
        # 每个滑块的配置：最小值，最大值，初始值
        self.slider_configs = {
            "Yaw": {"min": -97.5, "max": 97.5, "init": 0},
            "Pitch1": {"min": 4.0, "max": 118.0, "init": 15.0},
            "Pitch2": {"min": 11.0, "max": 118.0, "init": 18.0},
            "Roll": {"min": -169, "max": 180, "init": 0},
            "End_Pitch": {"min": -145.0, "max": 60, "init": 0},
            "End_Roll": {"min": -360, "max": 360, "init": 0}
        }

        main_layout = QVBoxLayout()

        # 串口选择与波特率
        serial_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "115200", "230400", "460800", "921600"])
        self.refresh_ports()
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.connect_serial)
        serial_layout.addWidget(QLabel("串口："))
        serial_layout.addWidget(self.port_combo)
        serial_layout.addWidget(QLabel("波特率："))
        serial_layout.addWidget(self.baud_combo)
        serial_layout.addWidget(self.connect_btn)
        main_layout.addLayout(serial_layout)

        # LED 区域 (5行3列)
        led_names = [
            "Chassis_LF", "Chassis_RF", "Chassis_LB", "Chassis_RB",
            "Subgantry_Lift_L", "Subgantry_Lift_R", "Subgantry_Stretch_L", "Subgantry_Stretch_R",
            "Gimbal", "Yaw", "Pitch1", "Pitch2", "Roll", "End_L", "End_R"
        ]
        led_grid = QGridLayout()
        for i, name in enumerate(led_names):
            row = i // 3  # 5行
            col = i % 3   # 3列
            led = LEDIndicator()
            label = QLabel(name)
            hbox = QHBoxLayout()
            hbox.addWidget(led)
            hbox.addWidget(label)
            container = QWidget()
            container.setLayout(hbox)
            self.leds[name.lower()] = led
            led_grid.addWidget(container, row, col)
        main_layout.addLayout(led_grid)

        # 滑块区 - 使用网格布局使滑块等长
        slider_grid = QGridLayout()
        slider_grid.setColumnStretch(0, 1)  # 设置两列等宽
        slider_grid.setColumnStretch(1, 1)
        
        for i, name in enumerate(self.slider_names):
            row = i // 2  # 3行
            col = i % 2   # 2列
            
            # 创建滑块组
            slider_group = QVBoxLayout()
            
            # 名称标签
            name_label = QLabel(name)
            name_label.setAlignment(Qt.AlignCenter)
            slider_group.addWidget(name_label)
            
            # 滑块和值标签
            slider_hbox = QHBoxLayout()
            
            slider = QSlider(Qt.Horizontal)
            config = self.slider_configs[name]
            slider.setRange(0, 1000)
            slider.setValue(int((config["init"] - config["min"]) / (config["max"] - config["min"]) * 1000))
            slider.setStyleSheet("""
                QSlider::groove:horizontal {
                    height: 10px;
                    background: #ddd;
                    border-radius: 5px;
                }
                QSlider::handle:horizontal {
                    background: #4285F4;
                    width: 20px;
                    height: 20px;
                    margin: -7px 0;
                    border-radius: 10px;
                }
            """)
            slider.setMinimumWidth(300)  # 设置最小宽度确保等长
            
            value_label = QLabel(f"{config['init']:.2f}")
            value_label.setAlignment(Qt.AlignCenter)
            value_label.setMinimumWidth(60)  # 固定值标签宽度
            
            slider_hbox.addWidget(slider)
            slider_hbox.addWidget(value_label)
            
            slider_group.addLayout(slider_hbox)
            
            self.slider_labels.append(value_label)
            slider.valueChanged.connect(lambda val, i=i: self.update_slider(i, val))
            self.sliders.append(slider)
            
            # 添加到网格
            slider_grid.addLayout(slider_group, row, col)

        main_layout.addLayout(slider_grid)

        # 滑块控制模式按钮
        self.slider_mode_btn = QPushButton("开启蓝牙控制模式")
        self.slider_mode_btn.setCheckable(True)
        self.slider_mode_btn.clicked.connect(self.toggle_slider_mode)
        main_layout.addWidget(self.slider_mode_btn, alignment=Qt.AlignCenter)

        # 添加打印选项按钮
        self.print_checkbox = QCheckBox("启用终端打印")
        self.print_checkbox.setChecked(False)  # 默认不打印
        main_layout.addWidget(self.print_checkbox, alignment=Qt.AlignCenter)

        # 串口手动发送
        send_layout = QHBoxLayout()
        self.send_input = QLineEdit()
        self.send_input.returnPressed.connect(self.send_data)
        self.send_button = QPushButton("发送")
        self.send_button.clicked.connect(self.send_data)
        send_layout.addWidget(self.send_input)
        send_layout.addWidget(self.send_button)
        main_layout.addLayout(send_layout)

        # 日志显示
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        main_layout.addWidget(QLabel("串口日志："))
        main_layout.addWidget(self.log_area)

        self.setLayout(main_layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial_data)
        self.last_update_time = 0  # 用于限制更新频率

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        self.port_combo.clear()
        for port in ports:
            self.port_combo.addItem(port.device)

    def connect_serial(self):
        port = self.port_combo.currentText()
        baudrate = int(self.baud_combo.currentText())
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
                
            self.serial = serial.Serial(port, baudrate, timeout=0.01)  # 减小超时时间
            self.serial.flushInput()  # 清空输入缓冲区
            self.connect_btn.setText("已连接")
            self.connect_btn.setEnabled(False)
            self.timer.start(20)  # 增加读取频率到50Hz
            self.log_area.append(f"已连接 {port} @ {baudrate} baud")
        except Exception as e:
            self.log_area.append(f"连接失败: {e}")
            self.reset_serial_connection()

    def reset_serial_connection(self):
        """重置串口连接状态"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except:
            pass
            
        self.serial = None
        self.connect_btn.setText("连接")
        self.connect_btn.setEnabled(True)
        self.timer.stop()
        
        # 将所有LED重置为灰色
        for led in self.leds.values():
            led.set_color("gray")
            
        self.log_area.append("串口连接已重置")

    def toggle_slider_mode(self):
        self.slider_mode_open = not self.slider_mode_open
        if self.slider_mode_open:
            self.slider_mode_btn.setText("关闭蓝牙控制模式")
        else:
            self.slider_mode_btn.setText("开启蓝牙控制模式")

    def update_slider(self, index, raw_value):
        name = self.slider_names[index]
        config = self.slider_configs[name]
        min_val = config["min"]
        max_val = config["max"]
        value = min_val + (raw_value / 1000.0) * (max_val - min_val)
        self.slider_labels[index].setText(f"{value:.2f}")

    def slider_print(self):
        if not self.serial or not self.serial.is_open:
            return
            
        if self.slider_mode_open:
            values = []
            for i in range(len(self.sliders)):
                slider_name = self.slider_names[i]
                cfg = self.slider_configs[slider_name]
                val = cfg["min"] + (self.sliders[i].value() / 1000.0) * (cfg["max"] - cfg["min"])
                val_int = int(val * 1000)
                values.append(f"{slider_name} = {val_int}")
            msg = f"slider open. {', '.join(values)}\n"
            try:
                self.serial.write(msg.encode())
            except Exception as e:
                self.log_area.append(f"发送滑块数据失败: {e}")
                self.reset_serial_connection()
        elif not self.slider_mode_open:
            msg = "slider close.\n"
            try:
                self.serial.write(msg.encode())
            except Exception as e:
                self.log_area.append(f"发送关闭滑块模式失败: {e}")
                self.reset_serial_connection()

        # ✅ 仅当勾选打印复选框时才输出终端信息
        if self.print_checkbox.isChecked():
            for i, slider_name in enumerate(self.slider_names):
                cfg = self.slider_configs[slider_name]
                val = cfg["min"] + (self.sliders[i].value() / 1000.0) * (cfg["max"] - cfg["min"])
                print(f"core.parm_->armCmd.set_angle_{slider_name} = {val:.3f}f;")
            print("-" * 50)


    def send_data(self):
        msg = self.send_input.text().strip()
        if msg:
            if not self.serial or not self.serial.is_open:
                self.log_area.append("发送失败: 串口未连接")
                return
                
            try:
                self.serial.write((msg + '\n').encode())
                self.log_area.append(f">> {msg}")
                self.send_input.clear()
            except Exception as e:
                self.log_area.append(f"发送失败: {e}")
                self.reset_serial_connection()

    def read_serial_data(self):
        try:
            # 先发送滑块数据
            self.slider_print()
            
            if not self.serial or not self.serial.is_open:
                return
                
            # 读取所有可用数据
            data = self.serial.read(self.serial.in_waiting or 1)
            if not data:
                return
                
            # 解码并分割为行
            decoded_data = data.decode(errors="ignore")
            lines = decoded_data.split('\n')
            
            # 处理每一行
            for line in lines:
                line = line.strip()
                if not line:
                    continue
                    
                # 处理以print:开头的日志 去掉print:然后记录到日志
                if line.startswith("print:"):
                    log_msg = ">> " + line[len("print:"):].strip()
                    self.log_area.append(log_msg)
                    continue
                
                # 处理LED状态更新（使用:分隔符）
                if ':' in line and 'line' in line:
                    parts = line.split(',')
                    for part in parts:
                        part = part.strip()
                        if ':' in part:
                            name_status = part.split(':', 1)
                            if len(name_status) == 2:
                                name = name_status[0].strip().lower()
                                status = name_status[1].strip().lower()
                                
                                if name in self.leds:
                                    color = "green" if status == "online" else "red" if status == "offline" else "gray"
                                    self.leds[name].set_color(color)
                                    continue  # 跳过日志记录
                    
                    # 处理完所有LED状态后，不再记录这行日志
                    continue
                
                # 处理滑块数据更新（使用=分隔符）
                if '=' in line and any(name.lower() in line.lower() for name in self.slider_names):
                    parts = line.split(',')
                    for part in parts:
                        part = part.strip()
                        if '=' in part:
                            name_val = part.split('=', 1)
                            if len(name_val) == 2:
                                name = name_val[0].strip()
                                val_str = name_val[1].strip()
                                
                                # 检查是否是滑块名称
                                if name in self.slider_names:
                                    try:
                                        float_val = float(val_str) / 1000
                                        index = self.slider_names.index(name)
                                        config = self.slider_configs[name]
                                        min_val = config["min"]
                                        max_val = config["max"]
                                        
                                        # 计算滑块位置
                                        slider_val = int(((float_val - min_val) / (max_val - min_val)) * 1000)
                                        slider_val = max(0, min(1000, slider_val))  # 确保在0-1000范围内
                                        
                                        if 0 <= index < len(self.sliders):
                                            # 更新滑块值（阻止信号循环）
                                            self.sliders[index].blockSignals(True)
                                            self.sliders[index].setValue(slider_val)
                                            self.slider_labels[index].setText(f"{float_val:.2f}")  # 显示整数值
                                            self.sliders[index].blockSignals(False)
                                    except ValueError:
                                        pass
                                    
                                    # 跳过日志记录
                                    continue
                
                    # 处理完所有滑块数据后，不再记录这行日志
                    continue
                
                # 如果未匹配任何模式，记录原始数据
                self.log_area.append(f"<< {line}")
                
        except serial.SerialException as e:
            self.log_area.append(f"串口错误: {e}")
            self.reset_serial_connection()
        except Exception as e:
            self.log_area.append(f"读取错误: {e}")
            self.reset_serial_connection()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = BluetoothControlPanel()
    win.show()
    sys.exit(app.exec_())
