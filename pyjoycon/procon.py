import hid
import time
import threading
from typing import Optional

class ProController:
    # Pro Controller特有的常量
    _INPUT_REPORT_SIZE = 49
    _INPUT_REPORT_PERIOD = 0.00833  # 1/120秒,对应120Hz IMU采样率
    _RUMBLE_DATA = b'\x00\x01\x40\x40\x00\x01\x40\x40'
    
    # Pro Controller的SPI内存地址常量
    _SPI_COLOR_DATA_ADDR = 0x6050
    _SPI_IMU_USER_CAL_ADDR = 0x8028
    _SPI_IMU_FACTORY_CAL_ADDR = 0x6020
    _SPI_STICK_CAL_ADDR = 0x603D
    
    def __init__(self, vendor_id: int, product_id: int, serial: str = None, simple_mode=False):
        """初始化Pro Controller
        
        Args:
            vendor_id: 供应商ID
            product_id: 产品ID 
            serial: 控制器序列号
            simple_mode: 简单模式标志
        """
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.serial = serial
        self.simple_mode = simple_mode
        
        # 内部状态
        self._input_hooks = []
        self._input_report = bytes(self._INPUT_REPORT_SIZE)
        self._packet_number = 0
        
        # 初始化校准数据
        self.set_accel_calibration((0, 0, 0), (1, 1, 1))
        self.set_gyro_calibration((0, 0, 0), (1, 1, 1))
        
        # 连接设备
        self._procon_device = self._open_device()
        self._read_controller_data()
        self._setup_sensors()
        
        # 启动数据更新线程
        self._update_thread = threading.Thread(target=self._update_input_report)
        self._update_thread.daemon = True
        self._update_thread.start()

    def _open_device(self):
        """打开Pro Controller设备连接"""
        try:
            if hasattr(hid, "device"):  # hidapi
                device = hid.device()
                device.open(self.vendor_id, self.product_id, self.serial)
            elif hasattr(hid, "Device"):  # hid
                device = hid.Device(self.vendor_id, self.product_id, self.serial)
            else:
                raise Exception("Unrecognized HID implementation!")
            return device
        except IOError as e:
            raise IOError('Failed to connect to Pro Controller') from e

    def _close(self):
        """关闭设备连接"""
        if hasattr(self, "_procon_device"):
            self._procon_device.close()
            del self._procon_device

    def _read_input_report(self) -> bytes:
        """读取输入报告"""
        return bytes(self._procon_device.read(self._INPUT_REPORT_SIZE))

    def _write_output_report(self, command, subcommand, argument):
        """写入输出报告
        
        Args:
            command: 命令字节
            subcommand: 子命令字节
            argument: 参数字节
        """
        self._procon_device.write(b''.join([
            command,
            self._packet_number.to_bytes(1, byteorder='little'),
            self._RUMBLE_DATA,
            subcommand,
            argument,
        ]))
        self._packet_number = (self._packet_number + 1) & 0xF

    def _send_subcmd_get_response(self, subcommand, argument) -> (bool, bytes):
        """发送子命令并获取响应
        
        Args:
            subcommand: 子命令字节
            argument: 参数字节
            
        Returns:
            (ack, data): 确认标志和响应数据
        """
        self._write_output_report(b'\x01', subcommand, argument)
        
        report = self._read_input_report()
        while report[0] != 0x21:
            report = self._read_input_report()
            
        return report[13] & 0x80, report[13:]

    def _spi_flash_read(self, address, size) -> bytes:
        """读取SPI闪存数据
        
        Args:
            address: 内存地址
            size: 读取大小
            
        Returns:
            读取的数据
        """
        assert size <= 0x1d
        argument = address.to_bytes(4, "little") + size.to_bytes(1, "little")
        ack, report = self._send_subcmd_get_response(b'\x10', argument)
        
        if not ack:
            raise IOError(f"SPI read failed at address {address:#06x}")
            
        if report[:2] != b'\x90\x10':
            raise IOError("Unexpected response received")
            
        return report[7:size+7]

    def _update_input_report(self):
        """更新输入报告的守护线程"""
        while True:
            report = self._read_input_report()
            while report[0] != 0x30:
                report = self._read_input_report()
                
            self._input_report = report
            
            for callback in self._input_hooks:
                callback(self)

    def _read_controller_data(self):
        """读取控制器数据(颜色、校准等)"""
        # 读取颜色数据
        color_data = self._spi_flash_read(self._SPI_COLOR_DATA_ADDR, 6)
        self.color_body = tuple(color_data[:3])
        self.color_btn = tuple(color_data[3:])
        
        # 读取IMU校准数据
        if self._spi_flash_read(0x8026, 2) == b"\xB2\xA1":
            imu_cal = self._spi_flash_read(self._SPI_IMU_USER_CAL_ADDR, 24)
        else:
            imu_cal = self._spi_flash_read(self._SPI_IMU_FACTORY_CAL_ADDR, 24)
            
        # 设置加速度计和陀螺仪校准
        self._set_imu_calibration(imu_cal)
        
        # 读取摇杆校准数据
        stick_cal = self._spi_flash_read(self._SPI_STICK_CAL_ADDR, 9)
        self._set_stick_calibration(stick_cal)

    def _setup_sensors(self):
        """设置传感器"""
        # 启用6轴传感器
        self._write_output_report(b'\x01', b'\x40', b'\x01')
        time.sleep(0.02)
        
        # 设置输入报告格式为0x30(标准完整模式)
        self._write_output_report(b'\x01', b'\x03', b'\x30')

    def _set_imu_calibration(self, cal_data: bytes):
        """设置IMU校准数据"""
        def to_int16le(hbyte, lbyte):
            uint16le = (lbyte << 8) | hbyte
            return uint16le if uint16le < 32768 else uint16le - 65536
            
        # 设置加速度计校准
        self.set_accel_calibration(
            (to_int16le(cal_data[0], cal_data[1]),
             to_int16le(cal_data[2], cal_data[3]),
             to_int16le(cal_data[4], cal_data[5])),
            (to_int16le(cal_data[6], cal_data[7]),
             to_int16le(cal_data[8], cal_data[9]),
             to_int16le(cal_data[10], cal_data[11]))
        )
        
        # 设置陀螺仪校准
        self.set_gyro_calibration(
            (to_int16le(cal_data[12], cal_data[13]),
             to_int16le(cal_data[14], cal_data[15]),
             to_int16le(cal_data[16], cal_data[17])),
            (to_int16le(cal_data[18], cal_data[19]),
             to_int16le(cal_data[20], cal_data[21]),
             to_int16le(cal_data[22], cal_data[23]))
        )

    def _set_stick_calibration(self, cal_data: bytes):
        """设置摇杆校准数据"""
        # TODO: 实现摇杆校准逻辑
        pass

    def set_gyro_calibration(self, offset_xyz=None, coeff_xyz=None):
        """设置陀螺仪校准参数"""
        if offset_xyz:
            self._gyro_offset_x, self._gyro_offset_y, self._gyro_offset_z = offset_xyz
        if coeff_xyz:
            cx, cy, cz = coeff_xyz
            self._gyro_coeff_x = 0x343b / cx if cx != 0x343b else 1
            self._gyro_coeff_y = 0x343b / cy if cy != 0x343b else 1
            self._gyro_coeff_z = 0x343b / cz if cz != 0x343b else 1

    def set_accel_calibration(self, offset_xyz=None, coeff_xyz=None):
        """设置加速度计校准参数"""
        if offset_xyz:
            self._accel_offset_x, self._accel_offset_y, self._accel_offset_z = offset_xyz
        if coeff_xyz:
            cx, cy, cz = coeff_xyz
            self._accel_coeff_x = 0x4000 / cx if cx != 0x4000 else 1
            self._accel_coeff_y = 0x4000 / cy if cy != 0x4000 else 1
            self._accel_coeff_z = 0x4000 / cz if cz != 0x4000 else 1

    def get_status(self) -> dict:
        """获取控制器状态
        
        Returns:
            包含按键、摇杆、传感器等状态的字典
        """
        return {
            "battery": {
                "charging": self._get_battery_charging(),
                "level": self._get_battery_level()
            },
            "buttons": {
                "a": self._get_button_state(3, 3),
                "b": self._get_button_state(3, 2),
                "x": self._get_button_state(3, 1),
                "y": self._get_button_state(3, 0),
                "plus": self._get_button_state(4, 1),
                "minus": self._get_button_state(4, 0),
                "home": self._get_button_state(4, 4),
                "capture": self._get_button_state(4, 5),
                "l": self._get_button_state(5, 6),
                "zl": self._get_button_state(5, 7),
                "r": self._get_button_state(3, 6),
                "zr": self._get_button_state(3, 7),
                "l-stick": self._get_button_state(4, 3),
                "r-stick": self._get_button_state(4, 2),
                "up": self._get_button_state(5, 1),
                "down": self._get_button_state(5, 0),
                "left": self._get_button_state(5, 3),
                "right": self._get_button_state(5, 2),
            },
            "analog-sticks": {
                "left": self._get_left_stick_state(),
                "right": self._get_right_stick_state()
            },
            "imu": {
                "accel": self._get_accel_state(),
                "gyro": self._get_gyro_state()
            }
        }

    def _get_button_state(self, byte_idx: int, bit_idx: int) -> bool:
        """获取按键状态"""
        return bool(self._input_report[byte_idx] & (1 << bit_idx))

    def _get_left_stick_state(self) -> dict:
        """获取左摇杆状态"""
        return {
            "horizontal": self._get_stick_value(6, 7, 8),
            "vertical": self._get_stick_value(7, 8, 4)
        }

    def _get_right_stick_state(self) -> dict:
        """获取右摇杆状态"""
        return {
            "horizontal": self._get_stick_value(9, 10, 8),
            "vertical": self._get_stick_value(10, 11, 4)
        }

    def _get_stick_value(self, lbyte: int, hbyte: int, shift: int) -> int:
        """获取摇杆数值"""
        return self._input_report[lbyte] | ((self._input_report[hbyte] & 0xF) << shift)

    def _get_accel_state(self) -> dict:
        """获取加速度计状态"""
        return {
            "x": self._get_accel_value(0),
            "y": self._get_accel_value(1),
            "z": self._get_accel_value(2)
        }

    def _get_gyro_state(self) -> dict:
        """获取陀螺仪状态"""
        return {
            "x": self._get_gyro_value(0),
            "y": self._get_gyro_value(1),
            "z": self._get_gyro_value(2)
        }

    def _get_accel_value(self, axis: int) -> float:
        """获取加速度计数值"""
        offset = 13 + axis * 2
        raw = int.from_bytes(self._input_report[offset:offset+2], 'little', signed=True)
        return (raw - getattr(self, f'_accel_offset_{["x","y","z"][axis]}')) * \
               getattr(self, f'_accel_coeff_{["x","y","z"][axis]}')

    def _get_gyro_value(self, axis: int) -> float:
        """获取陀螺仪数值"""
        offset = 19 + axis * 2
        raw = int.from_bytes(self._input_report[offset:offset+2], 'little', signed=True)
        return (raw - getattr(self, f'_gyro_offset_{["x","y","z"][axis]}')) * \
               getattr(self, f'_gyro_coeff_{["x","y","z"][axis]}')

    def _get_battery_charging(self) -> bool:
        """获取充电状态"""
        return bool(self._input_report[2] & 0x10)

    def _get_battery_level(self) -> int:
        """获取电池电量"""
        return (self._input_report[2] & 0xE0) >> 5

    def set_player_lamp(self, pattern: int):
        """设置玩家指示灯
        
        Args:
            pattern: LED模式(0-15)
        """
        self._write_output_report(b'\x01', b'\x30', bytes([pattern & 0xF]))

    def disconnect_device(self):
        """断开设备连接"""
        self._write_output_report(b'\x01', b'\x06', b'\x00')
        self._close()

    def __del__(self):
        """析构函数"""
        self._close()
