from base_ctrl import BaseController
import time


# 用于检测树莓派的函数
def is_raspberry_pi5():
    with open("/proc/cpuinfo", "r") as file:
        for line in file:
            if "Model" in line:
                if "Raspberry Pi 5" in line:
                    return True
                else:
                    return False


# 根据树莓派的型号来确定 GPIO 串口设备名称
if is_raspberry_pi5():
    base = BaseController("/dev/ttyAMA0", 115200)
else:
    base = BaseController("/dev/serial0", 115200)

# 轮子以0.2m/s的速度转动2秒钟后停止
base.send_command({"T": 1, "L": 0.2, "R": 0.2})
time.sleep(2)
base.send_command({"T": 1, "L": 0, "R": 0})
print("finish")
