import redis
import serial
import time

# 配置串口参数
ser = serial.Serial(
    port='COM7',      # 根据实际情况修改端口号，比如 Windows 使用 'COM3'，Linux 使用 '/dev/ttyUSB0'
    baudrate=9600,    # 设置波特率，需与单片机设置一致
    timeout=1         # 设置超时时间，单位为秒
)

r = redis.Redis(host='localhost', port=6379, decode_responses=True)

# 检查串口是否打开，如果没有则打开
if not ser.is_open:
    ser.open()

try:
    while True:
        # 从串口读取一行数据
        data = ser.readline().decode('utf-8').strip()
        if data:
            print(data)
        time_stamp = int(time.time())# 读取并解码为字符串
        if data == "RED":
            r.set("light_status", "RED"+"-"+str(time_stamp))
            r.set("EMERGENCY", "NO"+"-GREEN-0-RED-0")
        elif data == "GREEN":
            r.set("light_status", "GREEN" + "-" + str(time_stamp))
            r.set("EMERGENCY", "NO" + "-GREEN-0-RED-0")
        elif data == "1":
            light_status = r.get("light_status")
            _color = light_status.split("-")[0]
            _time_stamp = int(light_status.split("-")[1])
            time_count = time_stamp - _time_stamp
            emergence_str = "YES-"
            if _color == "GREEN":
                emergence_str += "GREEN-"+str(60-time_count)
            else:
                emergence_str += "GREEN-0"
            if _color == "RED":
                emergence_str += "-RED-"+str(30-time_count)
            else:
                emergence_str += "-RED-0"
            r.set("EMERGENCY", emergence_str)
            r.set("light_status", "EMERGENCY" + "-" + str(time_stamp))
        elif data.startswith("ratio"):
            ratio = float(data.split("-")[1])
            r.set("ratio", ratio)



except KeyboardInterrupt:
    print("Program interrupted")

finally:
    # 关闭串口
    ser.close()