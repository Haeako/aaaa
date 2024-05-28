import serial
import serial.tools.list_ports
import os

os.system('cls')
print('Search...')
ports = serial.tools.list_ports.comports(include_links=False)
for port in ports :
    print('Find port '+ port.device)
# Thiết lập kết nối với cổng Serial
DataSerial = serial.Serial('COM12', 112500)  # Thay đổi tên cổng Serial và tốc độ baud tương ứng

def send_position(data):
    # print(data)
    data = data + '\r'
    if data :
        DataSerial.write(data.encode('UTF-8'))
        #print(f"sending string {data}")


def get_position():
    while DataSerial.in_waiting == 0:
        pass

    data = DataSerial.readline()
    data = str(data, 'utf-8')
    return data
