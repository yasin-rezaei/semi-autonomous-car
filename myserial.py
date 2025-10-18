
import serial
from time import sleep

def send(data):
      data = data+"\n"
      ser.write(data.encode('utf-8'))
    
if __name__ == 'myserial':
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.reset_input_buffer()
