import serial

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)

while True:
	cmd = input("Input cmd: ")
	ser.write(cmd.encode('utf-8'))
