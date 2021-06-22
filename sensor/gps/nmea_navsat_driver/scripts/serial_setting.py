import serial

# references
# https://www.wless.ru/files/GPS/Locosys/LS2303x-G_datasheet_v0.1.pdf

serial_port = "/dev/ttyUSB0"

# baud rate to 57600
serial_baud = 9600
GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
GPS.write("$PMTK251,57600*2C\r\n")
print(GPS.readline())
GPS.close()

# frame rate to 10 Hz
serial_baud = 57600
GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)
GPS.write("$PMTK220,100*2F\r\n")
print(GPS.readline())

while True:
    data = GPS.readline().strip()
    print(data)
    print("------------")