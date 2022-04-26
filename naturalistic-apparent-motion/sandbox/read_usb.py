import serial

ser=serial.Serial('/dev/ttyACM0', 9600)
cpt = 1000
while cpt:
    readedText = ser.readline()
    print(readedText)
    cpt = cpt-1

ser.close()