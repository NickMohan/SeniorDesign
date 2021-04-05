import serial
from datetime import datetime

port = "/dev/ttyACM0"
baud = 9600
filePath = "output.txt"

outFile = open(filePath, "w+")
s = serial.Serial(port,baud)

while True:
    l = s.readLine()
#    l = l.decode("utf-8")
    putFile.write(datetime.now(),"\t %s, \n" % (l))
