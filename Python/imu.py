# Driver library to read IMU data over serial port

import serial
from time import *

ad = None

class IMUData:
    def __init__(self, ts, ax, ay, az, gx, gy, gz, mx, my, mz):
        self.timeStampMs = ts
        self.accel = (ax, ay, az)
        self.gyro = (gx, gy, gz)
        self.mag = (mx, my, mz)

    def __str__(self):
        return "t=" + str(self.timeStampMs) + " a=" + str(self.accel) + " g=" + str(self.gyro) + " m=" + str(self.mag)

def Initialize():
    global ad
    ad=serial.Serial('COM4',115200)
    sleep(1)

def ReadRaw():
    while (ad.inWaiting()==0):
        pass

    dataPacket=ad.readline()
    dataPacket=str(dataPacket,'utf-8')
    splitPacket=dataPacket.split(" ")
    ts=int(splitPacket[0])
    ax=int(splitPacket[1])
    ay=int(splitPacket[2])
    az=int(splitPacket[3])
    gx=int(splitPacket[4])
    gy=int(splitPacket[5])
    gz=int(splitPacket[6])
    mx=int(splitPacket[7])
    my=int(splitPacket[8])
    mz=int(splitPacket[9])
    return IMUData(ts, ax, ay, az, gx, gy, gz, mx, my, mz)

def main():
    Initialize()
    while (True):
        v = ReadRaw()
        print(v)

main()
