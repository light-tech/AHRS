# Driver library to read IMU data over serial port

import math
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

toDeg = 180.0 / math.pi

class StateEstimator:
    """
    State estimators are object that continuously consumes IMU data and produces estimates of the object
    orientation (roll, pitch, yaw or quaternion) in 3D space. The output could be fed into the visualizer.
    """
    def __init__(self):
        self.theta = 0  # Roll
        self.phi = 0    # Pitch
        self.psi = 0    # Yaw
    
    def Update(self, imuData):
        """
        Consume the IMU data to update the state estimate
        """
        ax,ay,az = imuData.accel
        mx,my,_ = imuData.mag
        self.theta = math.atan2(ax, az)   # Dumb estimate by relying only on accelerometer
        self.phi = math.atan2(ay, az)
        self.psi = math.atan2(my, mx)     # and magnetometer

    def __str__(self):
        return f"{self.theta*toDeg:.2f} {self.phi*toDeg:.2f} {self.psi*toDeg:.2f}"

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
    est = StateEstimator()
    while (True):
        v = ReadRaw()
        est.Update(v)
        print(est)

main()
