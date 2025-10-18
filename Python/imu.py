# Driver library to read IMU data over serial port

import math
import serial
from time import *

ad = None
calib = None
toDeg = 180.0 / math.pi
toRad = math.pi / 180.0
gyroRawToDps = 4.375 * 2 / 1000.0  # For LSM6D3S configured to use gyro range 245 dps
gyroRawToRps = 4.375 * 2 / 1000.0 * toRad
storedGyroOffsets = (382.714, -287.454, -414.814)

class IMUData:
    def __init__(self, ts, ax, ay, az, gx, gy, gz, mx, my, mz):
        self.timeStampMs = ts
        self.accel = (ax, ay, az)
        self.gyro = (gx, gy, gz)
        self.mag = (mx, my, mz)

    def __str__(self):
        return "t=" + str(self.timeStampMs) + " a=" + str(self.accel) + " g=" + str(self.gyro) + " m=" + str(self.mag)

class IMUCalibrator:
    def __init__(self):
        self.gx_offset = 0
        self.gy_offset = 0
        self.gz_offset = 0
    
    def __init__(self, gyroOffsets):
        gx,gy,gz = gyroOffsets
        self.gx_offset = gx
        self.gy_offset = gy
        self.gz_offset = gz
    
    def StartGyroCalib(self):
        input("Get ready to hold still for a few seconds and press Enter to continue...")
        num_samples = 0
        samples_needed = 500
        sx = 0
        sy = 0
        sz = 0
        while (num_samples < samples_needed):
            v = ReadRaw()
            if (v != None):
                print("Received the ", num_samples, " data point")
                gx,gy,gz = v.gyro
                sx += gx
                sy += gy
                sz += gz
                num_samples += 1
        self.gx_offset = sx / samples_needed
        self.gy_offset = sy / samples_needed
        self.gz_offset = sz / samples_needed
        print("Done: ", self)

    def AdjustGyro(self, g):
        gx,gy,gz = g
        return (gx - self.gx_offset, gy - self.gy_offset, gz - self.gz_offset)
    
    def __str__(self):
        return f"{self.gx_offset} {self.gy_offset} {self.gz_offset}"

class StateEstimator:
    """
    State estimators are object that continuously consumes IMU data and produces estimates of the object
    orientation (roll, pitch, yaw or quaternion) in 3D space. The output could be fed into the visualizer.
    """
    def __init__(self, calib):
        self.timeStampMs = None
        self.theta = 0  # Pitch
        self.phi = 0    # Roll
        self.psi = 0    # Yaw
        self.calib = calib
    
    def Update(self, imuData):
        """
        Consume the IMU data to update the state estimate
        """
        ax,ay,az = imuData.accel
        gx,gy,_ = self.calib.AdjustGyro(imuData.gyro)
        mx,my,mz = imuData.mag

        if (self.timeStampMs == None):
            dt = 0
        else:
            dt = (imuData.timeStampMs - self.timeStampMs) / 1000.0

        self.timeStampMs = imuData.timeStampMs

        # Complimentary filter
        self.theta = (self.theta + dt * gy * gyroRawToRps) * 0.95 + math.atan2(ax, az) * 0.05
        self.phi = (self.phi + dt * gx * gyroRawToRps) * 0.95 + math.atan2(ay, az) * 0.05

        # Estimate yaw using magnetometer, formula from Arduino code
        Xm = mx * math.cos(self.theta) - my * math.sin(self.phi) * math.sin(self.theta) + mz * math.cos(self.phi) * math.sin(self.theta)
        Ym = my * math.cos(self.phi) + mz * math.sin(self.phi)
        self.psi = math.atan2(Ym, Xm)
    
    def Roll(self):
        return self.phi
    
    def Pitch(self):
        return self.theta

    def Yaw(self):
        return self.psi
    
    def EulerAngles(self):
        return (self.phi, self.theta, self.psi)

    def Quaternion(self):
        raise NotImplementedError("Quaternion is not yet implemented.")

    def __str__(self):
        return f"{self.theta*toDeg:.2f} {self.phi*toDeg:.2f} {self.psi*toDeg:.2f}"

class MadgwickIMUStateEstimator:
    def __init__(self, calib):
        self.timeStampMs = None
        self.SEq = (1.0, 0.0, 0.0, 0.0)
        # Make these configurable in constructor
        self.gyroMeasError = 5.0 * toRad
        self.beta = math.sqrt(3.0 / 4.0) * self.gyroMeasError
        self.calib = calib
    
    def Update(self, imuData):
        """
        Consume the IMU data to update the state estimate
        """
        a_x,a_y,a_z = imuData.accel
        w_x,w_y,w_z = self.calib.AdjustGyro(imuData.gyro)
        w_x = w_x * gyroRawToRps
        w_y = w_y * gyroRawToRps
        w_z = w_z * gyroRawToRps
        SEq_1,SEq_2,SEq_3,SEq_4 = self.SEq

        if (self.timeStampMs == None):
            deltat = 0
        else:
            deltat = (imuData.timeStampMs - self.timeStampMs) / 1000.0
        self.timeStampMs = imuData.timeStampMs

        halfSEq_1 = 0.5 * SEq_1
        halfSEq_2 = 0.5 * SEq_2
        halfSEq_3 = 0.5 * SEq_3
        halfSEq_4 = 0.5 * SEq_4
        twoSEq_1 = 2.0 * SEq_1
        twoSEq_2 = 2.0 * SEq_2
        twoSEq_3 = 2.0 * SEq_3

        norm = math.sqrt(a_x * a_x + a_y * a_y + a_z * a_z)
        a_x /= norm
        a_y /= norm
        a_z /= norm

        f_1 = twoSEq_2 * SEq_4- twoSEq_1 * SEq_3- a_x
        f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4- a_y
        f_3 = 1.0- twoSEq_2 * SEq_2- twoSEq_3 * SEq_3- a_z
        J_11or24 = twoSEq_3
        J_12or23 = 2.0 * SEq_4
        J_13or22 = twoSEq_1
        J_14or21 = twoSEq_2
        J_32 = 2.0 * J_14or21
        J_33 = 2.0 * J_11or24

        SEqHatDot_1 = J_14or21 * f_2- J_11or24 * f_1
        SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2- J_32 * f_3
        SEqHatDot_3 = J_12or23 * f_2- J_33 * f_3- J_13or22 * f_1
        SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2

        norm = math.sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4)
        SEqHatDot_1 /= norm
        SEqHatDot_2 /= norm
        SEqHatDot_3 /= norm
        SEqHatDot_4 /= norm

        SEqDot_omega_1 =-halfSEq_2 * w_x- halfSEq_3 * w_y- halfSEq_4 * w_z
        SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z- halfSEq_4 * w_y
        SEqDot_omega_3 = halfSEq_1 * w_y- halfSEq_2 * w_z + halfSEq_4 * w_x
        SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y- halfSEq_3 * w_x

        SEq_1 += (SEqDot_omega_1- (self.beta * SEqHatDot_1)) * deltat
        SEq_2 += (SEqDot_omega_2- (self.beta * SEqHatDot_2)) * deltat
        SEq_3 += (SEqDot_omega_3- (self.beta * SEqHatDot_3)) * deltat
        SEq_4 += (SEqDot_omega_4- (self.beta * SEqHatDot_4)) * deltat

        norm = math.sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4)
        SEq_1 /= norm
        SEq_2 /= norm
        SEq_3 /= norm
        SEq_4 /= norm
        self.SEq = (SEq_1, SEq_2, SEq_3, SEq_4)
    
    def EulerAngles(self):
        q0,q1,q2,q3 = self.SEq
        roll = math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch = math.asin(2*(q0*q2-q3*q1))
        yaw = math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
        return (roll, pitch, yaw)
    
    def Quaternion(self):
        return self.SEq

    def __str__(self):
        return str(self.SEq)

def Initialize():
    global ad
    global calib
    ad=serial.Serial('COM4',115200)
    sleep(1)
    if (storedGyroOffsets == None):
        calib = IMUCalibrator()
        calib.StartGyroCalib()
    else:
        calib = IMUCalibrator(storedGyroOffsets)

def GetCalibrator():
    return calib

def ReadRaw():
    while (ad.inWaiting()==0):
        pass

    dataPacket=ad.readline()
    dataPacket=str(dataPacket,'utf-8')
    splitPacket=dataPacket.split(" ")
    if len(splitPacket) == 10:
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
    else:
        return None

def _main():
    Initialize()
    est = StateEstimator(calib)
    while (True):
        v = ReadRaw()
        if v != None:
            est.Update(v)
            print(est)

if __name__ == "__main__":
    _main()
