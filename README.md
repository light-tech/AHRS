# Attitude Heading Reference System

[Attitude Heading Reference System (AHRS)](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_20-181.pdf) is an integrated system that provides a vehicle (e.g. an aircraft) position and orientation in 3D space for navigation purpose. It is typically implemented by fusing data from various sensors such as accelerometers, gyroscopes, magnetometer (commonly referred to as *Inertia Measurement Units* or *IMU*). There are other applications such as self stabilizing platforms, self balancing robots, ...

In this project I am implementing the wellknown algorithms in this topic and applications to visualize the orientation.

## Hardware

These days, some IMUs such as BNO055 implement many algorithms in the hardware.

We want to use more basic IMUs which only supply raw accelerations, angular velocities, etc. and then process them in software running on the computer so if desire, one should be able to port them to run on the microcontroller.

The IMU combinations should not matter (ADXL335, MPU6050, MPU9250, BNO160, LSM6D3S, ...) as long as they can generate 9 numbers

 * accelerations in 3 directions $$X, Y, Z$$ in unit of $$g$$ (the gravitational acceleration, roughly $$1g = 9.8 m/s^2$$),
 * rotational velocities in 3 axes in unit of radians per second,
 * magnetic field strength in 3 axes in unit of Gauss.

## Calibration

## Quaternion

## Visualization

## References and Resources

 * Short introduction video https://www.youtube.com/watch?v=eqZgxR6eRjo

 * This course on 9-axis IMU https://www.youtube.com/watch?v=2AO_Gmh5K3Q&list=PLGs0VKk2DiYwEo-k0mjIkWXlkrJWAU4L9

 * MATLAB course on sensor fusion https://www.youtube.com/watch?v=6qV3YjFppuc&list=PLn8PRpmsu08ryYoBpEKzoMOveSTyS-h4a and Kalman filter https://www.youtube.com/watch?v=mwn8xhgNpFY&list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr