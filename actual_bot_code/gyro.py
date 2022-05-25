# NAME: gyro.py
# PURPOSE: handling low level distance gyro reading/control
# AUTHORS: Emma Bethel, Ryan McFarlane

import smbus
import time

# some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

SCALE_FACTOR = 131.0


class Gyro:

    def __init__(self):
        self.bus = smbus.SMBus(1)
        time.sleep(0.5)
        self.Device_Address = 0x68
        self._MPU_Init()
        init_gyro_z = self._read_raw_data(GYRO_ZOUT_H) 
        self.init_Gz = init_gyro_z / SCALE_FACTOR

    # the current angular velocity about the z axis (supposedly in degrees,
    #   but... it's not)
    @property
    def value(self):
        Gz = self._read_raw_data(GYRO_ZOUT_H) / SCALE_FACTOR
        return Gz - self.init_Gz

    # PURPOSE: some sort of setup stuff? i stole this from Ryan's code <3
    # PARAMETERS: N/A
    # RETURNS: N/A
    def _MPU_Init(self):
        #write to sample rate register
        self.bus.write_byte_data(self.Device_Address, SMPLRT_DIV, 7)
        
        #Write to power management register
        self.bus.write_byte_data(self.Device_Address, PWR_MGMT_1, 1)
        
        #Write to Configuration register
        self.bus.write_byte_data(self.Device_Address, CONFIG, 0)
        
        #Write to Gyro configuration register
        self.bus.write_byte_data(self.Device_Address, GYRO_CONFIG, 24)
        
        #Write to interrupt enable register
        self.bus.write_byte_data(self.Device_Address, INT_ENABLE, 1)
    
    # PURPOSE: reading current value of a given register on the gyro
    # PARAMETERS: addr - address of the MPU6050 Register to read from? I think?
    # RETURNS: the value stored at the given address
    def _read_raw_data(self, addr):
        #Accelero and Gyro value are 16-bit
        high = self.bus.read_byte_data(self.Device_Address, addr)
        low = self.bus.read_byte_data(self.Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
