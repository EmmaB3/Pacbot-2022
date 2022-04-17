import smbus


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
        self.Device_Address = 0x68
        init_gyro_z = self._read_raw_data(GYRO_ZOUT_H) 
        self.init_Gz = init_gyro_z / SCALE_FACTOR

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
    
    @property
    def value(self):
        Gz = self._read_raw_data(GYRO_ZOUT_H) / SCALE_FACTOR
        return Gz - self.init_Gz