'''
        Read Gyro and Accelerometer by Interfacing Raspberry Pi with MPU6050 using Python
	http://www.electronicwings.com
'''
import smbus			#import SMBus module of I2C
from time import sleep          #import
import board
import adafruit_tca9548a
import busio
import time
import adafruit_vl6180x

from gpiozero import PhaseEnableMotor

#some MPU6050 Registers and their Address
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


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


MIN_DIST = 50
READING_AMOUNT = 2
TURN_LEFT_YAW = 5.69
TURN_RIGHT_YAW = 5.69
TURN_AROUND_YAW = 11.38

# indices in sensors list
LEFT = 0
FRONT = 1
RIGHT = 2

# "directives"
FORWARD = 0
CORRECT_RIGHT = 1
CORRECT_LEFT = 2
STOP = 3

# turn slightly left to avoid collision
def go_left():
    print('go left')
    leftMotor.forward(0.3)
    rightMotor.forward(0.7)

# turn slightly right to avoid collision
def go_right():
    print('go right')
    leftMotor.forward(0.7)
    rightMotor.forward(0.3)

# stop to avoid collision
def stop_moving():
    print('stopping')
    leftMotor.stop()
    rightMotor.stop()

def forward():
    print('forward')
    leftMotor.forward(0.5)
    rightMotor.forward(0.5)



def pivot_left():
    print('turning left')
    leftMotor.backward(0.5)
    rightMotor.forward(0.5)
    time.sleep(0.67)
    stop_moving()


def pivot_right():
    print('turning right')
    leftMotor.forward(0.5)
    rightMotor.backward(0.5)
    time.sleep(0.63)
    stop_moving()

def turn_around():
    print('turning around')
    leftMotor.forward(0.5)
    rightMotor.backward(0.5)
    time.sleep(1.26)
    stop_moving()

def gyro_turn_left(init_Gz):
	#Read Gyroscope raw value

	gyro_z = read_raw_data(GYRO_ZOUT_H)
	

	Gz = gyro_z/131.0 # yaw rate in degrees/sec
	calibrated_Gz = Gz - init_Gz
	yaw = 0
	leftMotor.forward(0.3)
	rightMotor.backward(0.3)
	while abs(yaw) < TURN_LEFT_YAW:
		gyro_z = read_raw_data(GYRO_ZOUT_H)
		Gz = gyro_z/131.0 # yaw rate in degrees/sec
		calibrated_Gz = Gz - init_Gz
		yaw += calibrated_Gz*timeDiv
		sleep(timeDiv)

def gyro_turn_right(init_Gz):
	#Read Gyroscope raw value

	gyro_z = read_raw_data(GYRO_ZOUT_H)
	

	Gz = gyro_z/131.0 # yaw rate in degrees/sec
	calibrated_Gz = Gz - init_Gz
	yaw = 0
	leftMotor.backward(0.3)
	rightMotor.forward(0.3)
	while abs(yaw) < TURN_RIGHT_YAW:
		gyro_z = read_raw_data(GYRO_ZOUT_H)
		Gz = gyro_z/131.0 # yaw rate in degrees/sec
		calibrated_Gz = Gz - init_Gz
		yaw += calibrated_Gz*timeDiv
		sleep(timeDiv)

def gyro_turn_around(init_Gz):
	#Read Gyroscope raw value

	gyro_z = read_raw_data(GYRO_ZOUT_H)
	

	Gz = gyro_z/131.0 # yaw rate in degrees/sec
	calibrated_Gz = Gz - init_Gz
	yaw = 0
	leftMotor.backward(0.3)
	rightMotor.forward(0.3)
	while abs(yaw) < TURN_AROUND_YAW:
		gyro_z = read_raw_data(GYRO_ZOUT_H)
		Gz = gyro_z/131.0 # yaw rate in degrees/sec
		calibrated_Gz = Gz - init_Gz
		yaw += calibrated_Gz*timeDiv
		sleep(timeDiv)

def gyro_forward(init_Gz):
	#Read Gyroscope raw value
	yaw = 0
	leftMotor.forward(0.6)
	rightMotor.forward(0.6)
	while True:
		gyro_z = read_raw_data(GYRO_ZOUT_H)
		Gz = gyro_z/131.0 # yaw rate in degrees/sec
		calibrated_Gz = Gz - init_Gz
		yaw += calibrated_Gz*timeDiv
		sleep(timeDiv)
		if yaw < -0.05: 
			print("gotta go left")
			leftMotor.forward(0.5)
			rightMotor.forward(0.7)
		elif yaw > 0.05:
			print("gotta go right")
			leftMotor.forward(0.7)
			rightMotor.forward(0.5)
		else:
			print("onwards")
			leftMotor.forward(0.6)
			rightMotor.forward(0.6)
		



bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

yaw = 0 # initialize to 0
timeDiv = 0.001 # reading every timeDiv seconds
iterations = 0
# CALIBRATE FOR YAW
init_gyro_z = read_raw_data(GYRO_ZOUT_H) 
init_Gz = init_gyro_z/131.0


sensors = []

i2c = board.I2C()

tca = adafruit_tca9548a.TCA9548A(i2c)

leftMotor = PhaseEnableMotor(5, 12)
rightMotor = PhaseEnableMotor(6, 13)

# scan for devices connected to the multiplexer
for channel in range(8):
    if tca[channel].try_lock():
        print("Channel {}:".format(channel), end="")
        addresses = tca[channel].scan()
        print([hex(address) for address in addresses if address != 0x70])
        tca[channel].unlock()

sensors.append(adafruit_vl6180x.VL6180X(tca[1])) # forward facing 
sensors.append(adafruit_vl6180x.VL6180X(tca[0])) # left facing
sensors.append(adafruit_vl6180x.VL6180X(tca[2])) # right facing
# sensors.append(adafruit_vl6180x.VL6180X(tca[3]))

actions = [stop_moving, go_right, go_left]

# We want to be able to tell if more than 1 readings says we should turn. 
counts = [0] * 3

for i in range(4):
    gyro_turn_left(init_Gz)
stop_moving()

sleep(1.5)

# for i in range(4):
#     gyro_turn_right(init_Gz)
# stop_moving()

# sleep(1.5)

# for i in range(4):
#     gyro_turn_around(init_Gz)
# stop_moving()

# forward()
gyro_forward(init_Gz)
sleep(2)
stop_moving()