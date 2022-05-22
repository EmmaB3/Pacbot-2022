import board
import adafruit_tca9548a

import busio
import time
import adafruit_vl6180x

sensors = []

i2c = board.I2C()

tca = adafruit_tca9548a.TCA9548A(i2c)

# scan for devices connected to the multiplexer
for channel in range(8):
    if tca[channel].try_lock():
        print("Channel {}:".format(channel), end="")
        addresses = tca[channel].scan()
        print([hex(address) for address in addresses if address != 0x70])
        tca[channel].unlock()

sensors.append(adafruit_vl6180x.VL6180X(tca[0]))
sensors.append(adafruit_vl6180x.VL6180X(tca[1]))
sensors.append(adafruit_vl6180x.VL6180X(tca[2]))
sensors.append(adafruit_vl6180x.VL6180X(tca[3]))

while True:
	print("Sensor 1 reading: " + str(sensors[0].range) + "cm")
	print("Sensor 2 reading: " + str(sensors[1].range) + "cm")
	print("Sensor 3 reading: " + str(sensors[2].range) + "cm")
	print("Sensor 4 reading: " + str(sensors[3].range) + "cm")
	time.sleep(0.1)