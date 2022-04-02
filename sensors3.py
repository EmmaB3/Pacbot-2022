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

# We want to be able to tell if more than 1 readings says we should turn. 
left_count = 0
right_count = 0
while True:
    print("Sensor 0 reading: " + str(sensors[0].range) + "cm")
    print("Sensor 1 reading: " + str(sensors[1].range) + "cm")
    #print("Sensor 2 reading: " + str(sensors[2].range) + "cm")
    #print("Sensor 3 reading: " + str(sensors[3].range) + "cm")

    s0_distance = sensors[0].range
    s1_distance = sensors[1].range
    
    #Gets the difference betewen the two sensors
    difference = s0_distance - s1_distance
    
    
    #Turns robot left
    if (difference > 15):
        left_count = left_count + 1
        right_count= 0
        print("here left")
        if (left_count > 1):
             print("turn left")
             
    #Turns robot right         
    elif (difference < -15):
       
        right_count = right_count + 1
        left_count = 0
        print("here right")

        if (right_count > 1):
             print("turn right")
             
    #if there really isn't a difference between the sensors then don't turn at all and reset counters
    else:
        left_count = 0
        right_count = 0 
        continue
    
    time.sleep(1)