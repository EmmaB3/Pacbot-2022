#!/usr/bin/env python3
from enum import Enum
from messages.pacmanDirection_pb2 import PacmanDirection
import adafruit_tca9548a
import board
from gpiozero import PhaseEnableMotor
import os
import robomodules as rm
from variables import *
from messages import MsgType, message_buffers
from distanceSensor import DistanceSensor

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 0 # was 60


class DriveMode(Enum):
    STOPPED = 0
    TURNING = 1
    STRAIGHT = 2


# TODO: would be cool to add PID control (https://pypi.org/project/simple-pid/) 
#   based off gyro value when turning? maybe
class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.PACMAN_DIRECTION, MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.current_direction = PacmanDirection.W
        self.desired_direction = None
        self.mode = None

        # motors
        self.left_motor = PhaseEnableMotor(5, 12)
        self.right_motor = PhaseEnableMotor(6, 13)

        # distance sensors-- should these be their own module?? worried about 
        #   the latency though
        i2c = board.I2C()
        tca = adafruit_tca9548a.TCA9548A(i2c)

        # no clue what this does but i assume it's important
        for channel in range(8):
            if tca[channel].try_lock():
                print("Channel {}:".format(channel), end="")
                addresses = tca[channel].scan()
                print([hex(address) for address in addresses if address != 0x70])
                tca[channel].unlock()

        self.dist_sensors = [
            DistanceSensor('forward', tca[1], self._stop),
            DistanceSensor('left', tca[0], self._veer_right),
            DistanceSensor('right', tca[2], self._veer_left)
        ]

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PACMAN_DIRECTION:
            self.desired_direction = msg.direction
        print(f'got message {msg}')

    def tick(self):
        if self.desired_direction == PacmanDirection.STOP:  # ok it's kinda weird that "stop" is considered a direction. maybe have it be a separate boolean in the message???
            self.mode = DriveMode.STOPPED  # should this happen inside the _stop function?
            self._stop()
        if self.current_direction != self.desired_direction:
        #     self.mode = DriveMode.TURNING  # TODO: temporary
        # else:
            self.mode = DriveMode.STRAIGHT
        
        if self.mode == DriveMode.TURNING:
            pass  # TODO: put the gyro stuff in here idk
        else:
            self._drive_straight()
    
    def _drive_straight(self):
        is_straight = True  # homophobia
        for sensor in self.dist_sensors:
            print(f'{sensor} sensor value: {sensor.range}')
            if sensor.is_too_close():
                print(f'{sensor} sensor too close!')
                sensor.action()
                is_straight = False
                break
            else:
                print(f'no action needed for {sensor} sensor')
        
        if is_straight:
            self._drive_forward(0.3)
    
    def _veer_left(self):
        print('veering left')
        self.left_motor.forward(0.3)
        self.right_motor.forward(0.7)
    
    def _veer_right(self):
        print('veering right')
        self.left_motor.forward(0.7)
        self.right_motor.forward(0.3)
    
    def _stop(self):
        print('stop')
        self.left_motor.stop()
        self.left_motor.stop()
    
    def _drive_forward(self, speed):
        print(f'driving forward at speed {speed}')
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)


def main():
    module = MotorModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()
