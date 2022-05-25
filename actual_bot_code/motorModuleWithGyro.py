#!/usr/bin/env python3

# NAME: motorModuleWithGyro.py
# PURPOSE: depracated version of motorModule that reads angular position data 
#          directly from the gyro (instead of receiving yaw messages from 
#          gyroModule)
# AUTHORS: Emma Bethel, Ryan McFarlane
# NOTES: not gonna bother to document/clean up most of this so. read at your 
#        own risk (it's pretty much a combination of motorModule.py and 
#        gyroModule.py, and was also not used in the final competition code; 
#        something about having a lot of other stuff happen in the tick 
#        function makes the integral approximation super inaccurate)

import adafruit_tca9548a
import board
import os
import robomodules as rm

from enum import Enum
from distanceSensor import DistanceSensor
from gpiozero import PhaseEnableMotor
from gyro import Gyro
from messages import MsgType, message_buffers, PacmanDirection
from variables import *

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 1000

TURN_LEFT_YAW = 0.23
TURN_RIGHT_YAW = 0.23
TURN_AROUND_YAW = 0.4

YAW_ALLOWANCE = 0.001

RIGHT_MOTOR_OFFSET = 0

MIN_SPEED = 0.1

# distance sensor indices
CENTER = 0
LEFT = 1
RIGHT = 2


class DriveMode(Enum):
    STOPPED = 0
    TURNING = 1
    STRAIGHT = 2


class TurnDirection(Enum):
    LEFT = 0
    RIGHT = 1
    AROUND = 2


# TODO: would be cool to add PID control (https://pypi.org/project/simple-pid/) 
#   based off gyro value when turning? maybe
class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.PACMAN_DIRECTION]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.current_direction = PacmanDirection.W
        self.desired_direction = PacmanDirection.STOP
        self.mode = None
        self.turn_direction = None

        # gyro
        self.gyro = Gyro()
        self.yaw = 0

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
            DistanceSensor('forward', tca[1], self._stop, 50),
            DistanceSensor('left', tca[0], self._veer_right, 20),
            DistanceSensor('right', tca[2], self._veer_left, 20)
        ]

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PACMAN_DIRECTION:
            self.desired_direction = msg.direction
        print(f'got message {msg}')

    def tick(self):
        # if self.mode != DriveMode.STOPPED:
        #     done = self._turn_right(5.69)
        #     if done:
        #         self._stop()
        #         self.mode = DriveMode.STOPPED
        # if told to stop, stop
        # print(f'desired: {self.desired_direction} current: {self.current_direction}')
        if self.dist_sensors[CENTER].is_too_close():
            self.desired_direction = PacmanDirection.STOP
 
        if self.desired_direction == PacmanDirection.STOP:
            if self.mode != DriveMode.STOPPED:
                self.mode = DriveMode.STOPPED
                self._stop()
        elif self.current_direction != self.desired_direction:
            self.mode = DriveMode.TURNING
        else:
            self.mode = DriveMode.STRAIGHT

        if self.mode == DriveMode.TURNING:
            print(f'desired: {self.desired_direction}, current: {self.current_direction}')
            # starting turn
            if self.turn_direction is None:
                self._stop()
                self.yaw = 0
                self.turn_direction = self._pick_turn_direction()
                print(f'zeroing yaw, turn direction {self.turn_direction}')

            # continuing an already underway turn
            done = False
            if self.turn_direction == TurnDirection.AROUND:
                done = self._turn_around()
            elif self.turn_direction == TurnDirection.RIGHT:
                done = self._turn_right()
            else:
                done = self._turn_left()

            # completing turn
            if done:
                self.yaw = 0
                self.current_direction = self.desired_direction
                self.turn_direction = None
                self._stop()
                # self.mode = DriveMode.STOPPED # TEMP

        elif self.mode == DriveMode.STRAIGHT:
            self._drive_straight()
            # self._drive(MIN_SPEED, MIN_SPEED)
        else:
            self.desired_direction = PacmanDirection.STOP

    # returns turn direction (enum value)
    def _pick_turn_direction(self):
        # warning: quirky math shit ahead (basically i'm using the fact that 
        #   the directions are in an enum, w/ values 0 through 4, to inform the 
        #   choice of direction to turn)
        offset = self.desired_direction - self.current_direction
        print(f'PICK TURN DIRECTION {offset}')
        if abs(offset) == 2:
            return TurnDirection.AROUND
        elif offset == 1 or offset == -3:
            return TurnDirection.LEFT
        elif offset == -1 or offset == 3:
            return TurnDirection.RIGHT
        else:  # theoretically this should not happen
            return None

    def _drive_straight(self):
        print(f'left dist sensor {self.dist_sensors[LEFT].range}, right dist sensor {self.dist_sensors[RIGHT].range}')
        if self.dist_sensors[LEFT].is_too_close():
            print('swerving to avoid left wall')
            self.dist_sensors[LEFT].avoid()
        elif self.dist_sensors[RIGHT].is_too_close():
            print('swerving to avoid right wall')
            self.dist_sensors[RIGHT].avoid()
        else:
            print('driving straight with gyro')
            self._drive_straight_gyro()

    def _drive_straight_gyro(self):
        left_speed = 0.6
        right_speed = 0.6 + RIGHT_MOTOR_OFFSET
        self.yaw += (1.0 / FREQUENCY) * self.gyro.value
        print(f'yaw: {self.yaw}')
        motor_speedup = 10 * abs(self.yaw)
        print(f'motor speedup {motor_speedup}')
        if self.yaw < - YAW_ALLOWANCE:
            print("gotta go left")
            right_speed += motor_speedup
        elif self.yaw > YAW_ALLOWANCE:
            print("gotta go right")
            left_speed += motor_speedup
        else:
            print("onwards")
        if self.dist_sensors[CENTER].obstructed():
            print('obstructed!')
            left_speed = left_speed * 0.5
            right_speed = right_speed * 0.5
        print(f'left speed: {left_speed}, right speed: {right_speed}')
        self.left_motor.forward(min(1.0, left_speed))
        self.right_motor.forward(min(1.0, right_speed))

    def _drive_straight_dist_sensors(self):
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
        self.right_motor.forward(0.6 + RIGHT_MOTOR_OFFSET)

    def _veer_right(self):
        print('veering right')
        self.left_motor.forward(0.6)
        self.right_motor.forward(0.3 + RIGHT_MOTOR_OFFSET)

    def _stop(self):
        print('stop')
        self.left_motor.stop()
        self.right_motor.stop()

    def _drive_forward(self, speed):
        print(f'driving forward at speed {speed}')
        self.left_motor.forward(speed)
        self.right_motor.forward(speed)

    def _drive(self, left_speed, right_speed):
        if left_speed < 0:
            self.left_motor.backward(-left_speed)
        else:
            self.left_motor.forward(left_speed)
        if right_speed < 0:
            self.right_motor.backward(-right_speed)
        else:
            self.right_motor.forward(right_speed)

    # returns whether or not it's done turning
    def _turn(self, target, left):
        self.yaw += ((1.0 / FREQUENCY) * self.gyro.value)
        turn_speed = MIN_SPEED + min(0.1, 10 * (target - abs(self.yaw)))
        print(f'yaw: {self.yaw}, target: {target}, turn speed: {turn_speed}')
        if abs(self.yaw) < target:
            if left:
                self._drive(-turn_speed, turn_speed)
            else:
                self._drive(turn_speed, -turn_speed)
            return False
        else:
            return True

    # returns whether or not it's done turning
    def _turn_right(self):
        print('turning right')
        return self._turn(TURN_RIGHT_YAW, False)

    # returns whether or not it's done turning
    def _turn_left(self):
        print('turning left')
        return self._turn(TURN_LEFT_YAW, True)

    # returns whether or not it's done turning
    def _turn_around(self):
        print('turning around')
        return self._turn(TURN_AROUND_YAW, True)


def main():
    module = MotorModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()

