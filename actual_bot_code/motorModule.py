#!/usr/bin/env python3

# NAME: motorModule.py
# PURPOSE: module for driving the bot based on given direction commands
# AUTHORS: Emma Bethel, Rob Pitkin, Ryan McFarlane

import adafruit_tca9548a
import board
import os
import robomodules as rm

from simple_pid import PID
from enum import Enum
from distanceSensor import DistanceSensor
from gpiozero import PhaseEnableMotor
from messages import MsgType, message_buffers, PacmanDirection, GyroYaw
from variables import *
from ports import TcaPort

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 100

# target yaw values for turning 90/-90/180 degrees
TURN_LEFT_YAW = 8
TURN_RIGHT_YAW = 8
TURN_AROUND_YAW = 16.5

# maxumum discrepancy between actual yaw and target yaw upon completion of a 
#   turn
YAW_ALLOWANCE = 0.01

# adjustment to right motor speed in order to drive both sides at same speed 
#   (due to imperfect center of mass)
RIGHT_MOTOR_OFFSET =  -0.07

# multiplier for making turning speed proportional to distance from target yaw
YAW_MULTIPLIER = 0.15

# number of ticks to wait between completing a turn and continuing to drive
#   forward (need some time to zero gyro)
TICK_WAIT = 7

# base speed at which robot will operate (when driving straight)
BASE_SPEED = 0.65

# maximum number of ticks robot can stay in one grid square while attempting to 
#   drive forward before it is considered stalled/trapped and in need of 
#   self-correction
MAX_STALL = 200
# maxium length (in ticks) of a self correction attempt
MAX_REVERSE = 100

# distance sensor indices
CENTER = 0
LEFT = 1
RIGHT = 2


class DriveMode(Enum):
    STOPPED = 0
    TURNING = 1
    STRAIGHT = 2
    CORRECT = 3


class TurnDirection(Enum):
    LEFT = 0
    RIGHT = 1
    AROUND = 2


# TODO: would be cool to add PID control (https://pypi.org/project/simple-pid/) 
#   based off gyro value when turning? maybe
class MotorModule(rm.ProtoModule):
    def __init__(self, addr, port):
        self.subscriptions = [MsgType.PACMAN_DIRECTION, MsgType.GYRO_YAW, MsgType.LIGHT_STATE]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.current_direction = PacmanDirection.D
        self.desired_direction = PacmanDirection.STOP
        self.mode = None
        self.turn_direction = None
        self.tick_counter = 0
        self._yaw = 0
        self.ticks_since_start = 0
        self.stall_counter = 0
        self.prev_tick_pos = False
        self.reverse_counter = 0
        self.pacbot_pos = (0, 0)
        # set up pid
        self.pid = PID(0.1, 0.01, 0.005, setpoint=0, sample_time=1.0/FREQUENCY)
        self.pid.output_limits = (-0.15, 0.15)
        self.pid.auto_mode = False

        # motors
        self.left_motor = PhaseEnableMotor(5, 12)
        self.right_motor = PhaseEnableMotor(6, 13)

        # distance sensors-- should these be their own module?? worried about 
        #   the latency though
        i2c = board.I2C()
        self.tca = adafruit_tca9548a.TCA9548A(i2c)

        # no clue what this does but i assume it's important
        for channel in range(8):
            if self.tca[channel].try_lock():
                print("Channel {}:".format(channel), end="")
                addresses = self.tca[channel].scan()
                print([hex(address) for address in addresses if address != 0x70])
                self.tca[channel].unlock()

        self.front_dist = DistanceSensor('left', self.tca[TcaPort.FRONT_DIST], self._stop, 55)
        self.right_dist = DistanceSensor('right', self.tca[TcaPort.RIGHT_DIST], self._veer_right, 30)
        self.left_dist = DistanceSensor('forward', self.tca[TcaPort.LEFT_DIST], self._veer_left, 30)
        self.dist_sensors = [self.left_dist, self.front_dist, self.right_dist]

    @property
    def yaw(self):
        return self._yaw

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PACMAN_DIRECTION:
            self.desired_direction = msg.direction
            print(f'got direction {msg}')
        elif msg_type == MsgType.GYRO_YAW:
            self._yaw = msg.yaw
        elif msg_type == MsgType.LIGHT_STATE:
             self.pacbot_pos = (msg.pacman.x, msg.pacman.y)

    def tick(self):
        if self.desired_direction != PacmanDirection.STOP and self.pacbot_pos == self.prev_tick_pos:
            self.stall_counter += 1
        else:
            self.stall_counter = 0
        self.prev_tick_pos = self.pacbot_pos

        self.ticks_since_start += 1
        if self.tick_counter > 0:
            self.tick_counter -= 1

        if self.desired_direction == PacmanDirection.STOP:
            self._zero_gyro()
            if self.mode != DriveMode.STOPPED:
                self.mode = DriveMode.STOPPED
                self._stop()
        elif self.stall_counter > MAX_STALL:
            self.mode = DriveMode.CORRECT
        elif self.current_direction != self.desired_direction:
            self.mode = DriveMode.TURNING
        else:
            self.mode = DriveMode.STRAIGHT

        if self.mode == DriveMode.CORRECT:
            self.reverse_counter += 1
            self._drive(- BASE_SPEED, - (BASE_SPEED + RIGHT_MOTOR_OFFSET))
            if self.reverse_counter >= MAX_REVERSE:
                 self.stall_counter = 0
        elif self.mode == DriveMode.TURNING:
            print(f'desired: {self.desired_direction}, current: {self.current_direction}')
            # starting turn
            if self.turn_direction is None:
                # self._zero_gyro()
                self._stop()
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
                self.tick_counter = TICK_WAIT 
                self._zero_gyro()
                self._stop()
                self.current_direction = self.desired_direction
                self.turn_direction = None

        elif self.mode == DriveMode.STRAIGHT:
            if self.tick_counter == 0 and not self.front_dist.is_too_close():
                self.pid.auto_mode = True
                self._drive_straight()
            else:
                self.pid.auto_mode = False
                self._stop()
        else:
            self.desired_direction = PacmanDirection.STOP

    # PURPOSE: zeroes the stored gyro value (both locally and in gyro module)
    # PARAMETERS: N/A
    # RETURNS: N/A
    # NOTE: the zeroing in the gyro module is done by sending a message over 
    #       the local robotmodules server, and thus will not be instantaneous
    def _zero_gyro(self):
        msg = GyroYaw()
        msg.yaw = 0
        self._yaw = 0
        self.write(msg.SerializeToString(), MsgType.GYRO_YAW)

    # PURPOSE: decides neccessary direction to turn (left, right, or 180 
    #          degrees) in order to get robot from current direction to desired 
    #          one
    # PARAMETERS: N/A
    # RETURNS: the desired TurnDirection (enum value)
    def _pick_turn_direction(self):
        # yes i am doing arithmetic on the raw integer values of enums. no i 
        #   will not apologize <3
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

    # PURPOSE: drives robot forward in the current direction, using both gyro 
    #          and distance sensors to stay straight and avoid walls
    # PARAMETERS: N/A
    # RETURNS: N/A
    def _drive_straight(self):
        print(f'left dist sensor {self.left_dist.range}, right dist sensor {self.right_dist.range}')
        if self.left_dist.is_too_close():
            print('swerving to avoid left wall')
            self.left_dist.avoid()
        elif self.right_dist.is_too_close():
            print('swerving to avoid right wall')
            self.right_dist.avoid()
        else:
            print('driving straight with gyro')
            self._drive_straight_gyro()

    # PURPOSE: drives robot forward while keeping yaw (from gyro) as close to
    #          zero as possible
    # PARAMETERS: N/A
    # RETURNS: N/A
    def _drive_straight_gyro(self):
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED + RIGHT_MOTOR_OFFSET

        print(f'yaw: {self.yaw}')
        motor_speedup = min(0.15, YAW_MULTIPLIER * (abs(self.yaw)))
        # motor_speedup = abs(self.pid(self.yaw))
        print(f'motor speedup {motor_speedup}')
        if self.yaw < - YAW_ALLOWANCE:
            print("gotta go left")
            right_speed += motor_speedup
        elif self.yaw > YAW_ALLOWANCE:
            print("gotta go right")
            left_speed += motor_speedup
        else:
            print("onwards")
        if self.front_dist.obstructed():
            print('obstructed!')
            left_speed = left_speed * 0.5
            right_speed = right_speed * 0.5
        print(f'left speed: {left_speed}, right speed: {right_speed}')
        self._drive(left_speed, right_speed)
    
    # PURPOSE: drives robot at given speed
    # PARAMETERS: left_speed - the speed for the left wheel. should be in range 
    #                          [-1.0, 1.0] where negative means backward, 
    #                          positive forward
    #             right_speed - the speed for the right wheel. same range as 
    #                           left.
    # RETURNS: N/A
    # NOTE: if left or right speed < -1 or > 1, it will be capped at the 
    #       value corresponding to its sign
    def _drive(self, left_speed, right_speed):
        # if self.ticks_since_start < 10:
        #     left_speed = left_speed * 1 / (ticks_since_start + 1.0)
        #     right_speed = right_speed * (1 / (ticks_since_start + 1.0))
        if left_speed < 0:
            self.left_motor.backward(min(1.0, -left_speed))
        else:
            self.left_motor.forward(min(1.0, left_speed))
        if right_speed < 0:
            self.right_motor.backward(min(1.0, -right_speed))
        else:
            self.right_motor.forward(min(1.0, right_speed))

    # PURPOSE: drives robot away from an obstacle to its left
    # PARAMETERS: N/A
    # RETURNS: N/A
    def _veer_left(self):
        print('veering left')
        self._drive(0.3 , 0.4 + RIGHT_MOTOR_OFFSET)

    # PURPOSE: drives robot away from an obstacle to its right
    # PARAMETERS: N/A
    # RETURNS: N/A
    def _veer_right(self):
        print('veering right')
        self._drive(0.4, 0.3 + RIGHT_MOTOR_OFFSET)

    # PURPOSE: stops robot driving
    # PARAMETERS: N/A
    # RETURNS: N/A
    def _stop(self):
        self.ticks_since_start = 0
        self.left_motor.stop()
        self.right_motor.stop()

    # PURPOSE: turns robot in a desired direction until it reaches a desired 
    #          yaw value
    # PARAMETERS: target - desired yaw value for robot to turn to
    #             left - boolean; True if robot should turn left 
    #                    (counterclockwise), False otherwise
    # RETURNS: True if robot has reached desired yaw value, False otherwise
    def _turn(self, target, left):
        turn_speed = 0.1 + min((target - abs(self.yaw)) * 0.25, 0.3)
        print(f'yaw: {self.yaw}, target: {target}, speed: {turn_speed}')
        if abs(self.yaw) < target:
            if left:
                self._drive(-turn_speed, turn_speed + RIGHT_MOTOR_OFFSET)
            else:
                self._drive(turn_speed, -turn_speed - RIGHT_MOTOR_OFFSET)
            return False
        else:
            return True

    # PURPOSE: turns robot 90 degrees to the right (clockwise)
    # PARAMETERS: N/A
    # RETURNS: True if robot has reached desired yaw value, False otherwise
    def _turn_right(self):
        print('turning right')
        return self._turn(TURN_RIGHT_YAW, False)

    # PURPOSE: turns robot 90 degrees to the left (counterclockwise)
    # PARAMETERS: N/A
    # RETURNS: True if robot has reached desired yaw value, False otherwise
    def _turn_left(self):
        print('turning left')
        return self._turn(TURN_LEFT_YAW, True)

    # PURPOSE: turns robot 180 degrees to the left (counterclockwise)
    # PARAMETERS: N/A
    # RETURNS: True if robot has reached desired yaw value, False otherwise
    def _turn_around(self):
        print('turning around')
        return self._turn(TURN_AROUND_YAW, True)

    # PURPOSE: depracated; drives robot forward while veering away from side 
    #          walls using distance sensors.
    # PARAMETERS: N/A
    # RETURNS: N/A
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


def main():
    module = MotorModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()
