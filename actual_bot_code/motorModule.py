#!/usr/bin/env python3
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

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)

FREQUENCY = 100

TURN_LEFT_YAW = 7.5
TURN_RIGHT_YAW = 7.5
TURN_AROUND_YAW = 16.0

YAW_ALLOWANCE = 0.01

RIGHT_MOTOR_OFFSET = 0.0

YAW_MULTIPLIER = 0.15

TICK_WAIT = 5

BASE_SPEED = 0.6

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
        self.subscriptions = [MsgType.PACMAN_DIRECTION, MsgType.GYRO_YAW]
        super().__init__(addr, port, message_buffers, MsgType, FREQUENCY, self.subscriptions)
        self.state = None
        self.current_direction = PacmanDirection.D
        self.desired_direction = PacmanDirection.STOP
        self.mode = None
        self.turn_direction = None
        self.tick_counter = 0
        self._yaw = 0
        
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

        self.left_dist = DistanceSensor('left', self.tca[0], self._veer_right, 40)
        self.right_dist = DistanceSensor('right', self.tca[2], self._veer_left, 40)
        self.front_dist = DistanceSensor('forward', self.tca[1], self._stop, 60)
        # self.dist_sensors = [
        #     DistanceSensor('forward', tca[1], self._stop, 70),
        #     DistanceSensor('left', tca[0], self._veer_right, 40),
        #     DistanceSensor('right', tca[2], self._veer_left, 40)
        # ]

    @property
    def yaw(self):
        return self._yaw

    def msg_received(self, msg, msg_type):
        if msg_type == MsgType.PACMAN_DIRECTION:
            self.desired_direction = msg.direction
            print(f'got direction {msg}')
        elif msg_type == MsgType.GYRO_YAW:
            self._yaw = msg.yaw

    def tick(self):
        # print(f'yaw {self.yaw}')
        # if self.mode != DriveMode.STOPPED:
        #     done = self._turn_right(5.69)
        #     if done:
        #         self._stop()
        #         self.mode = DriveMode.STOPPED
        # if told to stop, stop
        # print(f'desired: {self.desired_direction} current: {self.current_direction}')
        if self.tick_counter > 0:
            self.tick_counter -= 1

        if self.desired_direction == PacmanDirection.STOP:
            self._zero_gyro()
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
                # self._stop() # TEMP
                # self.mode = DriveMode.STOPPED # TEMP

        elif self.mode == DriveMode.STRAIGHT:
            if self.tick_counter == 0 and not self.front_dist.is_too_close():
                self.pid.auto_mode = True
                self._drive_straight()
            else:
                self.pid.auto_mode = False
                self._stop()
        else:
            self.desired_direction = PacmanDirection.STOP

    def _zero_gyro(self):
        msg = GyroYaw()
        msg.yaw = 0
        self._yaw = 0
        self.write(msg.SerializeToString(), MsgType.GYRO_YAW)

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

    def _drive_straight_gyro(self):
        left_speed = BASE_SPEED
        right_speed = BASE_SPEED + RIGHT_MOTOR_OFFSET
        # self.yaw += (1.0 / FREQUENCY) * self.gyro.value
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
    
    def _drive(self, left_speed, right_speed):
        if left_speed < 0:
            self.left_motor.backward(min(1.0, -left_speed))
        else:
            self.left_motor.forward(min(1.0, left_speed))
        if right_speed < 0:
            self.right_motor.backward(min(1.0, -right_speed))
        else:
            self.right_motor.forward(min(1.0, right_speed))

    # def _drive_straight_dist_sensors(self):
    #     is_straight = True  # homophobia
    #     for sensor in self.dist_sensors:
    #         print(f'{sensor} sensor value: {sensor.range}')
    #         if sensor.is_too_close():
    #             print(f'{sensor} sensor too close!')
    #             sensor.action()
    #             is_straight = False
    #             break
    #         else:
    #             print(f'no action needed for {sensor} sensor')

    #     if is_straight:
    #         self._drive_forward(0.3)

    def _veer_left(self):
        print('veering left')
        self._drive(0.3 , 0.4 + RIGHT_MOTOR_OFFSET)

    def _veer_right(self):
        print('veering right')
        self._drive(0.4, 0.3 + RIGHT_MOTOR_OFFSET)

    def _stop(self):
        self.left_motor.stop()
        self.right_motor.stop()

    # def _drive_forward(self, speed):
    #     print(f'driving forward at speed {speed}')
    #     self.left_motor.forward(speed)
    #     self.right_motor.forward(speed)

    # returns whether or not it's done turning
    def _turn(self, target, left):
        # self.yaw += (1.0 / FREQUENCY) * self.gyro.value
        turn_speed = 0.1 + min((target - abs(self.yaw)) * 0.25, 0.3)
        print(f'yaw: {self.yaw}, target: {target}, speed: {turn_speed}')
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
    # while True:
    #     print(f'left: {module.dist_sensors[LEFT].range} right: {module.dist_sensors[RIGHT].range}')
    #     time.sleep(0.1)


if __name__ == "__main__":
    main()
