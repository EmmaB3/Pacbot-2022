#!/usr/bin/env python3
import os
from motorModule import MotorModule
from distanceSensor import DistanceSensor

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
PORT = os.environ.get("LOCAL_PORT", 11295)


class ReversibleMotorModule(MotorModule):
    def __init__(self, addr, port):
        super().__init__(addr, port)
        self.reversed = False
        self.back_dist = DistanceSensor('left', self.tca[0], self._stop, 40)

    def _turn_around(self):
        print('overridden turning around')
        # flip reversed flag
        self.reversed = not self.reversed

        # flip front and back distance sensors
        temp = self.front_dist
        self.front_dist = self.back_dist
        self.back_dist = temp
    
        return True

    @property
    def yaw(self):
        if self.reversed:
            return -1 * self._yaw
        return self._yaw

    def _drive(self, left_speed, right_speed):
        if self.reversed:
            left_speed *= -1
            right_speed *= -1

        super()._drive(left_speed, right_speed)

    def _turn_right(self):
        if self.reversed:
            return super()._turn_left()
        return super()._turn_right()

    def _turn_left(self):
        if self.reversed:
           return super()._turn_right()
        return super()._turn_left()

def main():
    module = ReversibleMotorModule(ADDRESS, PORT)
    module.run()


if __name__ == "__main__":
    main()
