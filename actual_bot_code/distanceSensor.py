import adafruit_vl6180x


MIN_DIST = 50
READING_AMOUNT = 2

class DistanceSensor:
    def __init__(self, name, tca_channel, avoidance_action, min_dist):
        self.name = name
        self.min_dist = min_dist
        self.avoid = avoidance_action
        self.sensor = adafruit_vl6180x.VL6180X(tca_channel)
        self.count = 0
    
    @property
    def range(self):
        return self.sensor.range
    
    def is_too_close(self):
        if self.range < self.min_dist:
            self.count += 1
            if self.count > READING_AMOUNT:
                return True
        else:
            self.count = 0
            return False
    
    def obstructed(self):
        return self.range < 100
    
    def __str__(self):
        return self.name
