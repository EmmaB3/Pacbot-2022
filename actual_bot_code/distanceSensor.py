import adafruit_vl6180x


MIN_DIST = 50
READING_AMOUNT = 2

class DistanceSensor:
    def __init__(self, name, tca_channel, avoidance_action):
        self.name = name
        self.avoid = avoidance_action
        self.sensor = adafruit_vl6180x.VL6180X(tca_channel)
        self.count = 0
    
    @property
    def value(self):
        return self.sensor.range
    
    def is_too_close(self):
        if self.value < MIN_DIST:
            self.count += 1
            if self.count > READING_AMOUNT:
                return True
        else:
            self.count = 0
            return False
    
    def __str__(self):
        return self.name
