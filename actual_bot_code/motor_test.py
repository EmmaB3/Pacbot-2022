from gpiozero import PhaseEnableMotor
import time

left_motor = PhaseEnableMotor(5, 12)
right_motor = PhaseEnableMotor(6, 13)

left_motor.forward(0.6)
right_motor.forward(0.63)

while True:
	print('driving')
	time.sleep(1)
