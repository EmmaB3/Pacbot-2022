from gpiozero import PhaseEnableMotor
from time import sleep

#motor = PhaseEnableMotor(phase[direction], enable[PWM])
motor1 = PhaseEnableMotor(5, 12)
motor2 = PhaseEnableMotor(6, 13)


while True: 
  motor1.forward(0.5)
  motor2.backward(0.5)