import RPi.GPIO as GPIO
from PCA9685 import PCA9685

GPIO.setmode(GPIO.BCM)

MOTOR_CTRL_MAX = 4096  # PWM resolution = 12 bit
MOT_0_A_CTRL = 0
MOT_0_B_CTRL = 1

pwm = PCA9685.PCA9685()

pwm.set_pwm_freq(60)

def motor_control(duty_cycle):
    if(duty_cycle >= 0):
        pwm.set_pwm(MOT_0_A_CTRL, 0, duty_cycle)
        pwm.set_pwm(MOT_0_B_CTRL, 0, 0)  # set pin LOW
    else:
        pwm.set_pwm(MOT_0_A_CTRL, 0, abs(duty_cycle))
        pwm.set_pwm(MOT_0_B_CTRL, 0, 4096)  # set pin HIGH

while(True):
    try:
        duty_cycle = int(input("Enter PWM duty cycle (min: -4096, max: 4096): "))
        motor_control(duty_cycle)
    except Exception as err:
        print(err)
        break
