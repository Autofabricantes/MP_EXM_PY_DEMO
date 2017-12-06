"""adc"""
import RPi.GPIO as GPIO
import time
import Adafruit_GPIO.SPI as SPI
from MCP3008 import MCP3008
from PCA9685 import PCA9685

MPOT_0 = 1  # ADC channel to which potentiometer is connected

""" Software SPI configuration for the MCP3008: """
CLK  = 18
MISO = 23
MOSI = 24
CS   = 25

adc = MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

"""motor"""

PCA_I2C_ADDR = 0x40
MOTOR_CTRL_MAX = 4095  # PWM resolution = 12 bit
MOT_0_PWM = 0

GPIO.setmode(GPIO.BCM)
MOT_0_A_CTRL = 20
MOT_0_B_CTRL = 21
GPIO.setup(MOT_0_A_CTRL, GPIO.OUT)
GPIO.setup(MOT_0_B_CTRL, GPIO.OUT)

pwm = PCA9685.PCA9685(PCA_I2C_ADDR)

pwm.set_pwm_freq(60)

def motor_control(duty_cycle):
    if(duty_cycle >= 0):
        pwm.set_pwm(MOT_0_PWM, 0, duty_cycle)  # set motor speed
        GPIO.output(MOT_0_A_CTRL, GPIO.LOW)  # set pin LOW
        GPIO.output(MOT_0_B_CTRL, GPIO.HIGH)  # set pin HIGH
    else:
        pwm.set_pwm(MOT_0_PWM, 0, abs(duty_cycle))  # set motor speed
        GPIO.output(MOT_0_A_CTRL, GPIO.HIGH)  # set pin HIGH
        GPIO.output(MOT_0_B_CTRL, GPIO.LOW)  # set pin LOW

while(True):
    try:
        duty_cycle = int(input("Enter PWM duty cycle (min: -4096, max: 4096): "))
        motor_control(duty_cycle)
        feedback = adc.read_adc(MPOT_0)
        print(feedback)
        time.sleep(0.5)
    except Exception as err:
        print(err)
        break

