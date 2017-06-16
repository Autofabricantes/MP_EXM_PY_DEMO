import RPi.GPIO as GPIO
import time
from MCP3008 import MCP3008

MPOT_0 = 1

""" Software SPI configuration for the MCP3008: """
CLK  = 18
MISO = 23
MOSI = 24
CS   = 25

adc = MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)

while(True):
    try:
        feedback = adc.read_adc(MPOT_0)
        print(feedback)
        time.sleep(0.5)
    except Exception as err:
        print(err)
        break
