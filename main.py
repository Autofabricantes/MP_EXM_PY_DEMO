import Adafruit_GPIO.SPI as SPI  # Needed for the MCP3008 module
import MCP3008
import PCA9685  # adafruit_python_gpio dependency
import PID
import open_myo as myo

MOTOR_CTRL_MAX = 4096  # PWM resolution = 12 bit
PID_KP = 10
PID_KI = 0
PID_KD = 0
FINGER_PWM_CH = 0
FINGER_ADC_CH = 0

""" Software SPI configuration for the MCP3008: """
CLK  = 18
MISO = 23
MOSI = 24
CS   = 25

adc = MCP3008.MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
pwm = PCA9685.PCA9685()
pid = PID.PID(PID_KP, PID_KI, PID_KD    )

pwm.set_pwm_freq(60)
pid.setWindup = MOTOR_CTRL_MAX
pid.SetPoint = 20;

""" With these three lines of code, the control of a single motor is achieved.
First the feedback value of the controller is obtained from a specific channel
of the ADC IC. Then, the control signal is computed with a PID. Finally, the
control signal is applied to the motor as a PWM signal generated by the PWM IC.
"""
feedback = mcp.read_adc(FINGER_ADC_CH)
pid.update(feedback)
pwm.set_pwm(FINGER_PWM_CH, 0, pid.output)