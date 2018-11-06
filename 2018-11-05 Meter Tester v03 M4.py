# 2018-11-05 Meter Tester v03 M4.py
# Feather M4-based analog meter tester utilizing the Retro CV Meter Wing
# (c) 2018 John Park's Workshop with Cedar Grove Studios
#
# The Meter Tester measures and displays analog meter parameters. The
#   meter voltage is monitored by a probe connection from analog input A4.
#   Up to +1.65mA current is output from the Retro CV Meter Wing to the
#   meter under test. The test potentiometer (attached to analog
#   input A3) is manually adjusted from zero to a level that causes the
#   meter to deflect to full-scale. The OLED continuously displays
#   meter parameters and recommended DAC resistor value.
#
# The Feather's A0~ DAC output pin provides a voltage that
#   is converted to a current through a series resistor to the analog ammeter.
#   The maximum 12-bit DAC value (65520 in the 16-bit CPy range of values)
#   produces 1.65mA to deflect the meter to full-scale when the series resistor
#   value is 2000 ohms, the recommended minimum series resistor value.
#

import digitalio
import pulseio  # for LED brightness control
import busio  # for the OLED wing
from analogio import AnalogIn, AnalogOut
import board
import time
from simpleio import map_range
import adafruit_ssd1306  # OLED wing
import neopixel as neo # on-board NeoPixel
import microcontroller  # for checking CPU temperature
import gc  # for checking memory capacity

# ### Setup ###
i2c = busio.I2C(board.SCL, board.SDA)
oled = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c)
oled.fill(0)
oled.show()

# analog inputs and meter output
probe_pin = AnalogIn(board.A4)
test_pin = AnalogIn(board.A3)
meter_pin = AnalogOut(board.A0)
meter_pin.value = 0  # Position the meter to zero

# on-board activity
led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT
led.value = True

# GPIO for pwm indicators
# neg_led = pulseio.PWMOut(board.D5)  # negative red LED
# pos_led = pulseio.PWMOut(board.D6)  # positive green LED
amber_led = pulseio.PWMOut(board.D9)  # absolute value amber LED
amber_led.duty_cycle = 32000  # turn on amber LED for general illumination

# piezo speaker (PWM output)
piezo = pulseio.PWMOut(board.D10, duty_cycle=0, frequency=440, variable_frequency=True)

# dim the on-board NeoPixel, white for startup
pixel = neo.NeoPixel(board.NEOPIXEL,1, brightness=0.01, auto_write=False)
pixel[0] = (200, 200, 200)
pixel.write()
time.sleep(0.1)

# ### Dictionaries and Lists ###
scales = [(-10, 10), (-8, 8), (-5, 5), (0, 5), (0, 8), (0, 10)]  # for music

# ### Helpers ###
def beep_on(f=440):  # start PWM piezo beeper
    piezo.frequency = f
    piezo.duty_cycle = int(65535/2)  # tone on, 50%
    return True

def beep_off(w=0):  # stop PWM piezo beeper after wait
    if w != 0:
        time.sleep(w)
    piezo.duty_cycle = 0  # tone off
    return True
    
def hello_A():  # play hello tune in key of A major
    beep_on(880)  # A5
    beep_off(0.25)
    beep_on(1109)  # C#6
    beep_off(0.25)
    beep_on(1319)  # E6
    beep_off(0.25)
    time.sleep(0.25)
    return True

# ### Set Initial Parameters ###
output_hold = 2  # number of seconds to hold measurement display
dac_resist = 39.89e3 # the PCB's series meter resistor value in ohms
test_volt = 0
probe_volt = 0

print("2018-11-05 Meter Tester v03 M4.py")
hello_A()

# splash screen
oled.text("Meter Tester v03", 0, 0)
oled.text("  Free:", 0, 8)
oled.text(str(int(gc.mem_free()/1e2)/10), 64, 8)
oled.text("  Freq: ", 0, 16)
oled.text(str(int(microcontroller.cpu.frequency/1e5)/10), 64, 16)
oled.text("  Temp:", 0, 24)
oled.text(str(int(microcontroller.cpu.temperature*10)/10), 64, 24)
oled.show()
time.sleep(3)

# information screen
oled.fill(0)
oled.text("Meter Tester v03", 0, 0)
oled.text("  Adjust from", 0, 8)
oled.text("  zero setting", 0, 16)
oled.text("  to full-scale", 0, 24)
oled.show()
time.sleep(3)

pixel[0] = (0, 0, 200)  # blue when operating
pixel.write()
led.value = False  # turn off the activity LED

# start the display timer
t0 = time.monotonic()

# ### Main Loop ###
while True:
    meter_pin.value = test_pin.value  # set the meter output voltage to the test voltage value

    if time.monotonic() > t0 + output_hold:  # change the display based on output_hold value
        # read the test voltage from the potentiometer and meter probe inputs
        #   scale from pseudo 12-bit value to actual voltage, accurate to three places
        test_volt = int(((test_volt + map_range(test_pin.value, 0, 65520, 0, 3.3)) / 2)*1e3)/1e3
        probe_volt = int(((probe_volt + map_range(probe_pin.value, 0, 65520, 0, 3.3)) / 2)*1e3)/1e3

        if probe_volt > test_volt: probe_volt = test_volt  # prevent negative values
        
        # calculate meter current and voltage values
        meter_current = (test_volt - probe_volt) / dac_resist
        meter_volt = probe_volt

        # calculate meter and DAC resistances
        if (test_volt - probe_volt) != 0:
            meter_resist = (probe_volt * dac_resist) / (test_volt - probe_volt)
            new_dac_resist = (3.3 - meter_volt) / meter_current
        else:
            meter_resist = None
            new_dac_resist = None
            
        oled.fill(0)
        oled.show()
        led.value = True  # turn on the activity LED
        
        if test_volt <= 0.05:  # don't start until the test voltage is > 50mV
            oled.text("LOW TEST VOLTAGE", 0, 0)
        else:
            oled.text("M mA=", 0, 0)  # meter milliamps, accurate 3 places
            oled.text(str(int(meter_current * 1e6)/1e3), 64, 0)
            oled.text("M mV=", 0, 8)  # meter millivolts, accurate 3 places
            oled.text(str(int(meter_volt * 1e4)/10), 64, 8)
            if meter_resist != None:
                oled.text("M ohms=", 0, 16)  # meter resistance
                oled.text(str(meter_resist), 64, 16)
                oled.text("DAC R =", 0, 24)  # recommended DAC resistor at +3.3V
                oled.text(str(int(new_dac_resist)), 64, 24)
            else:
                oled.text(" ---  Check  ---", 0, 16)
                oled.text("   Connections", 0, 24)

        oled.show()
        t0 = time.monotonic()

    time.sleep(output_hold/10)  # hold the test voltage for a while
    led.value = False  # turn off the activity LED