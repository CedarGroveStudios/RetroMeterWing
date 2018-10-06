# 2018-10-06 Retro CV Meter v04 M4Ex.py
# Feather M4 Express-based CV monitor with analog current meter output
# (c) 2018 John Park's Workshop with Cedar Grove Studios
#
# The Retro CV Meter board accepts a -10 to +10 volt CV signal and outputs
#   a 0 to 1.5mA signal to a connected analog ammeter. The PCB attenuates and
#   shifts the incoming signal, translating it into a 0 to +3.3 volt signal
#   compatible with the Feather's A2 analog input pin.
#
# The Feather's A0~ DAC output pin provides a voltage that is converted to a
#   current through a 2K-ohm resistor to the analog ammeter. The maximum
#   16-bit DAC value (65535) produces 1.5mA to deflect the meter to full-scale.
#   The series resistor can be altered to adjust to various meters, as needed.
#
# This code reads the input voltage and adjusts the scale for the meter. The
#   inertia_move helper integrates and sends the scaled voltage to the series
#   meter resistor. Integration protects the meter movement from sudden voltage
#   changes.
#

import digitalio
import pulseio
from analogio import AnalogIn, AnalogOut
import board
import time
from simpleio import map_range
import neopixel as neo

import microcontroller  # for checking CPU temperature
import gc  # for checking memory capacity

# ### Setup ###
# CV input and meter output
cv_pin = AnalogIn(board.A2)
scale_pin = AnalogIn(board.A3)
meter_pin = AnalogOut(board.A0)

# Feather battery voltage monitor
battery_pin = AnalogIn(board.VOLTAGE_MONITOR)

# on-board activity indicator (when moving the meter needle)
led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT

# GPIO for pwm indicators
neg_led = pulseio.PWMOut(board.D5)  # negative red LED
pos_led = pulseio.PWMOut(board.D6)  # positive green LED
amber_led = pulseio.PWMOut(board.D9)  # absolute value amber LED

# piezo speaker (PWM output)
piezo = pulseio.PWMOut(board.D10, duty_cycle=0, frequency=440, variable_frequency=True)

# dim the on-board NeoPixel, white for startup
pixel = neo.NeoPixel(board.NEOPIXEL,1, brightness=0.01, auto_write=False)
pixel[0] = (200, 200, 200)
pixel.write()
time.sleep(0.1)

# ### Dictionaries and Lists ###

scales = [(-10, 10), (-8, 8), (-5, 5), (0, 5), (0, 8), (0, 10)]

# ### Helpers ###
def inertia_move(v=0, target_v=0, rate=1, increment=1, low_scale=-10, high_scale=10):
    # This helper moves the output voltage from the current value to the
    # target value at the specified rate (seconds per input volt). Output
    # resolution is controlled by increment and is measured in volts per step.
    # The output scale is controlled by the values of low_scale and high_scale.

    led.value = True  # turn on activity indicator

    # limit the voltage increment to a reasonable value
    if increment <= 0:
        increment = 0.001  # minimum increment of 1mV/step
    if increment > 5:
        increment = 5      # maximum increment of 5V/step

    # This is the proportional movement control. It determines the
    # number of steps needed to move from the current value to the target
    # value based on the increment value.
    #
    delta_v = target_v - v
    steps = delta_v / increment
    if steps == 0:  # have to take at least one step
        steps = 1

    # This portion of code steps from the current value to the target value
    # over time specified by the rate.
    #
    # This is where input voltage is scaled before sending to the DAC output.
    # The meter's full-scale voltage are controlled by the low_scale and
    # high_scale parameters
    #
    for i in range(1+int(abs(steps))):
        meter_pin.value = int(map_range(v, low_scale, high_scale, 0, 65535))
        time.sleep(rate * increment)
        v = v + ((steps / abs(steps)) * increment)
    v = target_v

    led.value = False  # clear activity indicator
    return (v)  # return new v value

# ctl_idx converts an analog value to index integer
#   Input is masked to 8 bits to reduce noise then
#   hysteresis offset is applied. (ItsyBitsy M4 DAC and ADC are 12-bit;
#   CircuitPython converts to 16-bits.) The helper
#   returns a new index value and an input hysteresis offset.
def ctl_idx(ctl, max, offset, old_idx):
    if (ctl + offset > 65535) or (ctl + offset < 0): offset = 0
    idx = int(map_range((ctl + offset) & 0xFF00,1200,65500, 0, max))
    if idx > old_idx: offset = 2000
    if idx < old_idx: offset = -2000
    return idx, offset
    
def beep_on(f=440):  # start PWM piezo beeper
    piezo.frequency = f
    piezo.duty_cycle = int(65535/2)  # tone on, 50%
    return True

def beep_off(w=0):  # stop PWM piezo beeper after wait
    if w != 0:
        time.sleep(w)
    piezo.duty_cycle = 0  # tone off
    return True

def click():  # play click sound/beep
    beep_on(440)  # A4
    beep_off(0.025)
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
#
# The following parameters set rate, increment, and outut hold. You should
#   experiment with these parameters to get the right "feel" for the particular
#   application and analog meter.
#
move_rate = 0.050    # seconds to move 1 volt on a -10 to +10 volt scale
move_inc = 0.050     # smallest movement representing input voltage
output_hold = 0  # number of seconds to hold position before repositioning

# Initialize the hysteresis and scale values
scale_max = 6
scale_idx = 0
scale_offset = 0
meter_low, meter_high = scales[0]  # set default meter output scale values

print("2018-10-06 Retro CV Meter v04 M4Ex.py")
print("GC.mem_free:  %0.3f" % float(gc.mem_free()/1000), "KB")
print("CPU.freqency:   %0.1f" % float(microcontroller.cpu.frequency/1000000), "MHz")
print("CPU.temperature: %0.1f" % microcontroller.cpu.temperature, "C")
print("Battery voltage:  %0.1f" % map_range(battery_pin.value, 0, 65535, 0, 6.6), "V")

hello_A()

# Slowly center (zero) the meter's output before reading the input voltage.
#   This should take about one second to move the needle to the center.
meter_out_volt = inertia_move(-10, 0, 0.200, 0.050)
time.sleep(0.500)  # hold at center scale for 0.500 seconds

t0 = time.monotonic()  # set a timer for the scale potentiometer input rate

# ### Main Loop ###
while True:
    if time.monotonic() > t0 + 0.500:  #read the scale potentiometer every 500msec
        scale_idx_old = scale_idx  # remember the index for the previous range
        (scale_idx, scale_offset) = ctl_idx(scale_pin.value, scale_max, scale_offset, scale_idx_old) # determine scale index
        if scale_idx != scale_idx_old:  # if the index changed, blink the amber led
            amber_led.duty_cycle = 0
            click()
            amber_led.duty_cycle = 65535
            meter_low, meter_high = scales[scale_idx]  # set the meter scale limits
        t0 = time.monotonic()  # reset the scale potentiometer input timer

    # Read the CV input and map to a +/-10V value.
    #   A 3.3V input maps to a value of +10V. DO NOT adjust these values.
    cv_input_volt = map_range(cv_pin.value, 0, 65535, -10, +10)

    # Show the input voltage value and polarity on the DotStar indicator and leds
    if cv_input_volt > +1.0:    # 1V noise threshold
        pixel[0] = (0, 255, 0)  # green = positive
        pixel.write()
        # pos_led.duty_cycle = int(map_range(cv_input_volt, 1, meter_high, 0, 65535))
        amber_led.duty_cycle = int(map_range(cv_input_volt, 1, meter_high, 4096, 65535))
    elif cv_input_volt < -1.0:  # 1V noise threshold
        pixel[0] = (255, 0, 0)  # red = negative
        pixel.write()
        # neg_led.duty_cycle = int(map_range(cv_input_volt, -1, meter_low, 0, 65535))
        amber_led.duty_cycle = int(map_range(cv_input_volt, -1, meter_low, 4096, 65535))
    else:
        pixel[0] = (0, 0, 255)  # blue = zero
        pixel.write()
        # pos_led.duty_cycle = 0
        # neg_led.duty_cycle = 0
        amber_led.duty_cycle = 4096

    # Send the scaled input voltage to the analog meter via the inertia_move
    #   helper to the Trinket DAC output.
    meter_out_volt = inertia_move(meter_out_volt, cv_input_volt, move_rate, move_inc, meter_low, meter_high)

    time.sleep(output_hold)  # Hold the output voltage for a while.