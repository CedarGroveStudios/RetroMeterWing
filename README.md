# RetroMeterWing

A FeatherWing utility board that converts a +/-10V CV signal to a current level compatible with a retro analog ammeter. Attaches to an Adafruit Feather (M0, M0 Express, or M4 Express) running CircuitPython. Can optionally stand alone as an analog-only module if 3.3V power is supplied from an external regulator. From the collaborative project with John Park’s Workshop.

![Retro CV Meter Wing and M4 Express Rendering](https://github.com/CedarGroveStudios/RetroMeterWing/blob/master/retro%20meter%20v03.png)

The Retro CV Meter board accepts a -10 to +10 volt signal and through an analog op-amp, translates it into a 0 to +3.3V volt signal connected to the Adafruit Feather M0 or M4's A2 analog input pin. The A0~ DAC output pin provides current output through a 2K-ohm resistor to the analog meter connection. The maximum DAC value (65535) produces +1.5mA to the attached meter.

The CircuitPython code reads the input voltage and adjusts the DAC output for the scale for the meter. The ``inertia_move`` helper integrates the incoming signal then sends the scaled and smoothed voltage to the on-board meter resistor to convert the voltage to current compabile with the analog meter. The integration routine protects the meter movement from sudden voltage changes.

The most recent version of the code reads a meter scale input from a potentiometer attached to pin A3. Pins D5, D6, and D9 can be used to provide PWM outputs proportional to the value and polarity of the input signal.

![Retro CV Meter and M4 Express Close Rendering](https://github.com/CedarGroveStudios/RetroMeterWing/blob/master/retro%20meter%20v03%20bottom.png)
![Retro CV Meter and M4 Express Close Rendering](https://github.com/CedarGroveStudios/RetroMeterWing/blob/master/retro%20meter%20v03%20piezo.png)
![Retro CV Meter and M4 Express Close Rendering](https://github.com/CedarGroveStudios/RetroMeterWing/blob/master/retro%20meter%20v03%20close.png)
