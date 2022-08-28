##################################################
##################################################
# Miniature HVAC System Control Algorithm
# @ Junhyeok Hong 2022-08
# Takes SHT30 sensor measurements and controls
# LN298 Motor with Thermo-Electric Cooler Module
# Inspired code for SHT30 sensor measurement: https://github.com/ControlEverythingCommunity/SHT30
# Inspired code for LN298 Motor Control: https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-pi/
##################################################
##################################################


from ossaudiodev import control_labels
from termios import IEXTEN
import RPi.GPIO as GPIO
from time import sleep
import smbus
import time
import numpy as np


def pid_control(first_cycle, pv, pv_old, time_step, ie):
    Kp = 0  # P-Gain
    Ki = 0  # I-Gain
    Kd = 0  # D-Gain

    sp = 20  # Room Heating Set POint
    delta_t = time_step
    dpv = 0

    if (Kp == 0) & (Ki == 0) & (Kd == 0):
        pv_old = pv

    e = sp - pv

    if first_cycle is False:
        ie += e * delta_t
        dpv = (pv - pv_old) / delta_t

    P = Kp * e
    I = Ki * ie
    D = Kd * dpv

    value = P + I + D

    if value > 1:  # Check upper limit
        value = 1
        if first_cycle is False:
            ie -= e * delta_t  # anti-reset windup
    if value < 0:  # Check lower limit
        value = 0
        if first_cycle is False:
            ie -= e * delta_t  # anti-reset windup

    pv_old = pv

    if first_cycle is True:
        first_cycle = False

    control_output = 0

    return control_output, first_cycle, ie, pv_old


# GPIO Hook-up for LN298
in1 = 24
in2 = 23
en = 25
temp1 = 1

# GPIO set-up for LN298
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
GPIO.output(in1, GPIO.LOW)
GPIO.output(in2, GPIO.LOW)
p = GPIO.PWM(en, 1000)  # Maybe change PWM?
p.start(100)

first_cycle = True
pv_old = 0
time_step = 1
ie = 0

for x in range(500):
    # Get I2C bus
    bus = smbus.SMBus(1)

    # SHT30 address, 0x44(68)
    # Send measurement command, 0x2C(44)
    # 0x06(06) - High repeatability measurement
    bus.write_i2c_block_data(0x44, 0x2C, [0x06])

    time.sleep(0.5)

    # SHT30 address, 0x44(68)
    # Read data back from 0x00(00), 6 bytes
    # pv : Current Temperature (MSB, LSB, CRC) - Celsius
    data = bus.read_i2c_block_data(0x44, 0x00, 6)
    pv = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45

    control_output, first_cycle, ie, pv_old = pid_control(
        first_cycle, pv, pv_old, time_step, ie)

    if control_output == 1:  # Heating
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif control_output == 0:  # Cooling
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
    elif control_output == -1:  # Stop
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

    time.sleep(0.5)
