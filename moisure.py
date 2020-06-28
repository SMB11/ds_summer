#!/usr/bin/env python3
import PCF8591 as ADC
import RPi.GPIO as GPIO
import time

DO = 17
GPIO.setmode(GPIO.BCM)


def setup():
    ADC.setup(0x48)
    GPIO.setup(DO, GPIO.IN)


def loop():
    status = 1
    while True:
        print('Value: ', ADC.read(1))
        print('Value 2: ', ADC.read(2))

        time.sleep(2)


if __name__ == '__main__':
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        pass
