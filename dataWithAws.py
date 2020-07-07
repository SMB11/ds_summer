import os
import glob
import time
from time import sleep
from picamera import PiCamera
import boto3

import PCF8591 as ADC  # analog to digital converter

import RPi.GPIO as GPIO
from datetime import datetime

os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]
device_file = device_folder + '/w1_slave'

camera = PiCamera()

DHTPIN = 17  # humidity sensor pin

DO = 17

GPIO.setmode(GPIO.BCM)

MAX_UNCHANGE_COUNT = 100

STATE_INIT_PULL_DOWN = 1
STATE_INIT_PULL_UP = 2
STATE_DATA_FIRST_PULL_DOWN = 3
STATE_DATA_PULL_UP = 4
STATE_DATA_PULL_DOWN = 5

# //////////////////

dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table('ds_project')

# ///////////////////////


def read_temp_raw():
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    return lines


def read_temp():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()
    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        # temp_f = temp_c * 9.0 / 5.0 + 32.0
        #  temp_f
        return temp_c

# Humility sensor get data


def read_dht11_dat():
    GPIO.setup(DHTPIN, GPIO.OUT)
    GPIO.output(DHTPIN, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(DHTPIN, GPIO.LOW)
    time.sleep(0.02)
    GPIO.setup(DHTPIN, GPIO.IN, GPIO.PUD_UP)

    unchanged_count = 0
    last = -1
    data = []
    while True:
        current = GPIO.input(DHTPIN)
        data.append(current)
        if last != current:
            unchanged_count = 0
            last = current
        else:
            unchanged_count += 1
            if unchanged_count > MAX_UNCHANGE_COUNT:
                break

    state = STATE_INIT_PULL_DOWN

    lengths = []
    current_length = 0

    for current in data:
        current_length += 1

        if state == STATE_INIT_PULL_DOWN:
            if current == GPIO.LOW:
                state = STATE_INIT_PULL_UP
            else:
                continue
        if state == STATE_INIT_PULL_UP:
            if current == GPIO.HIGH:
                state = STATE_DATA_FIRST_PULL_DOWN
            else:
                continue
        if state == STATE_DATA_FIRST_PULL_DOWN:
            if current == GPIO.LOW:
                state = STATE_DATA_PULL_UP
            else:
                continue
        if state == STATE_DATA_PULL_UP:
            if current == GPIO.HIGH:
                current_length = 0
                state = STATE_DATA_PULL_DOWN
            else:
                continue
        if state == STATE_DATA_PULL_DOWN:
            if current == GPIO.LOW:
                lengths.append(current_length)
                state = STATE_DATA_PULL_UP
            else:
                continue
    if len(lengths) != 40:
        #print ("Data not good, skip")
        return False

    shortest_pull_up = min(lengths)
    longest_pull_up = max(lengths)
    halfway = (longest_pull_up + shortest_pull_up) / 2
    bits = []
    the_bytes = []
    byte = 0

    for length in lengths:
        bit = 0
        if length > halfway:
            bit = 1
        bits.append(bit)
    #print ("bits: %s, length: %d" % (bits, len(bits)))
    for i in range(0, len(bits)):
        byte = byte << 1
        if (bits[i]):
            byte = byte | 1
        else:
            byte = byte | 0
        if ((i + 1) % 8 == 0):
            the_bytes.append(byte)
            byte = 0
    #print (the_bytes)
    checksum = (the_bytes[0] + the_bytes[1] +
                the_bytes[2] + the_bytes[3]) & 0xFF
    if the_bytes[4] != checksum:
        #print ("Data not good, skip")
        return False

    return the_bytes[0], the_bytes[2]


# photoresistor part

def photoresistorSetup():
    ADC.setup(0x48)
    GPIO.setup(DO, GPIO.IN)

  # start all


file = open("/home/pi/data_logSt.csv", "a")


photoresistorSetup()
# photoresValue = ADC.read(0)
resultSt = read_dht11_dat()
humiditySt, temperatureSt = resultSt
# temp = read_temp()


print("Temperature: " + str(read_temp()) + " Light: "+str(ADC.read(0)) +
      " Humidity: " + str(humiditySt)+"  Temperature 2: " + str(temperatureSt)+" Moisure 1 " + str(ADC.read(1))+" Moisure 2 " + str(ADC.read(2)))

i = 0
if os.stat("/home/pi/data_logSt.csv").st_size == 0:
    file.write(
        "Time,Temperature 1,Photoresistor Value,Humidity,Temperature 2,Plant1Moisure,Plant2Moisure,Environment,Plant1Size,Plant2Size\n")
while True:
    i = i+1
    now = datetime.now()
    result = read_dht11_dat()

    if result:
        humidity, temperature = result
    else:
        humidity = humiditySt
        temperature = temperatureSt

    # print(result)
    camera.capture('./firstTest/{}.jpg'.format(now))
    try:
        table.put_item(
            Item={
                'Date': str(now),
                'Temperature 1': str(read_temp()),
                'Photoresistor Value': str(ADC.read(0)),
                'Humidity': str(humidity),
                'Temperature 2': str(temperature),
                'Plant1Moisure': str(ADC.read(1)),
                'Plant2Moisure': str(ADC.read(2)),
                'Environment': '1'
            }
        )
    except ClientError as e:
        print(e)

    file.write(str(now)+","+str(read_temp())+","+str(ADC.read(0)) +
               ","+str(humidity)+","+str(temperature)+","+str(ADC.read(1))+","+str(ADC.read(2)) + ","+"1" + "\n")
    print(str(now)+","+str(read_temp())+","+str(ADC.read(0)) +
          ","+str(humidity)+","+str(temperature)+","+str(ADC.read(1))+","+str(ADC.read(2)) + ","+"1" + "\n")
    file.flush()
    time.sleep(3600)

file.close()

# while True:
#     print("Temperature: "+ str(read_temp()))
#     time.sleep(0.5)
#     result = read_dht11_dat()
#     if result:
#         humidity, temperature = result
#         print("humidity: %s %%,  Temperature: %s C`" % (humidity, temperature))
#     time.sleep(0.5)
#     print('Value of Photoresistor: ', ADC.read(0))
#     time.sleep(1)
