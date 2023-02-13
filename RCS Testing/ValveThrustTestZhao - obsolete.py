# GPIO 22

import time
import sys
import RPi.GPIO as GPIO
from hx711 import HX711
import csv
referenceUnit=1
m = -.0014
b = 0.4498 # calibration

# Pin #15 of HX711 is "Rate" Output data rate control. 
# 10 Hz set to 0. 80 Hz set to 1

def cleanAndExit():
    print("Cleaning...")

    GPIO.cleanup()
        
    print("Bye!")
    sys.exit()
    
def printWeight(start_time):
    # to use both channels, you'll need to tare them both
    #hx.tare_A()
    #hx.tare_B()
    val = (m*(hx.get_value(1) + b))
    # calibrated value = m*val + b
    # calibrate in Excel using linear curve fit with calibration weights
    
    # print(time.time()-start_time,val)
    
    # To get weight from both channels (if you have load cells hooked up 
    # to both channel A and B), do something like this
    #val_A = hx.get_weight_A(5)
    #val_B = hx.get_weight_B(5)
    #print "A: %s  B: %s" % ( val_A, val_B )
    return [time.time()-start_time,val]

def write_csv(filename,data):
    with open(filename,'w',newline='') as csvfile:
        f = csv.writer(csvfile)
        for row in data:
            f.writerow(row)

hx = HX711(5, 6)
hx.set_reading_format("MSB", "MSB")
hx.RATE = 1

# GPIO.setmode(GPIO.BCM)
# GPIO.setup(22, GPIO.OUT)

# GPIO.output(22, GPIO.LOW)
print("valve off")

# HOW TO CALCULATE THE REFFERENCE UNIT
# To set the reference unit to 1. Put 1kg on your sensor or anything you have and know exactly how much it weights.
# In this case, 92 is 1 gram because, with 1 as a reference unit I got numbers near 0 without any weight
# and I got numbers around 184000 when I added 2kg. So, according to the rule of thirds:
# If 2000 grams is 184000 then 1000 grams is 184000 / 2000 = 92.
#hx.set_reference_unit(113)
hx.set_reference_unit(referenceUnit)

hx.tare()

print("Tare done! Add weight now...")

timeEnd = time.time() + 5 # seconds

data = []
start_time = time.time()
while time.time() < timeEnd:
    data.append(printWeight(start_time))

for i in range(10):
    # GPIO.output(22, GPIO.HIGH)
    print("valve on")
    timeEnd = time.time() + .5 # seconds
    while time.time() < timeEnd:
         data.append(printWeight(start_time))

    # GPIO.output(22, GPIO.LOW)
    print("valve off")
    timeEnd = time.time() + .5 # seconds
    while time.time() < timeEnd:
         data.append(printWeight(start_time))


print(data)
write_csv('test',data)
#'date orificenozzletype_hertz_feedsystem'
#write_csv('2022-01-17 1mmConical_10Hz_Tank .5 sec pulse',data)
#'date orificenozzletype_hertz_feedsystem'
# Write_csv('2022-12-07 Benchtop 10 sec 1mm conical.csv 3 of 3',data)
cleanAndExit()
