import RPi.GPIO as GPIO
import time

OUT_PIN = 5   # BCM 5 (pin 29)
IN_PIN  = 6   # BCM 6 (pin 31)

GPIO.setmode(GPIO.BCM)
GPIO.setup(OUT_PIN, GPIO.OUT)
GPIO.setup(IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

try:
    while True:
        GPIO.output(OUT_PIN, GPIO.HIGH)
        time.sleep(1)
        print("OUT=HIGH  IN=", GPIO.input(IN_PIN))

        GPIO.output(OUT_PIN, GPIO.LOW)
        time.sleep(0.1)
        print("OUT=LOW   IN=", GPIO.input(IN_PIN))

except KeyboardInterrupt:
    GPIO.cleanup()
