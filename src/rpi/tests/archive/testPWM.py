# External module imports
import RPi.GPIO as GPIO
import time

# Pin Definitons:
pwmPin = 18 # Broadcom pin 18 (P1 pin 12)
enablePin = 17

dc = 30 # duty cycle (0-100) for PWM pin

# Pin Setup:
GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
GPIO.setup(pwmPin, GPIO.OUT) # PWM pin set as output
GPIO.setup(enablePin, GPIO.OUT)
#pwm = GPIO.PWM(pwmPin, dc)  # Initialize PWM on pwmPin 100Hz frequency

# Initial state for LEDs:
GPIO.output(enablePin, GPIO.LOW)
GPIO.output(pwmPin, GPIO.LOW)
#pwm.start(dc)

print("Here we go! Press CTRL+C to exit")
try:
    while 1:
        dc = 100-dc
#        pwm.ChangeDutyCycle(dc)
        if dc == 30:
            GPIO.output(pwmPin, GPIO.HIGH)
            GPIO.output(enablePin, GPIO.LOW)
            print(1)
        else:
            GPIO.output(pwmPin, GPIO.LOW)
            GPIO.output(pwmPin, GPIO.HIGH)
            print(2)
        time.sleep(5)
except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
#    pwm.stop() # stop PWM
    GPIO.cleanup() # cleanup all GPIO
