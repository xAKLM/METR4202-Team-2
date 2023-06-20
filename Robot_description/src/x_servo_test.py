import pigpio
import time

if __name__ == "__main__":
    servoPin = 12

    pi = pigpio.pi()
    pi.set_mode(servoPin, pigpio.OUTPUT)
    
    pi.set_servo_pulsewidth(servoPin, 1200)
    time.sleep(2)
    pi.set_servo_pulsewidth(servoPin, 1500)

    pi.stop()
