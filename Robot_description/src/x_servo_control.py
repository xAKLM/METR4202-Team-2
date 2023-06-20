import pigpio
import time

if __name__ == "__main__":
    servoPin = 12
    start = 1200
    closing = True

    pi = pigpio.pi()
    pi.set_mode(servoPin, pigpio.OUTPUT)

    while (1):
        print(start)
        pi.set_servo_pulsewidth(servoPin, start)
        time.sleep(1.5)
        
        if start <= 1100:
                closing = True
        elif start >= 1600:
                closing = False

        if closing:
            start = start + 50
        else:
            start = start - 50

