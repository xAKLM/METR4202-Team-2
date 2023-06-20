from gpiozero import Servo
from time import sleep

def write(servo, angle):
    try:
        val = angle/45.0 - 1.0
        servo.value = val
        print(val)
    except KeyboardInterrupt:
        print("Servo Stopped")

if __name__ == "__main__":
    servo = Servo(12)
    
    write(servo, 45)

    sleep(1)
    write(servo, 45)
    sleep(1)

    servo.value = -1
    sleep(1)
    servo.value = 1
    sleep(1);
