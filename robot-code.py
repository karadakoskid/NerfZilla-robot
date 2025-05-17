import cv2
import numpy as np
import pigpio
from cvzone.FaceDetectionModule import FaceDetector

# Initialize Camera
cap = cv2.VideoCapture(0)
ws, hs = 640, 480
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

# Initialize pigpio for servo and motor control
pi = pigpio.pi()
if not pi.connected:
    print("Pigpio daemon is not running!")
    exit()

# Servo control pins
servoX_pin = 17  # GPIO pin for horizontal servo
servoY_pin = 27  # GPIO pin for vertical servo
servoExtra_pin = 22  # New servo (moves when face is locked)

# DC motor control pins
motor1_pin = 23  # GPIO pin for motor 1
motor2_pin = 24  # GPIO pin for motor 2

# Set servos to initial positions
initial_servoX = 1500  # 90 degrees (center)
initial_servoY = 1000  # 40 degrees (starting position)
initial_servoExtra = 2000  # 40 degrees (starting position)

pi.set_servo_pulsewidth(servoX_pin, initial_servoX)
pi.set_servo_pulsewidth(servoY_pin, initial_servoY)
pi.set_servo_pulsewidth(servoExtra_pin, initial_servoExtra)

# Initialize motors to be started at the beginning
pi.set_PWM_dutycycle(motor1_pin, 128)  # Start motor 1 at 50% speed
pi.set_PWM_dutycycle(motor2_pin, 128)  # Start motor 2 at 50% speed

# Face detector
detector = FaceDetector()
servoPos = [initial_servoX, initial_servoY]  # Initial servo positions

# Adjusted servo range for Y-axis to limit vertical movement
servoY_min = 900  # Minimum pulse width for vertical movement (upwards)
servoY_max = 1000  # Maximum pulse width for vertical movement (downwards)

# Define motor speed (0 to 255)
motor_speed = 128  # 50% speed

# Extra servo motion range
servoExtra_min = 1400  # 40 degrees
servoExtra_max = 2000  # 80 degrees
servoExtra_pos = servoExtra_min  # Start position
servoExtra_direction = 1  # 1 for increasing, -1 for decreasing

# Speed factor for servo X (increase for faster movement)
speed_factor = 0.3  # Adjust the speed of servo movement (lower value means faster)

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        # Get face coordinates
        fx, fy = bboxs[0]["center"]

        # Map the face's X coordinate to the servo's X pulse width
        target_servoX = np.interp(fx, [0, ws], [2000, 1300])

        # Apply speed factor for faster movement of servoX
        servoX = int(servoPos[0] + (target_servoX - servoPos[0]) * speed_factor)

        # Map the face's Y coordinate to the vertical servo pulse width
        servoY = np.interp(fy, [0, hs], [servoY_max, servoY_min])

        servoPos[0] = int(servoX)
        servoPos[1] = int(servoY)

        # Update servo positions
        pi.set_servo_pulsewidth(servoX_pin, servoPos[0])
        pi.set_servo_pulsewidth(servoY_pin, servoPos[1])

        # Move extra servo between 40° and 80° while face is locked
        if servoExtra_direction == 1:
            servoExtra_pos += 20  # Increase step size for faster movement
            if servoExtra_pos >= servoExtra_max:
                servoExtra_pos = servoExtra_max
                servoExtra_direction = -1  # Change direction
        else:
            servoExtra_pos -= 20  # Increase step size for faster movement
            if servoExtra_pos <= servoExtra_min:
                servoExtra_pos = servoExtra_min
                servoExtra_direction = 1  # Change direction

        pi.set_servo_pulsewidth(servoExtra_pin, servoExtra_pos)

        # Draw tracking indicators
        cv2.circle(img, (fx, fy), 50, (0, 0, 255), 2)
        cv2.putText(img, "TARGET LOCKED", (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

    else:
        cv2.putText(img, "NO TARGET", (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)

        # Reset extra servo to 40° when no face is detected
        pi.set_servo_pulsewidth(servoExtra_pin, initial_servoExtra)

    # Display servo values
    cv2.putText(img, f'Servo X: {servoPos[0]} us', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {servoPos[1]} us', (50, 150), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Extra Servo: {servoExtra_pos} us', (50, 200), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
pi.set_servo_pulsewidth(servoX_pin, 0)
pi.set_servo_pulsewidth(servoY_pin, 0)
pi.set_servo_pulsewidth(servoExtra_pin, 0)
pi.set_PWM_dutycycle(motor1_pin, 0)
pi.set_PWM_dutycycle(motor2_pin, 0)
pi.stop()