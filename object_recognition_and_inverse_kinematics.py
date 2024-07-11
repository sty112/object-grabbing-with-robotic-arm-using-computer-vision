import imutils
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import serial
import time
cap = cv2.VideoCapture(1)
cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
fx = 837.3315184425294
fy = 838.4441305218588
px = 335.3847326542147  # principal point x
py = 253.93443638782276  # principal point y
arduino = serial.Serial('com5', 9600)
i = 0


def inverse_kinematics(X, Y, Z_object):
    L1 = 114
    L2 = 159
    L3 = 105
    L4 = 141
    servo_1 = 0
    servo_3 = 0
    servo_4 = 0
    servo_5 = 0
    dis_armtocam = 236  # mm
    X = X-1
    print(X)
    Y = float(Y+dis_armtocam)
    print(Y)
    Z_object = float(Z_object)
    R_angle = math.sqrt(np.square(X)+np.square(Y))
    Z_offset = L1 - Z_object
    D_2 = L4 - Z_offset
    R = math.sqrt(np.square(D_2) + np.square(R_angle))
    print(R)
    try:
        angle_1 = math.atan(D_2/R_angle)*180/math.pi
        angle_2 = math.acos(
            ((np.square(R))+(np.square(L2))-(np.square(L3)))/(2*L2*R))*180/math.pi
        angle_3 = math.acos(
            ((np.square(R))+(np.square(L3))-(np.square(L2)))/(2*R*L3))*180/math.pi
        angle_4 = 90 - angle_1
        theta_1 = math.atan(X/Y)*180/math.pi
        theta_2 = angle_1 + angle_2
        theta_3 = math.acos(
            ((np.square(L2))+(np.square(L3))-(np.square(R)))/(2*L2*L3))*180/math.pi
        theta_4 = angle_3 + angle_4
        servo_1 = theta_2+15
    #    servo_2 = 180-theta_2-5
        servo_3 = theta_3-90+8
        servo_4 = theta_4-90+10
        servo_5 = 90 + theta_1
    except ValueError as e:
        print(e)
    return servo_1, servo_3, servo_4, servo_5


while True:
    ret, frame = cap.read()
    while i < 1:
        time.sleep(5)
        i = i+1
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([160, 20, 70])
    upper_red = np.array([190, 255, 255])
    lower_green = np.array([36, 25, 25])
    upper_green = np.array([86, 255, 255])
    lower_yellow = np.array([22, 93, 0])
    upper_yellow = np.array([45, 255, 255])
    lower_blue = np.array([101, 50, 38])
    upper_blue = np.array([110, 255, 255])

    mask_1 = cv2.inRange(hsv, lower_red, upper_red)
    mask_2 = cv2.inRange(hsv, lower_green, upper_green)
    mask_3 = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_4 = cv2.inRange(hsv, lower_yellow, upper_yellow)

    cnts1 = cv2.findContours(mask_1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # find the contours from the image and send a list of contour
    cnts1 = imutils.grab_contours(cnts1)
    # extract the contours from the returned tuple

    cnts2 = cv2.findContours(mask_2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts2 = imutils.grab_contours(cnts2)

    cnts3 = cv2.findContours(mask_3, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts3 = imutils.grab_contours(cnts3)

    cnts4 = cv2.findContours(mask_4, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts4 = imutils.grab_contours(cnts4)
    for c in cnts1:
        area1 = cv2.contourArea(c)
        # calculates area of the contour
        if area1 > 1000:
            # we detect only objects with area (pixels^2) bigger than 1000
            peri = cv2.arcLength(c, True)
            # computes the perimeter of the contour
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)
            # approximates the contour shape by reducing the number of vertices
            if len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                shape = "Square"
            else:
                shape = "Circle"
            # Drawing the shape and label
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.circle(frame, (int(px), int(py)), 3, (0, 0, 255), -1)
            Z = 33.5  # in cm
            Z_object = 10  # in mm
            # The equation X = (x - cx) * Z / fx converts the x pixel coordinate of an object in an image to its corresponding x coordinate in real-world 3D space
            X = (cx - px) * 10 * Z / fx
            Y = (cy - py) * 10 * Z / fy
            cv2.putText(frame, f"{shape} (Red)", (cx-20, cy-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
            s1, s3, s4, s5 = inverse_kinematics(X, Y, Z_object)
            s1 = round(s1)
            s3 = round(s3)
            s4 = round(s4)
            s5 = round(s5)
            print(
                f"for red servo_1 angle is: {s1}, servo_3 angle is {s3}, servo_4 angle is {s4}, servo_5 angle is {s5}, the colour is Red, the shape is {shape}")
            value = f'{s1},{s3},{s4},{s5},Red,{shape}\r'
            arduino.write(value.encode())

    for c in cnts2:
        area2 = cv2.contourArea(c)
        if area2 > 1000:
            # Detecting shapes
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)
            if len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                shape = "Square"
            else:
                shape = "Circle"
            # Drawing the shape and label
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.circle(frame, (int(px), int(py)), 3, (0, 0, 255), -1)
            Z = 33.5  # in cm
            Z_object = 10  # in mm
            # The equation X = (x - cx) * Z / fx converts the x pixel coordinate of an object in an image to its corresponding x coordinate in real-world 3D space
            X = (cx - px) * 10 * Z / fx
            Y = (cy - py) * 10 * Z / fy
            cv2.putText(frame, f"{shape} (Green)", (cx-20, cy-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
            s1, s3, s4, s5 = inverse_kinematics(X, Y, Z_object)
            s1 = round(s1)
            s3 = round(s3)
            s4 = round(s4)
            s5 = round(s5)
            print(
                f"for green servo_1 angle is: {s1}, servo_3 angle is {s3}, servo_4 angle is {s4}, servo_5 angle is {s5}, the colour is green, the shape is {shape}")
            value = f'{s1},{s3},{s4},{s5},Green,{shape}\r'
            arduino.write(value.encode())

    for c in cnts4:
        area4 = cv2.contourArea(c)
        if area4 > 1000:
            # Detecting shapes
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.03 * peri, True)
            print(len(approx))
            if len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                shape = "Square"
            else:
                shape = "Circle"
            # Drawing the shape and label
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
            M = cv2.moments(c)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
            cv2.circle(frame, (int(px), int(py)), 3, (0, 0, 255), -1)
            Z = 33.5  # in cm
            Z_object = 10  # in mm
            # The equation X = (x - cx) * Z / fx converts the x pixel coordinate of an object in an image to its corresponding x coordinate in real-world 3D space
            X = (cx - px) * 10 * Z / fx
            Y = (cy - py) * 10 * Z / fy
            cv2.putText(frame, f"{shape} (Yellow)", (cx-20, cy-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, 255)
            s1, s3, s4, s5 = inverse_kinematics(X, Y, Z_object)
            s1 = round(s1)
            s3 = round(s3)
            s4 = round(s4)
            s5 = round(s5)
            print(
                f"for yellow servo_1 angle is: {s1}, servo_3 angle is {s3}, servo_4 angle is {s4}, servo_5 angle is {s5}, the colour is yellow, the shape is {shape}")
            value = f'{s1},{s3},{s4},{s5},Yellow,{shape}\r'
            arduino.write(value.encode())

    if arduino.in_waiting > 0:
        # Read the data from Arduino
        data = arduino.readline().decode().strip()

        # Split the data into individual values
        s1, s3, s4, s5, colour, shape = data.split(',')

        # Process the received values
        # ...

        # Print the received values
        print(
            f"Received data: s1={s1}, s3={s3}, s4={s4}, s5={s5}, colour={colour}, shape={shape}")
    cv2.imshow("webcam", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
