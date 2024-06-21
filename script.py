import time
import numpy as np
import cv2
import math
import pyautogui
import keyboard
import serial

cv2.namedWindow("mask")

height = 1440
width = 2560


def nothing(x):
    pass


low_hsv = (0, 153, 133)
high_hsv = (29, 221, 236)

lh, ls, lv = low_hsv
hh, hs, hv = high_hsv
lwheel1 = 0
rwheel1 = 0
cv2.createTrackbar("lh", "mask", lh, 255, nothing)
cv2.createTrackbar("ls", "mask", ls, 255, nothing)
cv2.createTrackbar("lv", "mask", lv, 255, nothing)
cv2.createTrackbar("hh", "mask", hh, 255, nothing)
cv2.createTrackbar("hs", "mask", hs, 255, nothing)
cv2.createTrackbar("hv", "mask", hv, 255, nothing)

calibration_distance = 50  # см
calibration_linear_size = 51  # pixel
cam = cv2.VideoCapture(0)
m = []
flaga = 0
flagb = 0
ser = serial.Serial('COM4', 9600)

t122 = 0
t12 = 0
while (True):
    m = []
    success, frame = cam.read()
    original_frame = frame
    frame = cv2.flip(frame, 1)
    cv2.imwrite("ball_unknown_distance_2.jpg", original_frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lh = cv2.getTrackbarPos("lh", "mask")
    ls = cv2.getTrackbarPos("ls", "mask")
    lv = cv2.getTrackbarPos("lv", "mask")
    hh = cv2.getTrackbarPos("hh", "mask")
    hs = cv2.getTrackbarPos("hs", "mask")
    hv = cv2.getTrackbarPos("hv", "mask")

    mask = cv2.inRange(hsv, (lh, ls, lv), (hh, hs, hv))
    # print("low_hsv = ", (lh, ls, lv), "high_hsv = ", (hh, hs, hv))

    cv2.imshow("mask", mask)

    connectivity = 4
    output = cv2.connectedComponentsWithStats(mask, connectivity, cv2.CV_32S)

    # Get the results
    # The first cell is the number of labels
    num_labels = output[0]
    # The second cell is the label matrix
    labels = output[1]
    # The third cell is the stat matrix
    stats = output[2]

    filtered = np.zeros_like(mask)
    if len(m) > 1: m.pop(0)
    for i in range(1, num_labels):

        a = stats[i, cv2.CC_STAT_AREA]
        t = stats[i, cv2.CC_STAT_TOP]
        l = stats[i, cv2.CC_STAT_LEFT]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]

        # print(a)
        if (a >= 20):

            filtered[np.where(labels == i)] = 255
            linear_size = math.sqrt(a)

            distance_by_cam = round(calibration_distance * calibration_linear_size / linear_size)
            cv2.putText(frame, str(a), (l, t), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.rectangle(frame, (l, t), (l + w, t + h), (0, 255, 0), 2)
            if len(m) > 1: m.pop(0)
            m.append({"l": l, "t": t, "w": w, "h": h, "size": a})
    if len(m) == 0 or len(m) == 1 or len(m)>2:
        if time.time() - t12 > 0.5:
            ser.write(f"{1} {1}\n".encode())
            t12 = time.time()
    if len(m) == 2:
        l1 = m[0]["l"] + round(m[0]["w"] / 2)
        t1 = m[0]["t"] + round(m[0]["h"] / 2)
        l2 = m[1]["l"] + round(m[1]["w"] / 2)
        t2 = m[1]["t"] + round(m[1]["h"] / 2)
        if m[0]["l"] > m[1]["l"]:
            l1, l2 = l2, l1
            t1, t2 = t2, t1
        cv2.line(frame, (l1, t1), (l2, t2), (0, 0, 0), 2)
        cv2.line(frame, (l1, 0), (l1, t1), (0, 0, 0), 2)
        if t1 == t2:
            u = 90
        elif t1 > t2:
            u = math.atan((l2 - l1) / (t1 - t2)) * 180 / 3.14
        else:
            u = 180 - math.atan((l2 - l1) / (t2 - t1)) * 180 / 3.14

        size = m[0]["size"]
        if m[0]["size"] > m[1]["size"]:
            size = m[1]["size"]
        if m[0]["size"] > 200 and m[1]["size"] > 200:
            speed = (70 / 500) * (size - 200)
        else:
            speed = (-(70 / 200) * (200 - size))
        lwheel = speed
        rwheel = speed
        if u > 90:
            rwheel -= ((1.5 * speed / 3) / 180) * u
        elif u < 90:
            lwheel -= ((1.5 * speed / 3) / 90) * (90 - u)

        if (time.time()) - t122 > 0.4:
            if abs((lwheel - lwheel1) > 5 or abs(rwheel - rwheel1) > 5):
                print(round(speed), size, round(lwheel), round(rwheel))
                rwheel1 = rwheel
                lwheel1 = lwheel
                if (not rwheel == 0 and not lwheel == 0): ser.write(f"{round(lwheel)} {round(rwheel)}\n".encode())
            t122 = time.time()

        cv2.putText(frame, str(round(u)) + "deg", (l1 + 40, t1 - 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
                    cv2.LINE_AA)

    cv2.imshow("frame", cv2.resize(frame, (0, 0), fx=3, fy=3))

    cv2.imshow("filtered", filtered)

    key = cv2.waitKey(10) & 0xFF

    if (key == ord(' ')):
        break

cam.release()
cv2.destroyAllWindows()
cv2.waitKey(10)
ser.close()
