"""
Created on Sat Jan 9 19:54:38 2019

@author: omerfarukkoc
"""
import numpy as np
import cv2
import dlib
import math
import sys
# import serial

MousePoints = []

def mouseEventHandler(event, x, y, flags, param):
    global MousePoints
    if event == cv2.EVENT_LBUTTONDOWN:
        MousePoints = [(x, y)]
    elif event == cv2.EVENT_LBUTTONUP:
        MousePoints.append((x, y))

cv2.namedWindow("Frame")
cv2.setMouseCallback("Frame", mouseEventHandler)

IpCamURL = 'rtsp://admin:admin@192.168.1.168:554/0'
Cam = cv2.VideoCapture(0)

# Cam = cv2.VideoCapture(0)
# Cam.set(3, 2464)
# Cam.set(4, 2056)

ResizeWidth = 1024
ResizeHeight = 768

# CamWidth = int(Cam.get(3))
# CamHeight = int(Cam.get(4))
# CamFps = Cam.get(5)
if Cam.isOpened() == True:
    print('\nKamera Açıldı')
    # print("-Özellikler-\nW: ", CamWidth, "\nH: ", CamHeight, "\nFPS: ", CamFps, "\n")
else:
    print('\nHATA!! \nKamera Açılamadı!!')
    exit(1)

Tracker = dlib.correlation_tracker()
TrackerControl = False
PointControl = False
Count = 0
LocationSensitivity = 5

XSerialString = "-"
YSerialString = "-"
ZSerialString = "-"

# ser = serial.Serial(
#         port='/dev/ttyAMA0',
#         baudrate = 9600,
#         parity=serial.PARITY_NONE,
#         stopbits=serial.STOPBITS_ONE,
#         bytesize=serial.EIGHTBITS,
#         timeout=1
# )

while True:
    Ret, Frame = Cam.read()
    Frame = cv2.resize(Frame, (ResizeWidth, ResizeHeight), interpolation=cv2.INTER_LINEAR)
    try:
        if Ret != True:
            print('\nHATA!! Kameradan Frame Alınamıyor \nUygulamayı Yeniden Başlatın')
            cv2.destroyAllWindows()
            Cam.release()
            break
            exit(1)

        if len(MousePoints) == 2:
            # cv2.rectangle(Frame, MousePoints[0], MousePoints[1], (0, 0, 255), 2)
            TrackedRectangle = dlib.rectangle(MousePoints[0][0], MousePoints[0][1], MousePoints[1][0], MousePoints[1][1])
            Tracker.start_track(Frame, TrackedRectangle)
            TrackerControl = True
            MousePoints = []

        if TrackerControl == True:
            Tracker.update(Frame)
            track_rect = Tracker.get_position()
            x1 = int(track_rect.left())
            y1 = int(track_rect.top())
            x2 = int(track_rect.right())
            y2 = int(track_rect.bottom())
            Points = ((x1 + int((x2-x1)/2)), (y1 + int((y2-y1)/2)), (x2-x1))
            cv2.circle(Frame, (Points[0], Points[1]), 5, (0, 0, 255), -1)
            cv2.rectangle(Frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

            Count += 1
            if (Count % 5) == 0:
                OldPoints = Points
                PointControl = True

            if PointControl:
                if (Points[0] - OldPoints[0]) > LocationSensitivity:
                    cv2.putText(Frame, "X Hareket: Sag", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # XSerialString = "Sag"
                elif (OldPoints[0] - Points[0]) > LocationSensitivity:
                    cv2.putText(Frame, "X Hareket: Sol", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # XSerialString = "Sol"
                else:
                    cv2.putText(Frame, "X Hareket: -", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # XSerialString = "-"
                    
                if (Points[1] - OldPoints[1]) > LocationSensitivity:
                    cv2.putText(Frame, "Y Hareket: Asagi", (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # YSerialString = "Asagi"
                elif (OldPoints[1] - Points[1]) > LocationSensitivity:
                    cv2.putText(Frame, "Y Hareket: Yukari", (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # YSerialString = "Yukari"
                else:
                    cv2.putText(Frame, "Y Hareket: -", (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # YSerialString = "-"

                if (Points[2] - OldPoints[2]) > LocationSensitivity:
                    cv2.putText(Frame, "Z Hareket: Ileri", (5, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # ZSerialString = "Ileri"
                elif (OldPoints[2] - Points[2]) > LocationSensitivity:
                    cv2.putText(Frame, "Z Hareket: Geri", (5, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # ZSerialString = "Geri"
                else:
                    cv2.putText(Frame, "Z Hareket: -", (5, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                                cv2.LINE_AA)
                    # ZSerialString = "-"
                #print("X: "+ XSerialString + " Y: "+ YSerialString + " Z: "+ ZSerialString + "\n")
                # ser.write("X: "+ XSerialString + " Y: "+ YSerialString + " Z: "+ ZSerialString + "\n")

        cv2.putText(Frame, "Takip icin Mouse ile 2 nokta arasi secim yapiniz", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(Frame, "R: Yenile", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(Frame, "ESC: Cikis", (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Frame", Frame)
        k = cv2.waitKey(5) & 0xFF
        if k == ord("r"):
            MousePoints = []
            TrackerControl = False
        if k == 27:
            print("Çıkış Yapıldı")
            Cam.release()
            cv2.destroyAllWindows()
            break
    except:
        print("\nBeklenmedik Hata!!! Hata Kodu İle Üreticinize Başvurun\n", sys.exc_info()[0])
        raise

Cam.release()
cv2.destroyAllWindows()
