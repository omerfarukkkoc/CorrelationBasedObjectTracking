"""
Created on Sat Jan 9 19:54:38 2019

@author: omerfarukkoc
"""

import cv2
import dlib
import math
import sys
import serial
import RPi.GPIO as GPIO
GPIO.setwarnings(False)


TrigInputPin = 11
ResetInputPin = 13
CloseInputPin = 15
GPIO.setmode(GPIO.BOARD)
GPIO.setup(TrigInputPin, GPIO.IN, pull_up_down= GPIO.PUD_DOWN)
GPIO.setup(ResetInputPin, GPIO.IN, pull_up_down= GPIO.PUD_DOWN)
GPIO.setup(CloseInputPin, GPIO.IN, pull_up_down= GPIO.PUD_DOWN)

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
# Cam = cv2.VideoCapture(IpCamURL)

Tracker = dlib.correlation_tracker()

CamWidth = int(Cam.get(3))
CamHeight = int(Cam.get(4))
CamFps = Cam.get(5)
if Cam.isOpened() == True:
    print('\nKamera Açıldı')
    print("-Özellikler-\nW: ", CamWidth, "\nH: ", CamHeight, "\nFPS: ", CamFps, "\n")
else:
    print('\nHATA!! \nKamera Açılamadı!!')
    exit(1)


ResizeWidth = 1024
ResizeHeight = 768

TrackingAreaVariable = 100

TrackerControl = False
PointControl = False
TriggerControl = False
Count = 0
LocationSensitivity = 5

XSerialString = "-"
YSerialString = "-"
ZSerialString = "-"

x = 0
y = 0
z = 0

ser = serial.Serial(
        port='/dev/ttyAMA0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)

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

        k = cv2.waitKey(5) & 0xFF

        if TriggerControl != True:
            RectPoint1 = int(ResizeWidth / 2)
            RectPoint2 = int(ResizeHeight / 2)
            cv2.rectangle(Frame, ((RectPoint1 - TrackingAreaVariable), (RectPoint2 - TrackingAreaVariable)), ((RectPoint1 + TrackingAreaVariable), (RectPoint2 + TrackingAreaVariable)), (0, 0, 255), 2)


        # if len(MousePoints) == 2:
        # if k == ord("t"):
        if GPIO.input(TrigInputPin) == False:
            TrackedRectangle = dlib.rectangle(RectPoint1 - TrackingAreaVariable, RectPoint2 - TrackingAreaVariable, RectPoint1 + TrackingAreaVariable, RectPoint2 + TrackingAreaVariable)
            Tracker.start_track(Frame, TrackedRectangle)
            TrackerControl = True
            TriggerControl = True
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

            # Serial Send Package start
            x = x1
            y = y1
            z = int(math.sqrt(((x2-x1)**2) + ((y2-y1)**2)))
            cv2.putText(Frame, "X Lokasyon: "+str(x), (200, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(Frame, "Y Lokasyon: " + str(y), (200, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)
            cv2.putText(Frame, "Diyagonal(Z): " + str(z), (200, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA)

            s = x & 0xFF
            CRC = s
            # ser.write(s)
            s = (x >> 8) & 0xFF
            CRC += s
            # ser.write(s)

            s = y & 0xFF
            CRC += s
            # ser.write(s)
            s = (y >> 8) & 0xFF
            CRC += s
            # ser.write(s)

            s = z & 0xFF
            CRC += s
            # ser.write(s)
            s = (z >> 8) & 0xFF
            CRC += s
            # ser.write(s)
            if (CRC > 255): CRC -= 256
            # ser.write(CRC)

            # Serial Send Package stop


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

        # cv2.putText(Frame, "T: Takip", (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        # cv2.putText(Frame, "R: Yenile", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        # cv2.putText(Frame, "ESC: Cikis", (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Frame", Frame)


        if GPIO.input(ResetInputPin) == False:
        # if k == ord("r"):
            MousePoints = []
            TrackerControl = False
            TriggerControl = False
        if GPIO.input(CloseInputPin) == False:
        # if k == 27:
            print("Çıkış Yapıldı")
            Cam.release()
            cv2.destroyAllWindows()
            # GPIO.cleanup()
            break
    except:
        print("\nBeklenmedik Hata!!! Hata Kodu İle Üreticinize Başvurun\n", sys.exc_info()[0])
        raise

Cam.release()
cv2.destroyAllWindows()
GPIO.cleanup()
