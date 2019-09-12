
import cv2
import numpy as np
import sys
import urllib.request
import math
from time import sleep
#import RPi.GPIO as GPIO

#IpCamUrl = 'http://91.93.0.33:60001/'

""
def findContour(frame):
    areas = 0
    contourIndex = 0
    #image, contours, hierarchy = cv2.findContours(frame, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    image, contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > areas):
            areas = area
            contourIndex = idx

    locationss = (0, 0, 2, 2)
    if len(contours) > 0:
        locationss = cv2.boundingRect(contours[contourIndex])

    return locationss, areas

def nothing(x):
    pass


def SaveTemplate():
    cv2.imwrite("template.jpg", CropFrame)
    print("Template Kaydedildi")


cv2.namedWindow('Frame')

# outputPin = 7
#GPIO.setwarnings(False)
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(outputPin, GPIO.OUT)

HueMin = SaturationMin = ValueMin = 0
HueMax = 179
SaturationMax = ValueMax = 255

# cv2.namedWindow('Renk Ayar Penceresi')
# cv2.createTrackbar('H-Min', 'Renk Ayar Penceresi', 0, 179, nothing)
# cv2.createTrackbar('S-Min', 'Renk Ayar Penceresi', 0, 255, nothing)
# cv2.createTrackbar('V-Min', 'Renk Ayar Penceresi', 0, 255, nothing)
# cv2.setTrackbarPos('H-Min', 'Renk Ayar Penceresi', HueMin)
# cv2.setTrackbarPos('S-Min', 'Renk Ayar Penceresi', SaturationMin)
# cv2.setTrackbarPos('V-Min', 'Renk Ayar Penceresi', ValueMin)
# ColorOptionFrame = np.zeros((1, 500, 3), np.uint8)

ErodeMatrix = np.ones((3, 3), np.uint8)
DilateMatrix = np.ones((5, 5), np.uint8)

# print("Cpu: ", cv2.getNumberOfCPUs())

Cam = cv2.VideoCapture(0)
CamWidth = int(Cam.get(3))
CamHeight = int(Cam.get(4))
CamFps = Cam.get(5)

BorderSize = 100
TempFrame = 0
i = 0
OldPt = BorderSize, BorderSize

if Cam.isOpened() == True:
    print('\n\nKamera Açıldı')
    print("--Özellikler--\nWidth: ", CamWidth, "\nHeight: ", CamHeight, "\nFPS: ", CamFps, "\n")
else:
    print('HATA!! \nKamera Açılamadı!!')
    exit(1)

while True:
    try:

        # ImgResp = urllib.request.urlopen(IpCamUrl)
        # ImgNp = np.array(bytearray(ImgResp.read()), dtype=np.uint8)
        # Frame = cv2.imdecode(ImgNp, -1)

        Ret, Frame = Cam.read()

        if Ret != True:
            print('HATA!! Kameradan Frame Alınamıyor \nUygulamayı Yeniden Başlatın')
            cv2.destroyAllWindows()
            Cam.release()
            break
            exit(1)

        cv2.putText(Frame, "Kayit icin ESC Basin", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.rectangle(Frame, (BorderSize, BorderSize), (CamWidth - 315, CamHeight - 235), (255, 0, 0), 2)
        GrayFrame = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
        CropFrame = GrayFrame[BorderSize:CamHeight - 235, BorderSize:CamWidth - 315]
        cv2.imshow('Frame', Frame)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            SaveTemplate()
            print("Tarama Yapılıyor...")
            # GPIO.cleanup()
            break
    except:
        print("Beklenmedik Hata!!! ", sys.exc_info()[0])
        raise


#
# while True:
#
#     # cv2.imshow('Renk Ayar Penceresi', ColorOptionFrame)
#
#     try:
#
#         # ImgResp = urllib.request.urlopen(IpCamUrl)
#         # ImgNp = np.array(bytearray(ImgResp.read()), dtype=np.uint8)
#         # Frame = cv2.imdecode(ImgNp, -1)
#
#         Ret, Frame = Cam.read()
#
#         if Ret != True:
#             print('HATA!! Kameradan Frame Alınamıyor \nUygulamayı Yeniden Başlatın')
#             cv2.destroyAllWindows()
#             Cam.release()
#             break
#             exit(1)
#
#         # HueMin = cv2.getTrackbarPos('H-Min', 'Renk Ayar Penceresi')
#         # SaturationMin = cv2.getTrackbarPos('S-Min', 'Renk Ayar Penceresi')
#         # ValueMin = cv2.getTrackbarPos('V-Min', 'Renk Ayar Penceresi')
#         # LowerLimit = np.array([HueMin, SaturationMin, ValueMin], dtype=np.uint8)
#         # UpperLimit = np.array([HueMax, SaturationMax, ValueMax], dtype=np.uint8)
#
#
#         # frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
#         # reverseFrame = 255 - frame
#         # FrameHsv = cv2.cvtColor(Frame, cv2.COLOR_BGR2HSV)
#
#         # cv2.rectangle(Frame, (BorderSize, BorderSize), (CamWidth - BorderSize, CamHeight-BorderSize), (255, 0, 0), 2)
#         i = i + 1
#
#
#         GrayFrame = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
#
#         template = cv2.imread('template.jpg', 0)
#         w, h = template.shape[::-1]
#         res = cv2.matchTemplate(GrayFrame, template, cv2.TM_CCOEFF_NORMED)
#         threshold = 0.60
#         loc = np.where(res >= threshold)
#         # print(loc)
#         for pt in zip(*loc[::-1]):
#             cv2.rectangle(Frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2)
#             # GPIO.output(gpioPin, GPIO.HIGH)
#             print(pt)
#             cv2.putText(Frame, "Template Algilandi", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
#
#         cv2.imshow('Frame', Frame)
#
#         k = cv2.waitKey(5) & 0xFF
#         if k == 27:
#             print("Çıkış Yapıldı")
#             # GPIO.cleanup()
#             break
#     except:
#         print("Beklenmedik Hata!!! ", sys.exc_info()[0])
#         raise




while True:

    # cv2.imshow('Renk Ayar Penceresi', ColorOptionFrame)

    try:
        Ret, Frame = Cam.read()

        if Ret != True:
            print('HATA!! Kameradan Frame Alınamıyor \nUygulamayı Yeniden Başlatın')
            cv2.destroyAllWindows()
            Cam.release()
            break
            exit(1)

        # HueMin = cv2.getTrackbarPos('H-Min', 'Renk Ayar Penceresi')
        # SaturationMin = cv2.getTrackbarPos('S-Min', 'Renk Ayar Penceresi')
        # ValueMin = cv2.getTrackbarPos('V-Min', 'Renk Ayar Penceresi')
        # LowerLimit = np.array([HueMin, SaturationMin, ValueMin], dtype=np.uint8)
        # UpperLimit = np.array([HueMax, SaturationMax, ValueMax], dtype=np.uint8)


        # frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
        # reverseFrame = 255 - frame
        # FrameHsv = cv2.cvtColor(Frame, cv2.COLOR_BGR2HSV)

        # cv2.rectangle(Frame, (BorderSize, BorderSize), (CamWidth - BorderSize, CamHeight-BorderSize), (255, 0, 0), 2)
        i = i + 1


        GrayFrame = cv2.cvtColor(Frame, cv2.COLOR_BGR2GRAY)
        # CropFrame = GrayFrame[BorderSize:CamHeight - BorderSize, BorderSize:CamWidth - BorderSize]
        CropFrame = GrayFrame[BorderSize:CamHeight - 235, BorderSize:CamWidth - 315]
        TempFrame = CropFrame

        if not(i.__eq__(0)):
            w, h = TempFrame.shape[::-1]
            # w = TempFrame.shape[1]
            # h = TempFrame.shape[0]
            res = cv2.matchTemplate(GrayFrame, TempFrame, cv2.TM_CCOEFF_NORMED)
            threshold = 0.7
            loc = np.where(res >= threshold)
            for pt in zip(*loc[::-1]):
                # cv2.putText(Frame, "pt[0]: {}".format(pt[0]), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                # cv2.putText(Frame, "pt[1]: {}".format(pt[1]), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                print(pt, OldPt)
                # if (pt[0] < OldPt[0]):
                #     cv2.putText(Frame, "X Hareket: Sol", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
                # else:
                #     cv2.putText(Frame, "X Hareket: Sag", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
                #
                # if (pt[1] < OldPt[1]):
                #     cv2.putText(Frame, "Y Hareket: Yukari", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
                # else:
                #     cv2.putText(Frame, "Y Hareket: Asagi", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)

                OldPt = pt
                cv2.rectangle(Frame, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2)
        sleep(1)

        cv2.imshow('Frame', Frame)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            print("Çıkış Yapıldı")
            # GPIO.cleanup()
            break
    except:
        print("Beklenmedik Hata!!! ", sys.exc_info()[0])
        raise

cv2.destroyAllWindows()
Cam.release()