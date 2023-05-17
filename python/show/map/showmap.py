from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import sys
import matplotlib.pyplot as plt
from pathlib import Path
import struct

lineNum = 0
x = []
y = []
nWidth = 250
nHeight = 150

class PlayMap(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setGeometry(0,0, nWidth*4, nHeight*4)
        image = QImage(nWidth, nHeight, QImage.Format_RGB32)

        MV_NO_FREESPACE = 0
        MV_FREESPACE = 1
        MV_LIMIT_SPEED = 2
        MV_GROUNDPIN = 3 #档栏杆

        MV_LANEMARK = 4 #车道线
        MV_PARKINGLINE = 5#车位线

        MV_CURB = 10 #路沿
        MV_OBSTACLE = 11 #一般障碍物
        MV_PEOPLE = 12 #行人
        MV_NOMOTOR = 13 #非机动车二轮车
        MV_MOTOR = 14 #机动车
        MV_PILLAR  = 15 #柱子
        MV_GROUNDLOCK = 16 #地锁开，包括一版障碍物 警示锥

        MV_MOTOR_WHEELS = 17 #车轮分割
        NAtype  = 255  

        NO_FREE= 0x3F
        VEL_COLOR=122
        UNKNOW=100
        SLOT_ARER=32
        MV_NOMOTOR_SAVE=53

        # map[135+(nHeight - 95)*nWidth] = MV_CURB
        for i in range(nWidth):
            for j in range(nHeight):
                if MV_NOMOTOR == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(255,255,0))#yellow
                if MV_NOMOTOR_SAVE == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(255,255,0))#yellow
                elif MV_GROUNDPIN == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(255,0,255))#purple
                elif MV_OBSTACLE == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(0,255,255))#light blue
                elif MV_MOTOR == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(255,0,0))#red
                elif MV_MOTOR_WHEELS == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(255,0,0))#red
                elif MV_PEOPLE == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(128,42,42))#brown
                elif 32 == map[j*nWidth+ i] :#slot
                    image.setPixel(i,j,qRgb(0,255,0))#green
                elif VEL_COLOR == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(0,255,0))#green
                elif MV_PILLAR == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(0,0,255))#blue
                elif MV_CURB == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(0,0,0))#black
                elif UNKNOW == map[j*nWidth+ i] :
                    image.setPixel(i,j,qRgb(188, 143,143 ))#RosyBrown
                elif NO_FREE == map[j*nWidth+ i] :
                #            elif VEL_COLOR:
                    image.setPixel(i,j,qRgb(255-NO_FREE,255-NO_FREE,255-NO_FREE))
                else:
                    image.setPixel(i,j,qRgb(255,255,255))
        label = QLabel(self)
        label.setPixmap(QPixmap(image.scaled(nWidth*4, nHeight*4)))#

if __name__ == "__main__":
    argvstr = sys.argv
    if len(argvstr) == 2:
        with open(argvstr[1], 'rb') as f:
            map = f.read()
    elif len(argvstr) == 4:
        nWidth = int(argvstr[2])
        nHeight = int(argvstr[3])

        with open(argvstr[1], 'rb') as f:
            map = f.read()
    else:
        print('err input')

    app = QtWidgets.QApplication(sys.argv)
    window = PlayMap()
    window.show()
    window.setWindowTitle('plan map')
    sys.exit(app.exec_())