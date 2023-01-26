# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gcs.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
import os
import signal
import psutil
import time
class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(428, 376)
        MainWindow.setMinimumSize(QtCore.QSize(428, 376))
        MainWindow.setMaximumSize(QtCore.QSize(428, 376))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(40, 10, 371, 71))
        font = QtGui.QFont()
        font.setFamily("Purisa")
        font.setPointSize(16)
        font.setBold(True)
        font.setItalic(True)
        font.setUnderline(False)
        font.setWeight(75)
        font.setStrikeOut(False)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.takeoffButton = QtWidgets.QPushButton(self.centralwidget)
        self.takeoffButton.setGeometry(QtCore.QRect(140, 70, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Purisa")
        self.takeoffButton.setFont(font)
        self.takeoffButton.setObjectName("takeoffButton")
        self.startMissionButton = QtWidgets.QPushButton(self.centralwidget)
        self.startMissionButton.setGeometry(QtCore.QRect(140, 120, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Purisa")
        self.startMissionButton.setFont(font)
        self.startMissionButton.setObjectName("startMissionButton")
        self.stopObservationButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopObservationButton.setGeometry(QtCore.QRect(140, 170, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Purisa")
        self.stopObservationButton.setFont(font)
        self.stopObservationButton.setObjectName("stopObservationButton")
        self.landDrones = QtWidgets.QPushButton(self.centralwidget)
        self.landDrones.setGeometry(QtCore.QRect(140, 270, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Purisa")
        self.landDrones.setFont(font)
        self.landDrones.setObjectName("landDrones")
        self.returnToLaunchButton = QtWidgets.QPushButton(self.centralwidget)
        self.returnToLaunchButton.setGeometry(QtCore.QRect(140, 220, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Purisa")
        self.returnToLaunchButton.setFont(font)
        self.returnToLaunchButton.setObjectName("returnToLaunchButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 428, 20))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.startMissionButton.clicked.connect(self.startActivated)
        self.takeoffButton.clicked.connect(self.takeoffActivated)
        self.stopObservationButton.clicked.connect(self.stopObservationActivated)
        self.landDrones.clicked.connect(self.landActivated)
        self.returnToLaunchButton.clicked.connect(self.returnToLaunchActivated)
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "GROUND CONTROL STATION"))
        self.label.setText(_translate("MainWindow", "GROUND CONTROL STATION"))
        self.takeoffButton.setText(_translate("MainWindow", "TAKEOFF"))
        self.startMissionButton.setText(_translate("MainWindow", "START MISSION"))
        self.stopObservationButton.setText(_translate("MainWindow", "STOP OBSERVATION"))
        self.landDrones.setText(_translate("MainWindow", "LAND DRONES"))
        self.returnToLaunchButton.setText(_translate("MainWindow", "RETURN TO LAUNCH"))

    def get_process_id(self, name):
        for proc in psutil.process_iter():
            try:
                commandList = proc.cmdline()
                if len(commandList) < 4:
                    continue
                if commandList[1] == "main.py" and commandList[2] == "--droneType" and  commandList[3] == name:
                    return proc.pid
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return None

    def connectProcesses(self):
        self.observerPID = self.get_process_id("observer")
        self.targetPID = self.get_process_id("target")

    def startActivated(self):
        os.kill(self.targetPID, signal.SIGUSR1+6)
        os.kill(self.observerPID, signal.SIGUSR1+6)
        self.startMissionButton.setStyleSheet("background-color : red")
        self.startMissionButton.setDisabled(True)
        print("Mission Started!")
        
        
    def takeoffActivated(self):
        os.kill(self.observerPID, signal.SIGUSR1+5)
        os.kill(self.targetPID, signal.SIGUSR1+5)
        self.takeoffButton.setDisabled(True)
        self.takeoffButton.setStyleSheet("background-color : red")
        print("Takeoff started!")
    
    def stopObservationActivated(self):
        os.kill(self.observerPID, signal.SIGUSR1+7)
        print("Observation Stopped!")
        self.startMissionButton.setStyleSheet("background-color : white")
        self.startMissionButton.setDisabled(False)

    def returnToLaunchActivated(self):
        os.kill(self.targetPID, signal.SIGUSR1+7)
        print("Return to launch started!")

    def landActivated(self):
        os.kill(self.observerPID, signal.SIGUSR1+8)
        os.kill(self.targetPID, signal.SIGUSR1+4)
        print("Landing Started!")