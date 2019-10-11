# -*- coding: utf-8 -*-
import time
from threading import Thread

import numpy as np
from PyQt5.QtWidgets import *
from mbientlab.warble import *
from pyqtgraph.Qt import QtCore
from scipy.signal import find_peaks

import bplib
import dialog


class mainDialog(QDialog, dialog.Ui_Dialog):
    devices = {}
    dataRate = 90
    second = 3

    previous_front = 0
    previous_back = dataRate * second
    diff = 0
    num = dataRate * second

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        device_found = False
        while not device_found:
            try:
                self.device_mac = bplib.searchDevice()[0]
                device_found = True
            except IndexError:
                pass

        self.connectDevice()
        # Disconnect the device which is communicating with PC
        self.pushButton_2.clicked.connect(self.disconnectDevice)
        # Start recording
        self.pushButton_3.clicked.connect(self.startRecording)
        # Stop recording
        self.pushButton_4.clicked.connect(self.stopRecording)
        # How many seconds we display the signal
        self.spinBox.valueChanged.connect(self.displaySecond)

        self.p2 = self.win.addPlot(title="Blood Pressure")
        self.p2.setLabel('left', text='Blood Pressure', units='pF')
        self.p2.setLabel('bottom', text='Time')
        self.p2.enableAutoRange(axis=None, enable=True)
        self.curve1 = self.p2.plot()
        self.curve2 = self.p2.plot()
        self.curve3 = self.p2.plot()


    def buttonStatus(self, value):
        Selection = []
        for i in range(4):
            Selection.append(getattr(self, "pushButton_%d" % (i + 1)))
            Selection[i].setEnabled(value)

    def connectDevice(self):
        connected = False
        while not connected:
            try:
                address = self.device_mac
                device = bplib.connectDevice(address)
                self.mmr = bplib.State(device)
                bplib.turnonLED(self.mmr)
                bplib.setDevice(self.mmr)
                bplib.createTimer(self.mmr)
                connected = True
            except WarbleException as e:
                print(f"Connection failed... retrying")

    def disconnectDevice(self):
        try:
            bplib.disconnectDevice(self.mmr.device)
            self.mmr = None
            self.textBrowser.setText("Disconnected")
        except AttributeError as e:
            self.textBrowser.setText("First, Connect a MMR")

    def startRecording(self):
        def update():
            try:
                if np.shape(self.mmr.i2c)[0] <= self.num:
                    print(np.shape(self.mmr.i2c)[0])
                    self.curve1.setData(self.mmr.i2c[0:, 0], self.mmr.i2c[0:, 1])
                    # self.curve.setData(self.mmr.timeStamp[0:],self.mmr.i2c[0:,1])
                    signal_data = self.mmr.i2c[0:, 1]
                    peaks, _ = find_peaks(signal_data, distance=50)
                    troughs, _ = find_peaks(-signal_data, distance=50)
                    self.curve2.setData(self.mmr.i2c[peaks, 0], self.mmr.i2c[peaks, 1])
                    self.curve3.setData(self.mmr.i2c[troughs, 0], self.mmr.i2c[troughs, 1])
                else:
                    current = np.shape(self.mmr.i2c)[0]
                    self.diff = current - self.previous_back
                    self.previous_front += self.diff
                    self.previous_back += self.diff
                    signal_data = self.mmr.i2c[self.previous_front:self.previous_back, 1]

                    peaks, _ = find_peaks(signal_data, distance=50)
                    troughs, _ = find_peaks(-signal_data, distance=50)
                    print(peaks)
                    self.curve1.setData(self.mmr.i2c[self.previous_front:self.previous_back, 0], \
                                        self.mmr.i2c[self.previous_front:self.previous_back, 1])
                    self.curve2.setData(self.mmr.i2c[peaks+self.previous_front, 0], self.mmr.i2c[peaks+self.previous_front, 1])
                    self.curve3.setData(self.mmr.i2c[troughs+self.previous_front, 0], self.mmr.i2c[troughs+self.previous_front, 1])
                    # self.curve.setData(self.mmr.timeStamp[self.previous_front:self.previous_back], self.mmr.i2c[self.previous_front:self.previous_back:,1])

            except IndexError as e:
                pass
            except AttributeError as e:
                self.textBrowser.setText("First, Connect a MMR")
                pass

        def recordData():
            bplib.startRecording(self.mmr)

        th2 = Thread(target=recordData)
        th2.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(update)
        self.timer.start(11)

    def stopRecording(self):
        try:
            self.timer.stop()
            bplib.stopRecording(self.mmr)
            time.sleep(0.5)
            print(self.mmr.i2c)
            print(np.shape(self.mmr.i2c))
            self.textBrowser.setText("Writing data....")
            self.writeData()
            self.mmr.i2c = None

            self.previous_front = 0
            self.previous_back = self.dataRate * self.second
            self.diff = 0
            self.num = self.dataRate * self.second

            self.textBrowser.setText("Done")
        except AttributeError as e:
            self.textBrowser.setText("First, Connect a MMR")
            pass

    def writeData(self):
        bplib.writeData(self.mmr)

    def displaySecond(self):
        second = self.spinBox.value()
        self.second = second
        self.previous_back = self.dataRate * self.second
        diff = 0
        self.num = self.dataRate * self.second


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindow = mainDialog()
    myWindow.show()

    app.exec_()
