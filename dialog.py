# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'dialog.ui'
#
# Created by: PyQt5 UI code generator 5.13.0
#
# WARNING! All changes made in this file will be lost!


import pyqtgraph as pg
from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(1050, 504)  # 945
        self.horizontalLayoutWidget = QtWidgets.QWidget(Dialog)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(9, 10, 691, 461))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(11, 11, 11, 11)
        self.horizontalLayout.setSpacing(6)
        self.horizontalLayout.setObjectName("horizontalLayout")

        self.winWidget = QtWidgets.QWidget(self.horizontalLayoutWidget)
        self.winWidget.resize(691, 461)
        self.win = pg.GraphicsLayoutWidget(self.winWidget)
        self.win.resize(691, 461)

        """d = np.random.normal(size=100)
        d[50:54] += 30
        p1 = self.win.addPlot(title="95th percentile range", y=d)
        p1.enableAutoRange('y', 0.95)"""

        self.listWidget = QtWidgets.QListWidget(Dialog)
        self.listWidget.setGeometry(QtCore.QRect(720, 60, 171, 301))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setItalic(True)
        self.listWidget.setFont(font)
        self.listWidget.setObjectName("listWidget")

        self.gridLayoutWidget = QtWidgets.QWidget(Dialog)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(720, 370, 171, 119))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(11, 11, 11, 11)
        self.gridLayout.setSpacing(6)
        self.gridLayout.setObjectName("gridLayout")

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setItalic(True)

        self.pushButton_2 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.gridLayout.addWidget(self.pushButton_2, 2, 0, 1, 1)
        self.pushButton_2.setFont(font)

        self.pushButton_3 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.gridLayout.addWidget(self.pushButton_3, 3, 0, 1, 1)
        self.pushButton_3.setFont(font)

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setItalic(True)
        self.pushButton_4 = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.pushButton_4.setObjectName("pushButton_4")
        self.gridLayout.addWidget(self.pushButton_4, 4, 0, 1, 1)
        self.pushButton_4.setFont(font)

        font = QtGui.QFont()
        font.setPointSize(16)
        font.setItalic(True)
        self.textBrowser = QtWidgets.QTextBrowser(Dialog)
        self.textBrowser.setGeometry(QtCore.QRect(720, 10, 321, 41))
        self.textBrowser.setFont(font)
        self.textBrowser.setObjectName("textBrowser")

        self.spinBox = QtWidgets.QSpinBox(Dialog)
        self.spinBox.setGeometry(QtCore.QRect(910, 90, 42, 22))
        self.spinBox.setMinimum(1)
        self.spinBox.setMaximum(30)
        self.spinBox.setValue(3)
        self.spinBox.setObjectName("spinBox")

        self.label = QtWidgets.QLabel(Dialog)
        self.label.setGeometry(QtCore.QRect(910, 60, 131, 21))
        self.label.setFont(font)
        self.label.setObjectName("label")

        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.pushButton_2.setText(_translate("Dialog", "Disconnection"))
        self.pushButton_3.setText(_translate("Dialog", "Start Recording"))
        self.pushButton_4.setText(_translate("Dialog", "Stop Recording"))
        self.label.setText(_translate("Dialog", "Display Time (Unit : s)"))
