# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'login.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class login_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.setEnabled(True)
        MainWindow.resize(500, 400)
        MainWindow.setMinimumSize(QtCore.QSize(500, 400))
        MainWindow.setMaximumSize(QtCore.QSize(500, 400))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("icon/logo.jpg"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        MainWindow.setWindowIcon(icon)
        MainWindow.setStyleSheet("background-color:rgba(0,0,0,0);\n"
                                 "background-color: rgb(255, 255, 255);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(160, 40, 181, 111))
        font = QtGui.QFont()
        font.setFamily("幼圆")
        font.setPointSize(34)
        self.label.setFont(font)
        self.label.setMidLineWidth(0)
        self.label.setObjectName("label")
        self.edu_login = QtWidgets.QLineEdit(self.centralwidget)
        self.edu_login.setGeometry(QtCore.QRect(170, 170, 161, 31))
        self.edu_login.setStyleSheet("border:none;\n"
                                     "background-color: rgb(255, 255, 255);\n"
                                     "border-bottom:2px solid rgba(0,0,0,100);\n"
                                     "")
        self.edu_login.setObjectName("edu_login")
        self.edu_password = QtWidgets.QLineEdit(self.centralwidget)
        self.edu_password.setGeometry(QtCore.QRect(170, 230, 161, 31))
        self.edu_password.setStyleSheet("border:none;\n"
                                        "background-color: rgb(255, 255, 255);\n"
                                        "border-bottom:2px solid rgba(0,0,0,100);\n"
                                        "")
        self.edu_password.setEchoMode(QtWidgets.QLineEdit.Password)
        self.edu_password.setObjectName("edu_password")
        self.bt_login = QtWidgets.QPushButton(self.centralwidget)
        self.bt_login.setGeometry(QtCore.QRect(210, 280, 75, 23))
        self.bt_login.setStyleSheet("QPushButton{\n"
                                    "    background-color:qlineargradient(spread:pad,x1:0,y1:0,x2:1, y2:1, stop:0\n"
                                    "rgba(102,133,156,255), stop:1 rgba(117,255,201,255));\n"
                                    "    color:rgb(255,255,255);\n"
                                    "    border:none;\n"
                                    "    font: 10pt \"幼圆\";\n"
                                    "}\n"
                                    "QPushButton:pressed{\n"
                                    "    padding-left:5px;\n"
                                    "    padding-top:5px;    \n"
                                    "}    \n"
                                    "")
        self.bt_login.setObjectName("bt_login")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionxiugai = QtWidgets.QAction(MainWindow)
        self.actionxiugai.setObjectName("actionxiugai")
        self.actionjingt = QtWidgets.QAction(MainWindow)
        self.actionjingt.setObjectName("actionjingt")
        self.actionback = QtWidgets.QAction(MainWindow)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("icon/back.png"), QtGui.QIcon.Normal,
                        QtGui.QIcon.Off)
        self.actionback.setIcon(icon1)
        self.actionback.setObjectName("actionback")

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "智屏微翻肛鉴别系统"))
        self.label.setText(_translate("MainWindow", "登录系统"))
        self.edu_login.setPlaceholderText(_translate("MainWindow", "账号："))
        self.edu_password.setPlaceholderText(_translate("MainWindow", "密码："))
        self.bt_login.setText(_translate("MainWindow", "登录"))
        self.actionxiugai.setText(_translate("MainWindow", "镜头参数"))
        self.actionjingt.setText(_translate("MainWindow", "镜头大小"))
        self.actionback.setText(_translate("MainWindow", "退出"))
