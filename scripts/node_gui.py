#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String, Float32, Int16
from PyQt5 import QtCore, QtGui, QtWidgets
from robotic_sas_auv_ros.msg import Sensor, SetPoint
from robotic_sas_auv_ros.msg import Error
from robotic_sas_auv_ros.msg import Actuator
from simple_pid import PID

class Ui_Dialog(object):

    def __init__(self):
        #GUIDANCE
        self.depth = 0
        self.yaw = 0
        self.setpoint = SetPoint()
        self.setpoint.depth = 0
        self.setpoint.yaw = 0


        

        self.pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)

        rospy.Subscriber('/arduino/sensor', Sensor, self.callback_sensor)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)

        self.timer.start(100)

    def callback_sensor(self, data):
        self.depth = data.depth
        self.yaw = data.yaw


    def update(self):
        self.lcd_depth.display(self.depth)
        self.lcd_yaw.display(self.yaw)


    def setupUI(self,Dialog):
        Dialog.setObjectName("GUI_TECHSAS")
        Dialog.resize(349, 453)

        #LABEL DEPTH
        self.label_depth = QtWidgets.QLabel(Dialog)
        self.label_depth.setGeometry(QtCore.QRect(10, 10, 67, 17))
        self.label_depth.setObjectName("DEPTH")
        #Button DEPTH
        self.poss_depth = QtWidgets.QPushButton(Dialog)
        self.poss_depth.setGeometry(QtCore.QRect(230, 30, 101, 41))
        self.poss_depth.setObjectName("SEND DEPTH")
        #LCD DEPTH
        self.lcd_depth = QtWidgets.QLCDNumber(Dialog)
        self.lcd_depth.setGeometry(QtCore.QRect(10, 30, 101, 41))
        self.lcd_depth.setObjectName("LCD_YAW")
        #INPUT DEPTH
        self.depth_line = QtWidgets.QLineEdit(Dialog)
        self.depth_line.setGeometry(QtCore.QRect(120, 30, 101, 41))
        self.depth_line.setObjectName("LINE_DEPTH")

        #LABEL YAW
        self.label_yaw = QtWidgets.QLabel(Dialog)
        self.label_yaw.setGeometry(QtCore.QRect(10, 80, 67,17))
        self.label_yaw.setObjectName("YAW")
        #Button YAW
        self.poss_yaw = QtWidgets.QPushButton(Dialog)
        self.poss_yaw.setGeometry(QtCore.QRect(230, 100, 101,41))
        self.poss_yaw.setObjectName("SEND YAW")
        #LCD YAW
        self.lcd_yaw = QtWidgets.QLCDNumber(Dialog)
        self.lcd_yaw.setGeometry(QtCore.QRect(10, 100, 101, 41))
        self.lcd_yaw.setObjectName("LCD_DEPTH")
        #INPUT YAW
        self.yaw_line = QtWidgets.QLineEdit(Dialog)
        self.yaw_line.setGeometry(QtCore.QRect(120, 100, 101, 41))
        self.yaw_line.setObjectName("LINE_YAW")

        #LABEL ROLL
        self.label_roll = QtWidgets.QLabel(Dialog)
        self.label_roll.setGeometry(QtCore.QRect(10, 150, 71,17))
        self.label_roll.setObjectName("ROLL")
        #LABEL ROLL_P
        self.label_roll_p = QtWidgets.QLabel(Dialog)
        self.label_roll_p.setGeometry(QtCore.QRect(30, 170, 21,17))
        self.label_roll_p.setObjectName("LABEL_ROLL_P")
        #LABEL ROLL_I
        self.label_roll_i = QtWidgets.QLabel(Dialog)
        self.label_roll_i.setGeometry(QtCore.QRect(100, 170, 21,17))
        self.label_roll_i.setObjectName("LABEL_ROLL_I")
        #LABEL ROLL_D
        self.label_roll_d = QtWidgets.QLabel(Dialog)
        self.label_roll_d.setGeometry(QtCore.QRect(170, 170, 21,17))
        self.label_roll_d.setObjectName("LABEL_ROLL_D")
        #INPUT ROLL_P
        self.roll_p_line = QtWidgets.QLineEdit(Dialog)
        self.roll_p_line.setGeometry(QtCore.QRect(10, 190, 61, 21))
        self.roll_p_line.setObjectName("LINE_ROLL_P")
        #INPUT ROLL_I
        self.roll_i_line = QtWidgets.QLineEdit(Dialog)
        self.roll_i_line.setGeometry(QtCore.QRect(80, 190, 61, 21))
        self.roll_i_line.setObjectName("LINE_ROLL_I")
        #INPUT ROLL_D
        self.roll_d_line = QtWidgets.QLineEdit(Dialog)
        self.roll_d_line.setGeometry(QtCore.QRect(150, 190, 61, 21))
        self.roll_d_line.setObjectName("LINE_ROLL_D")
        #Button ROLL
        self.poss_roll = QtWidgets.QPushButton(Dialog)
        self.poss_roll.setGeometry(QtCore.QRect(230, 180, 101,41))
        self.poss_roll.setObjectName("SEND ROLL")

        #LABEL PITCH
        self.label_pitch = QtWidgets.QLabel(Dialog)
        self.label_pitch.setGeometry(QtCore.QRect(10, 220, 71,17))
        self.label_pitch.setObjectName("PITCH")
        #LABEL PITCH_P
        self.label_pitch_p = QtWidgets.QLabel(Dialog)
        self.label_pitch_p.setGeometry(QtCore.QRect(30, 240, 21,17))
        self.label_pitch_p.setObjectName("LABEL_PITCH_P")
        #LABEL PITCH_I
        self.label_pitch_i = QtWidgets.QLabel(Dialog)
        self.label_pitch_i.setGeometry(QtCore.QRect(100, 240, 21,17))
        self.label_pitch_i.setObjectName("LABEL_PITCH_I")
        #LABEL PITCH_D
        self.label_pitch_d = QtWidgets.QLabel(Dialog)
        self.label_pitch_d.setGeometry(QtCore.QRect(170, 240, 21,17))
        self.label_pitch_d.setObjectName("LABEL_PITCH_D")
        #INPUT PITCH_P
        self.pitch_p_line = QtWidgets.QLineEdit(Dialog)
        self.pitch_p_line.setGeometry(QtCore.QRect(10, 260, 61, 21))
        self.pitch_p_line.setObjectName("LINE_PITCH_P")
        #INPUT PITCH_I
        self.pitch_i_line = QtWidgets.QLineEdit(Dialog)
        self.pitch_i_line.setGeometry(QtCore.QRect(80, 260, 61, 21))
        self.pitch_i_line.setObjectName("LINE_PITCH_I")
        #INPUT PITCH_D
        self.pitch_d_line = QtWidgets.QLineEdit(Dialog)
        self.pitch_d_line.setGeometry(QtCore.QRect(150, 260, 61, 21))
        self.pitch_d_line.setObjectName("LINE_PITCH_D")
        #Button PITCH
        self.poss_pitch = QtWidgets.QPushButton(Dialog)
        self.poss_pitch.setGeometry(QtCore.QRect(230, 250, 101,41))
        self.poss_pitch.setObjectName("SEND PITCH")

        #LABEL YAW
        self.label_yaw2 = QtWidgets.QLabel(Dialog)
        self.label_yaw2.setGeometry(QtCore.QRect(10, 290, 71,17))
        self.label_yaw2.setObjectName("YAW")
        #LABEL YAW_P
        self.label_yaw2_p = QtWidgets.QLabel(Dialog)
        self.label_yaw2_p.setGeometry(QtCore.QRect(30, 310, 21,17))
        self.label_yaw2_p.setObjectName("LABEL_YAW_P")
        #LABEL YAW_I
        self.label_yaw2_i = QtWidgets.QLabel(Dialog)
        self.label_yaw2_i.setGeometry(QtCore.QRect(100, 310, 21,17))
        self.label_yaw2_i.setObjectName("LABEL_YAW_I")
        #LABEL YAW_D
        self.label_yaw2_d = QtWidgets.QLabel(Dialog)
        self.label_yaw2_d.setGeometry(QtCore.QRect(170, 310, 21,17))
        self.label_yaw2_d.setObjectName("LABEL_PITCH_D")
        #INPUT YAW_P
        self.yaw2_p_line = QtWidgets.QLineEdit(Dialog)
        self.yaw2_p_line.setGeometry(QtCore.QRect(10, 330, 61, 21))
        self.yaw2_p_line.setObjectName("LINE_YAW_P")
        #INPUT YAW_I
        self.yaw2_i_line = QtWidgets.QLineEdit(Dialog)
        self.yaw2_i_line.setGeometry(QtCore.QRect(80, 330, 61, 21))
        self.yaw2_i_line.setObjectName("LINE_YAW_I")
        #INPUT YAW_D
        self.yaw2_d_line = QtWidgets.QLineEdit(Dialog)
        self.yaw2_d_line.setGeometry(QtCore.QRect(150, 330, 61, 21))
        self.yaw2_d_line.setObjectName("LINE_YAW_D")
        #Button YAW
        self.poss_yaw2 = QtWidgets.QPushButton(Dialog)
        self.poss_yaw2.setGeometry(QtCore.QRect(230, 320, 101,41))
        self.poss_yaw2.setObjectName("SEND YAW")

        #LABEL HEAVE
        self.label_heave = QtWidgets.QLabel(Dialog)
        self.label_heave.setGeometry(QtCore.QRect(10, 360, 71,17))
        self.label_heave.setObjectName("HEAVE")
        #LABEL HEAVE_P
        self.label_heave_p = QtWidgets.QLabel(Dialog)
        self.label_heave_p.setGeometry(QtCore.QRect(30, 380, 21,17))
        self.label_heave_p.setObjectName("LABEL_HEAVE_P")
        #LABEL HEAVE_I
        self.label_heave_i = QtWidgets.QLabel(Dialog)
        self.label_heave_i.setGeometry(QtCore.QRect(100, 380, 21,17))
        self.label_heave_i.setObjectName("LABEL_HEAVE_I")
        #LABEL HEAVE_D
        self.label_heave_d = QtWidgets.QLabel(Dialog)
        self.label_heave_d.setGeometry(QtCore.QRect(170, 380, 21,17))
        self.label_heave_d.setObjectName("LABEL_HEAVE_D")
        #INPUT HEAVE_P
        self.heave_p_line = QtWidgets.QLineEdit(Dialog)
        self.heave_p_line.setGeometry(QtCore.QRect(10, 400, 61, 21))
        self.heave_p_line.setObjectName("LINE_HEAVE_P")
        #INPUT HEAVE_I
        self.heave_i_line = QtWidgets.QLineEdit(Dialog)
        self.heave_i_line.setGeometry(QtCore.QRect(80, 400, 61, 21))
        self.heave_i_line.setObjectName("LINE_HEAVE_I")
        #INPUT HEAVE_D
        self.heave_d_line = QtWidgets.QLineEdit(Dialog)
        self.heave_d_line.setGeometry(QtCore.QRect(150, 400, 61, 21))
        self.heave_d_line.setObjectName("LINE_HEAVE_D")
        #Button HEAVE
        self.poss_heave = QtWidgets.QPushButton(Dialog)
        self.poss_heave.setGeometry(QtCore.QRect(230, 390, 101,41))
        self.poss_heave.setObjectName("SEND HEAVE")




        self.retranslateUi(Dialog)
        QtCore.QMetaObject.connectSlotsByName(Dialog)
    
        self.poss_depth.clicked.connect(self.sendpositiondepth)
        self.poss_yaw.clicked.connect(self.sendpositionyaw)
        self.poss_roll.clicked.connect(self.send_roll_p and self.send_roll_i and self.send_roll_d)
        self.poss_pitch.clicked.connect(self.send_pitch_p and self.send_pitch_i and self.send_pitch_d)
        self.poss_yaw2.clicked.connect(self.send_yaw_p and self.send_yaw_i and self.send_yaw_d)
        self.poss_heave.clicked.connect(self.send_heave_p and self.send_heave_i and self.send_heave_d)


    def sendpositiondepth(self):
        self.setpoint.depth = float(self.depth_line.text())
        # self.pub_status.publish('posisi')
        self.pub_set_point.publish(self.setpoint)

    def sendpositionyaw(self):
        self.setpoint.yaw = int(self.yaw_line.text())
        # self.pub_status.publish('posisi')
        self.pub_set_point.publish(self.setpoint)
    
    def send_roll_p(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_roll_i(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_roll_d(self):
        print("TOLONG DIISI YA WHHEHEHEH")

    def send_pitch_p(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_pitch_i(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_pitch_d(self):
        print("TOLONG DIISI YA WHHEHEHEH")

    def send_yaw_p(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_yaw_i(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_yaw_d(self):
        print("TOLONG DIISI YA WHHEHEHEH")

    def send_heave_p(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_heave_i(self):
        print("TOLONG DIISI YA WHHEHEHEH")
    def send_heave_d(self):
        print("TOLONG DIISI YA WHHEHEHEH")

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "GUI_TECHSAS"))
        self.poss_depth.setText(_translate("Dialog", "SEND_DEPTH"))
        self.poss_yaw.setText(_translate("Dialog", "SEND_YAW"))
        self.poss_roll.setText(_translate("Dialog","SEND_ROLL"))
        self.poss_pitch.setText(_translate("Dialog","SEND_PITCH"))
        self.poss_yaw2.setText(_translate("Dialog","SEND_YAW"))
        self.poss_heave.setText(_translate("Dialog","SEND_HEAVE"))
        self.label_depth.setText(_translate("Dialog", "DEPTH"))
        self.label_yaw.setText(_translate("Dialog", "YAW"))
        self.label_roll.setText(_translate("Dialog", "ROLL"))
        self.label_roll_p.setText(_translate("Dialog", "P"))
        self.label_roll_i.setText(_translate("Dialog", "I"))
        self.label_roll_d.setText(_translate("Dialog", "D"))
        self.label_pitch.setText(_translate("Dialog", "PITCH"))
        self.label_pitch_p.setText(_translate("Dialog", "P"))
        self.label_pitch_i.setText(_translate("Dialog", "I"))
        self.label_pitch_d.setText(_translate("Dialog", "D"))
        self.label_yaw2.setText(_translate("Dialog", "YAW"))
        self.label_yaw2_p.setText(_translate("Dialog", "P"))
        self.label_yaw2_i.setText(_translate("Dialog", "I"))
        self.label_yaw2_d.setText(_translate("Dialog", "D"))
        self.label_heave.setText(_translate("Dialog", "HEAVE"))
        self.label_heave_p.setText(_translate("Dialog", "P"))
        self.label_heave_i.setText(_translate("Dialog", "I"))
        self.label_heave_d.setText(_translate("Dialog", "D"))



if __name__ == "__main__":
    try:
        rospy.init_node("Node_gui", anonymous=False)
        app = QtWidgets.QApplication(sys.argv)
        Dialog = QtWidgets.QDialog()
        ui = Ui_Dialog()
        ui.setupUI(Dialog)
        Dialog.show()
        sys.exit(app.exec_())
    
    except:
        rospy.ROSInterruptException


