import json
import socket
import sys
import time
from math import degrees, radians

import qdarktheme
from PySide6 import QtWidgets
from PySide6.QtCore import QLocale
from spatialmath import SE3, SO3

import sim_swift
from ui_main import Ui_MainWindow


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setFixedSize(1000, 800)

        # Setup UI:
        self.ui.LaunchButton.clicked.connect(self.LaunchSim)
        self.ui.SendButton.clicked.connect(self.Send)
        self.ui.HomeButton.clicked.connect(self.GoHome)
        self.ui.Velocity_slider.valueChanged.connect(self.Velocity)
    
        self.ui.JointJogButton.toggled.connect(self.ui.cart_X_doubleSpinBox.setDisabled)
        self.ui.JointJogButton.toggled.connect(self.ui.cart_Y_doubleSpinBox.setDisabled)
        self.ui.JointJogButton.toggled.connect(self.ui.cart_Z_doubleSpinBox.setDisabled)
        self.ui.JointJogButton.toggled.connect(self.ui.cart_Rx_SpinBox.setDisabled)
        self.ui.JointJogButton.toggled.connect(self.ui.cart_Ry_SpinBox.setDisabled)
        self.ui.JointJogButton.toggled.connect(self.ui.cart_Rz_SpinBox.setDisabled)

        self.ui.JointJogButton.toggled.connect(self.ui.TrajectoryType_comboBox.setDisabled)

        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis1.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis2.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis3.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis4.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis5.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis6.setDisabled)

        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis1Slider.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis2Slider.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis3Slider.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis4Slider.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis5Slider.setDisabled)
        self.ui.TrajectoryMotionButton.toggled.connect(self.ui.Axis6Slider.setDisabled)

        # Languages:
        self.__language_dict = {
            "English": "en_US",
            "Русиянский": "ru_RU"
        }
        self.ui.LangChange_comboBox.currentTextChanged.connect(self.__langChanged)

        # Костыли:
        self.ui.TrajectoryMotionButton.toggle()
        self.ui.SimulatorSwitch_radioButton.toggle()
        self.ui.Velocity_label.setText(str(self.ui.Velocity_slider.value()) + " %")
        self.connected = False

        # TCP UI:
        self.ui.TCP_Connect_button.clicked.connect(self.TCPConnect)

    def TCPConnect(self):
        if self.connected == False:
            self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_client.settimeout(0.5)

            host_addr = self.ui.TCP_host_addr_lineEdit.text()
            host_port = int(self.ui.TCP_host_port_lineEdit.text())
            self.tcp_client.connect((host_addr, host_port))

            self.ui.TCP_Connect_button.setText("Disconnect")
            self.ui.TCP_Connect_button.setStyleSheet("background-color : rgb(0,255,0); Font : bold")
            self.connected = True

        else:
            self.tcp_client.close()
            self.ui.TCP_Connect_button.setText("Connect")
            self.ui.TCP_Connect_button.setStyleSheet("background-color : rgb(255,0,0); Font : bold")
            self.connected = False

    def TCP_send(self, instructions: dict):
        if hasattr(self, "tcp_client"):
            json_string = json.dumps(instructions)
            text = json_string.encode(encoding='ascii')
            self.tcp_client.sendall(text)
            reply = self.tcp_client.recv(128)
            self.ConsolePrint(reply.decode(encoding='ascii'))

    def LaunchSim(self):
        sim_swift.StartSwift()
        self.RefreshCoord()

    def Send(self):
        maxVel = [i * self.ui.Velocity_slider.value() / 100 for i in sim_swift.qdmax]
        cartGain = self.ui.Velocity_slider.value() / 100

        if self.ui.JointJogButton.isChecked():
            if self.ui.SimulatorSwitch_radioButton.isChecked():
                self.ConsolePrint(f"{time.localtime()[3]}:{time.localtime()[4]}\t jog\t vel: {round(degrees(min(maxVel)))} deg / s")
                sim_swift.SwiftVisualise(sim_swift.JointJog([
                    radians(self.ui.Axis1.value()),
                    radians(self.ui.Axis2.value()),
                    radians(self.ui.Axis3.value()),
                    radians(self.ui.Axis4.value()),
                    radians(self.ui.Axis5.value()),
                    radians(self.ui.Axis6.value()),
                ], maxVel))
            else:
                memory_write = {
                    "dsID":"www.hc-system.com.RemoteMonitor",
                    "reqType": "command",
                    "cmdData":
                            ["rewriteDataList", "800","6","0", 
                                str(self.ui.Axis1.value() * 1000), 
                                str(self.ui.Axis2.value() * 1000), 
                                str(self.ui.Axis3.value() * 1000), 
                                str(self.ui.Axis4.value() * 1000), 
                                str(self.ui.Axis5.value() * 1000), 
                                str(self.ui.Axis6.value() * 1000)
                            ]
                }
                self.TCP_send(memory_write)
        else:
            if self.ui.SimulatorSwitch_radioButton.isChecked():
                if self.ui.TrajectoryType_comboBox.currentText() == "jtraj":
                    self.ConsolePrint(f"{time.localtime()[3]}:{time.localtime()[4]}\t jtraj\t vel: {round(degrees(min(maxVel)))} deg / s")
                    sim_swift.SwiftVisualise(sim_swift.JointTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), self.GrabCoord(), maxVel))
                if self.ui.TrajectoryType_comboBox.currentText() == "mstraj":
                    self.ConsolePrint(f"{time.localtime()[3]}:{time.localtime()[4]}\t mstraj\t vel: {round(degrees(min(maxVel)))} deg / s")
                    sim_swift.SwiftVisualise(sim_swift.MultiSegmentTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), self.GrabCoord(), maxVel))
                if self.ui.TrajectoryType_comboBox.currentText() == "ctraj":
                    self.ConsolePrint(f"{time.localtime()[3]}:{time.localtime()[4]}\t ctraj\t vel: {round(degrees(min(maxVel)))} deg / s")
                    sim_swift.SwiftVisualise(sim_swift.CartesianTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), self.GrabCoord(), cartGain))
            else:
                memory_write = {
                    "dsID":"www.hc-system.com.RemoteMonitor",
                    "reqType": "command",
                    "cmdData":
                            ["rewriteDataList", "800","6","0", 
                                str(round(self.ui.cart_X_doubleSpinBox.value() * 1000)), 
                                str(round(self.ui.cart_Y_doubleSpinBox.value() * 1000)), 
                                str(round(self.ui.cart_Z_doubleSpinBox.value() * 1000)), 
                                str(round(self.ui.cart_Rx_SpinBox.value())), 
                                str(round(self.ui.cart_Ry_SpinBox.value())), 
                                str(round(self.ui.cart_Rz_SpinBox.value()))
                            ]
                }
                self.TCP_send(memory_write)
            self.RefreshCoord()

    def GrabCoord(self) -> SE3:
        target = SE3.Trans(
            self.ui.cart_X_doubleSpinBox.value() / 1000,
            self.ui.cart_Y_doubleSpinBox.value() / 1000,
            self.ui.cart_Z_doubleSpinBox.value() / 1000
        ) * SE3.RPY([ # ) * SE3.EulerVec([
            radians(self.ui.cart_Rx_SpinBox.value()),
            radians(self.ui.cart_Ry_SpinBox.value()),
            radians(self.ui.cart_Rz_SpinBox.value())
            ])
        return target

    def RefreshCoord(self):
        # threading.Timer(0.5, self.RefreshCoord).start()
        pose = sim_swift.robot.fkine(sim_swift.robot.q)[0]
        self.ui.toolCoordX_label.setText(str(round(pose.t[0] * 1000, 3)))
        self.ui.toolCoordY_label.setText(str(round(pose.t[1] * 1000, 3)))
        self.ui.toolCoordZ_label.setText(str(round(pose.t[2] * 1000, 3)))

        # eul_angles = SE3.eul(sim_swift.robot.fkine(sim_swift.robot.q), unit="deg")
        eul_angles = SO3.rpy(sim_swift.robot.fkine(sim_swift.robot.q))

        self.ui.toolCoordRx_label.setText(str(round(degrees(eul_angles[1]), 3)))
        self.ui.toolCoordRy_label.setText(str(round(degrees(eul_angles[0]), 3)))
        self.ui.toolCoordRz_label.setText(str(round(degrees(eul_angles[2]), 3)))

        self.ui.Joint1Angle_label.setText(str(round(degrees(sim_swift.robot.q[0]), 3)))
        self.ui.Joint2Angle_label.setText(str(round(degrees(sim_swift.robot.q[1]), 3)))
        self.ui.Joint3Angle_label.setText(str(round(degrees(sim_swift.robot.q[2]), 3)))
        self.ui.Joint4Angle_label.setText(str(round(degrees(sim_swift.robot.q[3]), 3)))
        self.ui.Joint5Angle_label.setText(str(round(degrees(sim_swift.robot.q[4]), 3)))
        self.ui.Joint6Angle_label.setText(str(round(degrees(sim_swift.robot.q[5]), 3)))

    def GoHome(self):
        maxVel = [i * self.ui.Velocity_slider.value() / 100 for i in sim_swift.qdmax]
        if self.ui.JointJogButton.isChecked():
            self.ui.Axis1.setValue(0)
            self.ui.Axis2.setValue(0)
            self.ui.Axis3.setValue(0)
            self.ui.Axis4.setValue(0)
            self.ui.Axis5.setValue(0)
            self.ui.Axis6.setValue(0)
        elif self.ui.TrajectoryMotionButton.isChecked():
            sim_swift.SwiftVisualise(sim_swift.JointJog(sim_swift.robot.qz, maxVel))

    def Velocity(self):
        self.ui.Velocity_label.setText(str(self.ui.Velocity_slider.value()) + " %")

    def ConsolePrint(self, arg):
        self.ui.Console.appendPlainText(str(arg))

    def __langChanged(self, lang: str) -> dict:
        translations = {}
        with open('lang.json', 'r', encoding='utf-8') as file:
            translations = json.load(file)

        language = lang
        if not lang:
            language = QLocale.system().name()
            if language not in translations:
                language = 'en_US'  # Default language


        translation = translations[self.__language_dict[language]]
        self.__changeLang(translation)

    def __changeLang(self, translation: dict):
        self.ui.Axis1Label.setText(translation["Axis_1"])
        self.ui.Axis2Label.setText(translation["Axis_2"])
        self.ui.Axis3Label.setText(translation["Axis_3"])
        self.ui.Axis4Label.setText(translation["Axis_4"])
        self.ui.Axis5Label.setText(translation["Axis_5"])
        self.ui.Axis6Label.setText(translation["Axis_6"])
        self.ui.MotionType_label.setText(translation["MotionType"])
        self.ui.JointJogButton.setText(translation["JointJog"])
        self.ui.TrajectoryMotionButton.setText(translation["Trajectory"])
        self.ui.TrajectoryType_label.setText(translation["TrajectoryType"])
        self.ui.LaunchButton.setText(translation["LaunchSim"])
        self.ui.SendButton.setText(translation["SendButton"])
        self.ui.HomeButton.setText(translation["Home"])
        self.ui.VelocityTag_label.setText(translation["Velocity"])
        self.ui.ToolPosition_label.setText(translation["ToolPos"])
        self.ui.JointAngles_label.setText(translation["JointAngle"])
        self.ui.ConsoleLabel.setText(translation["Log"])
        self.ui.ConsoleSend_button.setText(translation["LogSendButton"])

if __name__ == "__main__":
    # appThread = threading.Thread()

    app = QtWidgets.QApplication(sys.argv)
    qdarktheme.setup_theme()
    window = MainWindow()
    window.setWindowTitle("Robot_GUI")
    window.show()

    sys.exit(app.exec())