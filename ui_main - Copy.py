import sys
from math import degrees, radians
import time

import qdarktheme
from PySide6 import QtWidgets
from PySide6.QtCore import QObject, QThread, Signal, QThreadPool, Slot
from spatialmath import SE3, SO3

import sim_swift
from ui import Ui_MainWindow

"""
class RefreshWorker(QObject):
    finished = Signal()
    def refresh(self):
        self.ThreadActive = True
        while self.ThreadActive:
            sim_swift.refresh()
        self.finished.emit()

class Worker(QObject):
    finished = Signal()
    progress = Signal()
    def JointTraj(self, coord, maxVel):
        traj = sim_swift.JointTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), coord, maxVel)
        # sim_swift.DrawRobot(sim_swift.JointTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), coord, maxVel))
        for i in range(len(traj.q)): # Transforming
            sim_swift.robot.q = traj.q[i]
            # self.progress.emit()
            # sim_swift.env.step(sim_swift.dt)
        self.finished.emit()
"""

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setFixedSize(1000, 800)

        self.ThreadManager = QThreadPool()

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

        # Костыли:
        self.ui.TrajectoryMotionButton.toggle()
        self.ui.Velocity_label.setText(str(self.ui.Velocity_slider.value()) + " %")

    @Slot()
    def Refresh(self):
        self.ThreadManager.start(self.RefreshSim)

    @Slot()
    def SimDraw(self, coord, maxVel):
        self.ThreadManager.start(self.DrawTraj(coord, maxVel))

    def DrawTraj(self, coord, maxVel):
        traj = sim_swift.JointTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), coord, maxVel)
        # sim_swift.DrawRobot(sim_swift.JointTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), coord, maxVel))
        for i in range(len(traj.q)): # Transforming
            sim_swift.robot.q = traj.q[i]
            self.Refresh()

    def RefreshSim(self):
        while True:
            sim_swift.refresh()

    def LaunchSim(self):
        sim_swift.StartSwift()
        # self.RefreshWorker = RefreshWorker()
        # self.RefreshThread = QThread()

        # self.RefreshWorker.moveToThread(self.RefreshThread)
        # self.RefreshThread.started.connect(self.RefreshWorker.refresh)

        # self.RefreshWorker.finished.connect(self.RefreshThread.quit)
        # self.RefreshWorker.finished.connect(self.RefreshWorker.deleteLater)
        # self.RefreshThread.finished.connect(self.RefreshThread.deleteLater)
        # self.RefreshThread.start()
        self.Refresh()
        # self.RefreshCoord()

    def Send(self):

        maxVel = [i * self.ui.Velocity_slider.value() / 100 for i in sim_swift.qdmax]
        cartGain = self.ui.Velocity_slider.value() / 100

        coord = self.GrabCoord()
    
        self.SimDraw(coord, maxVel)

        # self.SenderThread.start()

        """
        maxVel = [i * self.ui.Velocity_slider.value() / 100 for i in sim_swift.qdmax]
        cartGain = self.ui.Velocity_slider.value() / 100
        # maxVel = sim_swift.qdmax * self.ui.Velocity_slider.value() / 100
        if self.ui.JointJogButton.isChecked():
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
            if self.ui.TrajectoryType_comboBox.currentText() == "jtraj":
                self.ConsolePrint(f"{time.localtime()[3]}:{time.localtime()[4]}\t jtraj\t vel: {round(degrees(min(maxVel)))} deg / s")
                sim_swift.SwiftVisualise(sim_swift.JointTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), self.GrabCoord(), maxVel))
            if self.ui.TrajectoryType_comboBox.currentText() == "mstraj":
                self.ConsolePrint(f"{time.localtime()[3]}:{time.localtime()[4]}\t mstraj\t vel: {round(degrees(min(maxVel)))} deg / s")
                sim_swift.SwiftVisualise(sim_swift.MultiSegmentTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), self.GrabCoord(), maxVel))
            if self.ui.TrajectoryType_comboBox.currentText() == "ctraj":
                self.ConsolePrint(f"{time.localtime()[3]}:{time.localtime()[4]}\t ctraj\t vel: {round(degrees(min(maxVel)))} deg / s")
                sim_swift.SwiftVisualise(sim_swift.CartesianTrajectory(sim_swift.robot.fkine(sim_swift.robot.q), self.GrabCoord(), cartGain))
        """

    def GrabCoord(self):
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
        # threading.Timer(0.1, self.RefreshCoord).start()
        pose = sim_swift.robot.fkine(sim_swift.robot.q)[0]
        self.ui.toolCoordX_label.setText(str(round(pose.t[0] * 1000, 3)))
        self.ui.toolCoordY_label.setText(str(round(pose.t[1] * 1000, 3)))
        self.ui.toolCoordZ_label.setText(str(round(pose.t[2] * 1000, 3)))

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
            sim_swift.JointJog(sim_swift.robot.qz, maxVel)

    def Velocity(self):
        self.ui.Velocity_label.setText(str(self.ui.Velocity_slider.value()) + " %")

    def ConsolePrint(self, arg):
        self.ui.Console.appendPlainText(str(arg))

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    qdarktheme.setup_theme()
    window = MainWindow()
    window.setWindowTitle("Robot_GUI")
    window.show()

    sys.exit(app.exec())