from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

# UI block which includes a lable, a slider and an editor
class block:
    def __init__(self, window, name, x, y, l, r):
        super().__init__()
        self.initUI(window, name, x, y, l, r)
    
    def initUI(self, window, name, x, y, l, r):
        self.scale = 1000
        self.Lvalue = l
        self.Rvalue = r
        
        # name
        self.Lbl = QLabel(window)
        self.Lbl.setText(name)
        self.Lbl.move(x, y)
        
        # slider
        self.Sld = QSlider(Qt.Horizontal, window)
        self.Sld.name = name
        self.Sld.setMinimum(0)
        self.Sld.setMaximum(int(self.scale))
        self.Sld.setFocusPolicy(Qt.NoFocus)
        self.Sld.setGeometry(x+45, y-6, 100, 30)
        
        # show value
        self.Vel = QLabel(window)
        self.Vel.setText('0.000')
        self.Vel.move(x+160, y-2)
        
        # set value
        self.Tex = QLineEdit(window)
        self.Tex.setText('0.000')
        self.Tex.move(x+200, y-4)
        self.Tex.resize(50, 20)
        
        self.set(0)
    
    def set(self, value):
        position = int((value - self.Lvalue) * self.scale / (self.Rvalue - self.Lvalue))
        self.Sld.setValue(position)
        self.Vel.setText("{:.2f}".format(value))

class groupV:
    def __init__(self, window, x, y, itemSize, itemName, LBoundary, RBoundary, enable):
        super().__init__()
        self.initUI(window, x, y, itemSize, itemName, LBoundary, RBoundary, enable)
        self.itemValue = [0] * itemSize
    
    def initUI(self, window, x, y, itemSize, itemName, LBoundary, RBoundary, enable):
        self.itemSize = itemSize
        # group edge
        self.Edg = QLabel(window)
        self.Edg.setFrameStyle(QFrame.Panel  | QFrame.Plain)
        self.Edg.setAlignment(Qt.AlignBottom | Qt.AlignRight)
        self.Edg.setGeometry(20 + x, 20 + y, 280, 410)
        
        # item block
        self.B = []
        v = [40, 90, 140, 190, 240, 290, 340]
        for i in range(self.itemSize):
            self.B.append(block(window, itemName[i], 30 + x,  v[i] + y, LBoundary[i], RBoundary[i]))
        
        # enable button
        self.CheckBox_Enable = QCheckBox('Enable', window)
        self.CheckBox_Enable.move(30 + x, 393 + y)
        self.CheckBox_Enable.stateChanged.connect(enable)
        
        # back zero button
        self.Button_Zero = QPushButton('Zero', window)
        self.Button_Zero.move(100 + x, 390 + y)
        self.Button_Zero.clicked.connect(self.zero)
        
        # set value button
        self.Button_Set = QPushButton('Set', window)
        self.Button_Set.move(200 + x, 390 + y)
        self.Button_Set.clicked.connect(self.set)
    
    def zero(self):
        for i in range(self.itemSize):
            self.B[i].set(0)
            self.itemValue[i] = 0
    
    def set(self):
        for i in range(self.itemSize):
            value = float(self.B[i].Tex.text())
            self.B[i].set(value)
            self.itemValue[i] = value
    
    def update(self):
        for i in range(self.itemSize):
            self.B[i].set(self.itemValue[i])
            self.B[i].Tex.setText("{:.4f}".format(self.itemValue[i]))
    
class groupH:
    def __init__(self, window, x, y, connect, rcm, run, demo):
        super().__init__()
        self.initUI(window, x, y, connect, rcm, run, demo)
        self.withRCM = False
        self.onInterplation = False
    
    def initUI(self, window, x, y, connect, rcm, run, demo):
        # group edge
        self.Edg = QLabel(window)
        self.Edg.setFrameStyle(QFrame.Panel  | QFrame.Plain)
        self.Edg.setAlignment(Qt.AlignBottom | Qt.AlignRight)
        self.Edg.setGeometry(20 + x, 430 + y, 570, 60)
        
        # connet button
        self.CheckBox_Connect = QCheckBox('Robot', window)
        self.CheckBox_Connect.move(30 + x, 450 + y)
        self.CheckBox_Connect.stateChanged.connect(connect)
        
        # RCM button
        self.CheckBox_RCM = QCheckBox('RCM', window)
        self.CheckBox_RCM.move(110 + x, 450 + y)
        self.CheckBox_RCM.stateChanged.connect(rcm)
        
        # visual servoing button
        self.CheckBox_Vision = QCheckBox('Vision', window)
        self.CheckBox_Vision.move(190 + x, 450 + y)
        
        # voice button
        self.CheckBox_Voice = QCheckBox('Voice', window)
        self.CheckBox_Voice.move(270 + x, 450 + y)
        
        # IMU button
        self.CheckBox_IMU = QCheckBox('IMU', window)
        self.CheckBox_IMU.move(350 + x, 450 + y)

        # run button
        self.CheckBox_Run = QCheckBox('Run', window)
        self.CheckBox_Run.move(430 + x, 450 + y)
        self.CheckBox_Run.stateChanged.connect(run)

        # demo button
        self.CheckBox_Demo = QCheckBox('Demo', window)
        self.CheckBox_Demo.move(510 + x, 450 + y)
        self.CheckBox_Demo.stateChanged.connect(demo)
        
        # information board
        self.Info = QLabel(window)
        self.Info.setFrameStyle(QFrame.Panel | QFrame.Plain)
        self.Info.setAlignment(Qt.AlignTop   | Qt.AlignLeft)
        self.Info.setGeometry(20 + x, 500 + y, 570, 200)