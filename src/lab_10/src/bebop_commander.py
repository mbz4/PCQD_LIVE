#!/usr/bin/env python3
'''
### Node for controlling take off, landing, and basic movements of the drone.
    cmd src from docs: https://bebop-autonomy.readthedocs.io/en/latest/piloting.html#start-flight-plan
    concept inspired by: https://github.com/LTU-RAI/bebop_gui/blob/7185a1f03cacf6488bc481c4b881a1cea6464dca/bebop_gui.py#L138
    TODO: 
    - add acrobatics buttons
    - rosbag record btn for later ML methods (TBA)
    - camera adjustment commands w/ shortcuts & buttons
    - basic file manager for topics to record (TBA)
    - battery status bar w/ auto emergency landing when @<15% capacity
    - acquired rosbag preprocessing function (TBA)
    - training ML methods w/ acquired data (TBA)
    - selectors for different path planning solutions (TBA)
    - rqt reconfigure
'''
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from PyQt5.QtWidgets import  QWidget, QApplication, QPushButton, QDial, QShortcut, QProgressBar, QDesktopWidget, QTableWidgetItem, QLabel
from PyQt5.QtGui import QKeySequence, QImage, QPixmap, QColor#, QIcon
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot#, QSize
import sys
import numpy as np
#import signal
from cv_bridge import CvBridge

class App(QWidget):
    changeQImage = pyqtSignal(QImage) #Creating a signal
    changeBatteryBar = pyqtSignal(CommonCommonStateBatteryStateChanged) #Creating a signal

    #checkHz = pyqtSignal(list)
    # ----------------------------------------------------------------------------------------
    def __init__(self):
        super(QWidget, self).__init__()
        self.velocityPub = rospy.Publisher("/bebop/velocity", Twist, queue_size=10) #og: cmd_vel
        self.takeoffPub = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.landPub = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.emergency = rospy.Publisher("/bebop/reset", Empty, queue_size=10)
        self.cameraPub = rospy.Publisher("/bebop/camera_control", Twist, queue_size=10)
        self.batterySub = rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged, self.batteryCallback, queue_size=10)
        self.imgSub = rospy.Subscriber("/bebop/image_raw", Image, self.imageCallback, queue_size = 10)
        self.goFwrd_vel = Twist()
        self.goBwrd_vel = Twist()
        self.turnCCW_vel = Twist()
        self.turnCW_vel = Twist()
        self.translateRight_vel = Twist()
        self.translateLeft_vel = Twist()
        self.ascend_vel = Twist()
        self.descend_vel = Twist()
        self.hover_vel = Twist()
        self.move_camera = Twist()
        self.vel_scale = 1.0
        self.MAX_SPEED = 1
        self.ENABLE_GUI = True
        self.debug_msg = False # set to true to enable debug msgs to terminal
        self.hover()
        self.initUI()
    #-------------------------------------------------------------------------------------------------------
    
    @pyqtSlot(QImage)
    def setImage(self, image):
        self.imageLabel.setPixmap(QPixmap.fromImage(image)) #Setting QImage in imageLabel

    @pyqtSlot(CommonCommonStateBatteryStateChanged)
    def setBatteryBar(self, status):
        self.batteryBar.setValue(status.percent) # setting battery percent
    
        #Callback for QImage
    def imageCallback(self, data):
        bridge = CvBridge()
        rgbImage = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough") #converting rosimage to cvimage
        h, w, ch = rgbImage.shape
        bytesPerLine = ch * w
        convertToQtFormat = QImage(rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888) #converting cvimage to QImage
        p = convertToQtFormat.scaled(320, 240, Qt.KeepAspectRatio)
        self.changeQImage.emit(p) #emitting QImage through changeQImage-signal to setImage

        #Callback for battery
    def batteryCallback(self, status):
        self.changeBatteryBar.emit(status) #emitting battarystatus through changeBatteryBar-signal to setBatteryBar
        if self.debug_msg:
            rospy.loginfo(f"battery status = {status.percent}%")
        if status.percent < 15:
            #self.emergencyStop_Shortcut()
            rospy.loginfo(f"\n\tLow Battery Warning\n\t\t{status.percent}%")
    # ----------------------------------------------------------------------------------------
    # emergency >> sends linear and ang velocity = 0; then lands the drone
    def emergencyStop(self):
        if self.stop_btn.isChecked():
            self.stop_btn.setStyleSheet("border-radius :40; background-color: darkred; border : 2px solid darkred;font-size: 25px;font-family: Arial")
            self.hover_vel.linear.x = 0
            self.hover_vel.linear.y = 0
            self.hover_vel.linear.z = 0
            self.hover_vel.angular.z = 0
            self.velocityPub.publish(self.hover_vel)
            self.landPub.publish()
            # self.emergency.publish() # resets drone instantly
            self.ENABLE_GUI = False 
        else:
            self.stop_btn.setStyleSheet("border-radius :40; background-color: red; border : 2px solid darkred;font-size: 25px;font-family: Arial")
            self.ENABLE_GUI = True
        
        # disable all inputs in emergency mode
        self.shortcut_fwrd.setEnabled(self.ENABLE_GUI)
        self.shortcut_bwrd.setEnabled(self.ENABLE_GUI)
        self.shortcut_turnCW.setEnabled(self.ENABLE_GUI)
        self.shortcut_turnCCW.setEnabled(self.ENABLE_GUI)
        self.shortcut_takeoff.setEnabled(self.ENABLE_GUI)
        self.shortcut_land.setEnabled(self.ENABLE_GUI)
        self.turnCW_btn.setEnabled(self.ENABLE_GUI)
        self.turnCCW_btn.setEnabled(self.ENABLE_GUI)
        self.translateLeft_btn.setEnabled(self.ENABLE_GUI)
        self.translateRight_btn.setEnabled(self.ENABLE_GUI)
        self.goBwrd_btn.setEnabled(self.ENABLE_GUI)
        self.goFwrd_btn.setEnabled(self.ENABLE_GUI)
        self.land_btn.setEnabled(self.ENABLE_GUI)
        self.takeoff_btn.setEnabled(self.ENABLE_GUI)
        self.vel_slider.setEnabled(self.ENABLE_GUI)
        self.throttleBar.setEnabled(self.ENABLE_GUI)
        self.ascend_btn.setEnabled(self.ENABLE_GUI)
        self.descend_btn.setEnabled(self.ENABLE_GUI)
        #self.camUp_btn.setEnabled(self.ENABLE_GUI)
        #self.camDown_btn.setEnabled(self.ENABLE_GUI)

        self.shortcut_translateRight.setEnabled(self.ENABLE_GUI)
        self.shortcut_translateLeft.setEnabled(self.ENABLE_GUI)
        self.shortcut_fwrd.setEnabled(self.ENABLE_GUI)
        self.shortcut_bwrd.setEnabled(self.ENABLE_GUI)
        self.shortcut_turnCW.setEnabled(self.ENABLE_GUI)
        self.shortcut_turnCCW.setEnabled(self.ENABLE_GUI)
        self.shortcut_takeoff.setEnabled(self.ENABLE_GUI)
        self.shortcut_land.setEnabled(self.ENABLE_GUI)
        self.shortcut_ascend.setEnabled(self.ENABLE_GUI)
        self.shortcut_descend.setEnabled(self.ENABLE_GUI)
        self.shortcut_increment_throttle_val.setEnabled(self.ENABLE_GUI)
        self.shortcut_decrement_throttle_val.setEnabled(self.ENABLE_GUI)
        #self.shortcut_cam_up.setEnabled(self.ENABLE_GUI)
        #self.shortcut_cam_down.setEnabled(self.ENABLE_GUI)
        self.exit.setEnabled(self.ENABLE_GUI)
        if self.debug_msg:
            rospy.loginfo(f"emergencyStop")
    # ----------------------------------------------------------------------------------------
    # in case of spacebar key press, execute emergency stop
    def emergencyStop_Shortcut(self):
        if not self.stop_btn.isChecked():
            self.stop_btn.setChecked(True)
        else:
            self.stop_btn.setChecked(False)
        self.emergencyStop()
    
    # Takeoff >> publish once a std_msgs/Empty type message to /takeoff topic
    def takeOff(self):
        self.takeoffPub.publish()
        if self.debug_msg:
            rospy.loginfo(f"takeOff")

    # Land >> publish once a std_msgs/Empty type message to /land topic
    def land(self):
        self.landPub.publish()
        if self.debug_msg:
            rospy.loginfo(f"land")
    
    def goFwrd(self):
        self.goFwrd_vel.linear.x = +1*self.vel_scale
        self.velocityPub.publish(self.goFwrd_vel)
        if self.debug_msg:
            rospy.loginfo(f"goFwrd")
    
    def camUp(self):
        self.move_camera.angular.y += 10
        self.cameraPub.publish(self.move_camera)
        if self.debug_msg:
            rospy.loginfo(f"camUP")
    
    def camDown(self):
        self.move_camera.angular.y -= 10
        self.cameraPub.publish(self.move_camera)
        if self.debug_msg:
            rospy.loginfo(f"camDown")

    def goBwrd(self):
        self.goBwrd_vel.linear.x = -1*self.vel_scale
        self.velocityPub.publish(self.goBwrd_vel)
        if self.debug_msg:
            rospy.loginfo(f"goBwrd")

    def turnCCW(self):
        self.turnCCW_vel.angular.z = -0.5*self.vel_scale
        self.velocityPub.publish(self.turnCCW_vel)
        if self.debug_msg:
            rospy.loginfo(f"turnCCW")

    def turnCW(self):
        self.turnCW_vel.angular.z = +0.5*self.vel_scale
        self.velocityPub.publish(self.turnCW_vel)
        if self.debug_msg:
            rospy.loginfo(f"turnCCW")
    
    def translateRight(self):
        self.translateRight_vel.linear.y = -1*self.vel_scale
        self.velocityPub.publish(self.translateRight_vel)
        if self.debug_msg:
            rospy.loginfo(f"translateRight")

    def translateLeft(self):
        self.translateLeft_vel.linear.y = 1*self.vel_scale
        self.velocityPub.publish(self.translateLeft_vel)
        if self.debug_msg:
            rospy.loginfo(f"translateLeft")

    def ascend(self):
        self.ascend_vel.linear.z = +0.5*self.vel_scale
        self.velocityPub.publish(self.ascend_vel)
        if self.debug_msg:
            rospy.loginfo(f"ascend")

    def descend(self):
        self.descend_vel.linear.z = -0.5*self.vel_scale
        self.velocityPub.publish(self.descend_vel)
        if self.debug_msg:
            rospy.loginfo(f"descend")

    def hover(self):
        self.hover_vel.linear.x = 0
        self.hover_vel.linear.y = 0
        self.hover_vel.linear.z = 0
        self.hover_vel.angular.z = 0 
        if self.debug_msg:
            rospy.loginfo(f"hover")
    
    def changeVelValue(self, value):
        self.vel_scale = np.interp(value, (0, 100), (0, self.MAX_SPEED))
        self.throttleBar.setValue(int(self.vel_scale*100))

    def increment_vel_value(self):
        self.vel_scale += 0.025
        self.vel_scale = np.clip(self.vel_scale, 0, self.MAX_SPEED)
        self.throttleBar.setValue(int(self.vel_scale*100))
        self.vel_slider.setValue(int(self.vel_scale*100))
        if self.debug_msg:
            rospy.loginfo(f"increment_vel_value")
    
    def decrement_vel_value(self):
        self.vel_scale -= 0.025
        self.vel_scale = np.clip(self.vel_scale, 0, self.MAX_SPEED)
        self.throttleBar.setValue(int(self.vel_scale*100))
        self.vel_slider.setValue(int(self.vel_scale*100))
        if self.debug_msg:
            rospy.loginfo(f"decrement_vel_value")

    # experimental fast exit shortcut
    def on_exit(self):
        self.emergencyStop_Shortcut()
        if self.debug_msg:
            rospy.loginfo(f"exiting...")
        sys.exit()
    # ----------------------------------------------------------------------------------------
    def initUI(self):
        #create mainwindow
        
        qtRectangle = self.frameGeometry()
        
        centerPoint = QDesktopWidget().availableGeometry().topLeft()
        qtRectangle.moveCenter(centerPoint)
        #self.setGeometry(250, 100, 1100, 687)
        self.move(qtRectangle.topRight())
        self.setWindowTitle("BEBOP COMMANDER")
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        
        #creating edge label2
        self.edgeLabel = QLabel(self)
        self.edgeLabel.setStyleSheet("border: 2px solid grey; background-color: #e3e1e1")
        self.edgeLabel.move(325, 4)
        self.edgeLabel.resize(320, 240)
        
        #create a Image Label
        self.imageLabel = QLabel(self)
        self.imageLabel.move(325, 4)
        self.imageLabel.resize(320, 240)
        
        #create push button - big red round emergency stop button
        self.stop_btn = QPushButton('STOP', self)
        self.stop_btn.setGeometry(1000, 85, 80, 80)
        self.stop_btn.setCheckable(True)
        self.stop_btn.setChecked(False)
        self.stop_btn.move(240, 10)
        self.stop_btn.setStyleSheet("border-radius :40; background-color: red; border : 2px solid darkred;font-size: 25px;font-family: Arial")
        self.stop_btn.clicked.connect(self.emergencyStop)

        #Creating vel_scale label
        self.vel_label = QLabel("Vel [%]", self)
        self.vel_label.move(110, 80)
        self.vel_label.resize(100, 20)

        #create dial - adjust velocity
        self.vel_slider = QDial(self)
        self.vel_slider.setNotchesVisible(True)
        self.vel_slider.move(25,120)
        self.vel_slider.resize(110,110)
        self.vel_slider.setMinimum(0)
        self.vel_slider.setMaximum(100)
        self.vel_slider.setValue(int(self.vel_scale*100))
        self.vel_slider.valueChanged[int].connect(self.changeVelValue)

        #create throttle 'progress' bar - throttle [%]
        self.throttleBar = QProgressBar(self)
        self.throttleBar.move(140, 110)
        self.throttleBar.setOrientation(Qt.Vertical)
        self.throttleBar.setMaximum(100)
        self.throttleBar.resize(15, 120)
        self.throttleBar.setValue(int(self.vel_scale*100))

        #create takeoff/land label
        self.camera_tilt_label = QLabel("Takeoff/Land", self)
        self.camera_tilt_label.move(130, 5)
        self.camera_tilt_label.resize(120, 15)

        #create push button - takeoff
        self.takeoff_btn = QPushButton(u"\u21F1", self)
        self.takeoff_btn.resize(45,45)
        self.takeoff_btn.move(130,30)
        self.takeoff_btn.clicked.connect(self.takeOff)
        
        #create push button - land
        self.land_btn = QPushButton(u"\u21F2", self)
        self.land_btn.resize(45,45)
        self.land_btn.move(180,30)
        self.land_btn.clicked.connect(self.land)

        #create push button - rotate ccw
        self.turnCCW_btn = QPushButton(u"\u21bb", self)
        self.turnCCW_btn.setAutoRepeat(True)
        self.turnCCW_btn.resize(45,45)
        self.turnCCW_btn.move(170,100)
        self.turnCCW_btn.clicked.connect(self.turnCCW)

        #create push button - rotate cw
        self.turnCW_btn = QPushButton(u"\u21ba", self)
        self.turnCW_btn.setAutoRepeat(True)
        self.turnCW_btn.resize(45,45)
        self.turnCW_btn.move(170,200)
        self.turnCW_btn.clicked.connect(self.turnCW)

        #create push button - ascend
        self.ascend_btn = QPushButton(u"\u21c8", self)
        self.ascend_btn.setAutoRepeat(True)
        self.ascend_btn.resize(45,45)
        self.ascend_btn.move(270,100)
        self.ascend_btn.clicked.connect(self.ascend)

        #create push button - descend
        self.descend_btn = QPushButton(u"\u21ca", self)
        self.descend_btn.setAutoRepeat(True)
        self.descend_btn.resize(45,45)
        self.descend_btn.move(270,200)
        self.descend_btn.clicked.connect(self.descend)

        #create push button - go forward
        self.goFwrd_btn = QPushButton(u"\u2191", self)
        self.goFwrd_btn.setAutoRepeat(True)
        self.goFwrd_btn.resize(45,45)
        self.goFwrd_btn.move(220,100)
        self.goFwrd_btn.clicked.connect(self.goFwrd)
        
        #create push button - go backward
        self.goBwrd_btn = QPushButton(u"\u2193", self)
        self.goBwrd_btn.setAutoRepeat(True)
        self.goBwrd_btn.resize(45,45)
        self.goBwrd_btn.move(220,200)
        self.goBwrd_btn.clicked.connect(self.goBwrd)
        
        #create push button - translate right
        self.translateRight_btn = QPushButton(u"\u2192", self)
        self.translateRight_btn.setAutoRepeat(True)
        self.translateRight_btn.resize(45,45)
        self.translateRight_btn.move(270,150)
        self.translateRight_btn.clicked.connect(self.translateRight)
        
        #create push button - translate left
        self.translateLeft_btn = QPushButton(u"\u2190", self)
        self.translateLeft_btn.setAutoRepeat(True)
        self.translateLeft_btn.resize(45,45)
        self.translateLeft_btn.move(170,150)
        self.translateLeft_btn.clicked.connect(self.translateLeft)
        
        #create camera tilt label
        self.camera_tilt_label = QLabel(u"Camera \u21f3", self)
        self.camera_tilt_label.move(5, 5)
        self.camera_tilt_label.resize(100, 15)

        #create push button - camera up
        self.camUp_btn = QPushButton(u"\u21e7", self)
        self.camUp_btn.resize(45,45)
        self.camUp_btn.move(55,30)
        self.camUp_btn.clicked.connect(self.camUp)

        #create push button - camera up
        self.camDown_btn = QPushButton(u"\u21e9", self)
        self.camDown_btn.resize(45,45)
        self.camDown_btn.move(5,30)
        self.camDown_btn.clicked.connect(self.camDown)
        
        #Creating battery label
        self.batteryLabel = QLabel("Bat [%]", self)
        self.batteryLabel.move(5, 80)
        self.batteryLabel.resize(60, 20)

        #create battery bar
        self.batteryBar = QProgressBar(self)
        self.batteryBar.move(5, 110)
        self.batteryBar.setOrientation(Qt.Vertical)
        self.batteryBar.setMaximum(100)
        self.batteryBar.resize(15, 120)

        self.update()
        self.show() #showing GUI
        
        self.velocityPub.publish(self.hover_vel)
    # ----------------------------------------------------------------------------------------
        # keyboard shortcut commands
        self.shortcut_fwrd = QShortcut(QKeySequence("w"), self)
        self.shortcut_fwrd.activated.connect(self.goFwrd)
        
        self.shortcut_bwrd = QShortcut(QKeySequence("s"), self)
        self.shortcut_bwrd.activated.connect(self.goBwrd)
        
        self.shortcut_turnCW = QShortcut(QKeySequence("Shift+A"), self)
        self.shortcut_turnCW.activated.connect(self.turnCW)
        
        self.shortcut_turnCCW = QShortcut(QKeySequence("Shift+D"), self)
        self.shortcut_turnCCW.activated.connect(self.turnCCW)
        
        self.shortcut_translateLeft = QShortcut(QKeySequence("a"), self)
        self.shortcut_translateLeft.activated.connect(self.translateLeft)

        self.shortcut_translateRight = QShortcut(QKeySequence("d"), self)
        self.shortcut_translateRight.activated.connect(self.translateRight)

        self.shortcut_takeoff = QShortcut(QKeySequence("t"), self)
        self.shortcut_takeoff.activated.connect(self.takeOff)
        
        self.shortcut_land = QShortcut(QKeySequence("g"), self)
        self.shortcut_land.activated.connect(self.land)
        
        self.shortcut_ascend = QShortcut(QKeySequence("e"), self)
        self.shortcut_ascend.activated.connect(self.ascend)
        
        self.shortcut_descend = QShortcut(QKeySequence("q"), self)
        self.shortcut_descend.activated.connect(self.descend)
        
        self.shortcut_increment_throttle_val = QShortcut(QKeySequence("Shift+E"), self)
        self.shortcut_increment_throttle_val.activated.connect(self.increment_vel_value)
        
        self.shortcut_decrement_throttle_val = QShortcut(QKeySequence("Shift+Q"), self)
        self.shortcut_decrement_throttle_val.activated.connect(self.decrement_vel_value)
        
        self.shortcut_cam_up = QShortcut(QKeySequence("Shift+R"), self)
        self.shortcut_cam_up.activated.connect(self.camUp)
        
        self.shortcut_cam_down = QShortcut(QKeySequence("Shift+F"), self)
        self.shortcut_cam_down.activated.connect(self.camDown)

        self.shortcut_emergency_stop = QShortcut(QKeySequence(Qt.Key_Space), self)
        self.shortcut_emergency_stop.activated.connect(self.emergencyStop_Shortcut)
        
        self.exit = QShortcut("Escape", self)
        self.exit.activated.connect(self.on_exit)

        self.changeQImage.connect(self.setImage) #connecting signal to slot
        self.changeBatteryBar.connect(self.setBatteryBar) #connecting signal to slot
    # ----------------------------------------------------------------------------------------

if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node("bebop_commander", anonymous=True)
    rospy.loginfo("Starting Bebop Commander...")
    ex = App()
    sys.exit(app.exec_())
