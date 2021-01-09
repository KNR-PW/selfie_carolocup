import os
import rospy
import rospkg
import rosservice

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import UInt8
from std_msgs.msg import Int8
from std_srvs.srv import Empty

from custom_msgs.msg import Buttons


class MyPlugin(Plugin):
    LANE_PILOT_STATE_TOPIC = "/state/lane_control"
    TASK_STATE_TOPIC = "/state/task"
    BUTTON_TOPIC_NAME = "/selfie_out/buttons"
    CHANGE_RC_SERVICE_NAME = "/state/rc"
    RES_ODOM_SERVICE_NAME = "/reset/odom"
    RES_VISION_SERVICE_NAME = "/resetVision"
    RES_LANE_CONTROL_SERVICE_NAME = "/resetLaneControl"

    RC_MODES = {-1: "itself", 0: "uninitialized", 1: "manual mode",
                2: "semi-autonomous", 3: "autonomous mode"}

    TASKS = {0: "SELFIE_IDLE",
             1: "SELFIE_READY",
             2: "SELFIE_NO_ACTION",
             3: "BUTTON_PARKING_DRIVE_PRESSED ",
             4: "BUTTON_OBSTACLE_DRIVE_PRESSED",
             5: "START_DRIVE",
             6: "END_DRIVE",
             7: "AUTONOMOUS_DRIVE ",
             8: "DETECT_START_LINE",
             9: "DETECT_CROSS_LINE",
             10: "DETECT_CROSSROAD ",
             11: "DETECT_OBSTACLE",
             12: "CHANGE_LANE",
             13: "START_SEARCHING_PLACE",
             14: "FOUND_PLACE_MEASURING",
             15: "FIND_PROPER_PLACE",
             16: "START_PARK ",
             17: "IN_PLACE ",
             18: "OUT_PLACE",
             19: "READY_TO_DRIVE ",
             20: "APPROACHING_TO_INTERSECTION ",
             21: "APPROACHING_TO_INTERSECTION2",
             22: "STOPPED_ON_INTERSECTION ",
             23: "WAITING_ON_INTERSECTION",
             24: "ROAD_CLEAR"}

    LANE_MODES = {0: "UNINITIALIZED",
                  1: "PASSIVE_RIGHT",
                  2: "ON_RIGHT",
                  3: "OVERTAKE",
                  4: "ON_LEFT",
                  5: "RETURN_RIGHT"}

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path(
            'rqt_selfie_base'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.button_1.pressed.connect(self.press_button1)
        self._widget.button_2.pressed.connect(self.press_button2)
        self._widget.button_rc_change.pressed.connect(self.change_rc_mode)
        self._widget.button_res_lane.pressed.connect(self.restart_lane_control)
        self._widget.button_res_odometry.pressed.connect(self.restart_odometry)
        self._widget.button_res_vision.pressed.connect(self.restart_vision)

        # init publishers and subscribers
        self.pub_button = rospy.Publisher(
            self.BUTTON_TOPIC_NAME, Buttons, queue_size=1)
        self.pub_rc_state = rospy.Publisher(
            self.CHANGE_RC_SERVICE_NAME, UInt8, queue_size=1)
        self.srv_res_lane = rospy.ServiceProxy(
            self.RES_LANE_CONTROL_SERVICE_NAME, Empty)
        self.srv_res_odometry = rospy.ServiceProxy(
            self.RES_ODOM_SERVICE_NAME, Empty)
        self.srv_res_vision = rospy.ServiceProxy(
            self.RES_VISION_SERVICE_NAME, Empty)

        self.sub_lane_pilot_state = rospy.Subscriber(
            self.LANE_PILOT_STATE_TOPIC, Int8, self.lane_pilot_state_callback, queue_size=1)
        self.sub_task_state = rospy.Subscriber(
            self.TASK_STATE_TOPIC, Int8, self.task_state_callback, queue_size=1)
        self.sub_rc_state = rospy.Subscriber(
            self.CHANGE_RC_SERVICE_NAME, UInt8, self.changed_rc_callback, queue_size=1)

        # Other variables
        self.rc_mode = -1
        self._widget.rc_label.setText(self.RC_MODES[self.rc_mode])

    def press_button1(self):
        rospy.logdebug("Pressed button1 button")
        msg = Buttons(is_pressed_first=True, is_pressed_second=False)
        self.pub_button.publish(msg)

    def press_button2(self):
        rospy.logdebug("Pressed 'button2' button")
        msg = Buttons(is_pressed_first=False, is_pressed_second=True)
        self.pub_button.publish(msg)

    def change_rc_mode(self):
        rospy.logdebug("Pressed change RC button")
        self.rc_mode += 1
        if self.rc_mode >= len(self.RC_MODES) - 1:
            self.rc_mode = 1
        self.pub_rc_state.publish(self.rc_mode)

    def changed_rc_callback(self, data: UInt8):
        self._widget.rc_label.setText(self.RC_MODES[data.data])

    def restart_lane_control(self):
        rospy.logdebug("Pressed restart lane_control button")
        service_list = rosservice.get_service_list()
        if self.RES_LANE_CONTROL_SERVICE_NAME not in service_list:
            rospy.logwarn(self.RES_LANE_CONTROL_SERVICE_NAME +
                          " service server is not active")
        else:
            response = self.srv_res_lane()

    def restart_odometry(self):
        rospy.logdebug("Pressed restart odometry button")
        service_list = rosservice.get_service_list()
        if self.RES_ODOM_SERVICE_NAME not in service_list:
            rospy.logwarn(self.RES_ODOM_SERVICE_NAME +
                          " service server is not active")
        else:
            response = self.srv_res_odometry()

    def restart_vision(self):
        rospy.logdebug("Pressed restart vision button")
        service_list = rosservice.get_service_list()
        if self.RES_VISION_SERVICE_NAME not in service_list:
            rospy.logwarn(self.RES_VISION_SERVICE_NAME +
                          " service server is not active")
        else:
            response = self.srv_res_vision()

    def lane_pilot_state_callback(self, data: Int8):
        self._widget.lane_control_label.setText(self.LANE_MODES[data.data])

    def task_state_callback(self, data: Int8):
        self._widget.task_label.setText(self.TASKS[data.data])

    def shutdown_plugin(self):
        self.pub_button.unregister()
