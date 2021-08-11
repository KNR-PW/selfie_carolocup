import os
import rospy
import rospkg
import rosservice
import math
from datetime import datetime, timedelta

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Qt
from std_msgs.msg import Int8
from std_srvs.srv import Empty
from rqt_selfie_base.car_widget import CarWidget

from custom_msgs.msg import Buttons
from custom_msgs.msg import DriveCommand


class CarStatusPlugin(Plugin):
    LANE_PILOT_STATE_TOPIC = "/state/lane_control"
    TASK_STATE_TOPIC = "/state/task"
    BUTTON_TOPIC_NAME = "/selfie_out/buttons"
    RC_STATUS_TOPIC = "/state/rc"
    RES_ODOM_SERVICE_NAME = "/reset/odom"
    RES_VISION_SERVICE_NAME = "/resetVision"
    RES_LANE_CONTROL_SERVICE_NAME = "/resetLaneControl"
    RES_GAZEBO_WORLD = "/gazebo/reset_world"
    DRIVE_COMMAND_TOPIC = "/selfie_in/drive"

    RC_MODES = {
        -1: "itself",
        0: "manual mode",
        1: "semi-autonomous",
        2: "autonomous mode"
    }

    TASKS = {
        0: "TASK_SHIFTING",
        1: "WAITING_FOR_BUTTON",
        2: "GATE_CLOSED",
        3: "STARTING_DRIVE",
        4: "STARTING_DISTANCE_REACHED",
        5: "AUTONOMOUS_DRIVE",
        6: "STARTING_LINE_DETECTED",
        7: "INTERSECTION_STOP_DETECTED",
        8: "EVENT_SENT",
        9: "SEARCHING_PARKING_SPOT",
        10: "PLACE_INITIALLY_FOUND",
        11: "PLACE_PROPER_FOUND",
        12: "PLACE_NOT_FOUND",
        13: "APPROACHING_TO_PARKING_SPOT",
        14: "ENTRY_PARKING_SPOT",
        15: "PARKING_IDLE",
        16: "EXIT_PARKING_SPOT",
        17: "PARKING_COMPLETED",
        18: "APPROACHING_TO_INTERSECTION",
        19: "BLIND_APPROACHING",
        20: "STOP_TIME_ON_INTERSECTION",
        21: "STOP_OBSTACLE_ON_INTERSECTION",
        22: "PASSING_INTERSECTION"
    }

    LANE_MODES = {
        0: "UNINITIALIZED",
        1: "PASSIVE_RIGHT",
        2: "ON_RIGHT",
        3: "OVERTAKE",
        4: "ON_LEFT",
        5: "RETURN_RIGHT"
    }

    def __init__(self, context):
        super(CarStatusPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CarStatusPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_selfie_base'),
                               'resource', 'CarStatusPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('CarStatusPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.button_1.pressed.connect(self.press_button1)
        self._widget.button_2.pressed.connect(self.press_button2)
        self._widget.button_res_lane.pressed.connect(self.restart_lane_control)
        self._widget.button_res_odometry.pressed.connect(self.restart_odometry)
        self._widget.button_res_vision.pressed.connect(self.restart_vision)
        self._widget.button_restart_simulation.pressed.connect(
            self.restart_simulation)
        self._widget.check_box_advanced_view.stateChanged.connect(
            self.switch_view_callback)
        self._widget.button_select_manual.clicked.connect(self.change_mode)
        self._widget.button_select_semi_auto.clicked.connect(self.change_mode)
        self._widget.button_select_auto.clicked.connect(self.change_mode)

        self.car_scene = CarWidget()
        self._widget.graphicsView.setScene(self.car_scene)
        # Reimplement resize Event to fit scene
        self._widget.graphicsView.resizeEvent = lambda x: self._widget.graphicsView.fitInView(
            self.car_scene.sceneRect(), Qt.KeepAspectRatio)

        # init publishers and subscribers
        self.pub_button = rospy.Publisher(self.BUTTON_TOPIC_NAME,
                                          Buttons,
                                          queue_size=1)
        self.pub_switch_state = rospy.Publisher("/simulation/switch_state",
                                                Int8,
                                                queue_size=1)
        self.srv_res_lane = rospy.ServiceProxy(
            self.RES_LANE_CONTROL_SERVICE_NAME, Empty)
        self.srv_res_odometry = rospy.ServiceProxy(self.RES_ODOM_SERVICE_NAME,
                                                   Empty)
        self.srv_res_vision = rospy.ServiceProxy(self.RES_VISION_SERVICE_NAME,
                                                 Empty)
        self.srv_res_world = rospy.ServiceProxy(self.RES_GAZEBO_WORLD, Empty)

        self.sub_lane_pilot_state = rospy.Subscriber(
            self.LANE_PILOT_STATE_TOPIC,
            Int8,
            self.lane_pilot_state_callback,
            queue_size=1)
        self.sub_task_state = rospy.Subscriber(self.TASK_STATE_TOPIC,
                                               Int8,
                                               self.task_state_callback,
                                               queue_size=1)
        self.sub_rc_state = rospy.Subscriber(self.RC_STATUS_TOPIC,
                                             Int8,
                                             self.changed_rc_callback,
                                             queue_size=1)
        self.sub_car_drive = rospy.Subscriber(self.DRIVE_COMMAND_TOPIC,
                                              DriveCommand,
                                              self.drive_command_callback,
                                              queue_size=1)

        # Other variables
        self._widget.rc_label.setText(self.RC_MODES[-1])

        self.check_if_running_simulation()
        self._widget.advanced_elements.hide()

        self.redraw_wheels_time = datetime.now()
        self.redraw_timeout = timedelta(milliseconds=250)
        rospy.loginfo("Rqt plugin initialized successfully")

    def press_button1(self):
        rospy.logdebug("Pressed button1 button")
        msg = Buttons(is_pressed_first=True, is_pressed_second=False)
        self.pub_button.publish(msg)

    def press_button2(self):
        rospy.logdebug("Pressed 'button2' button")
        msg = Buttons(is_pressed_first=False, is_pressed_second=True)
        self.pub_button.publish(msg)

    def changed_rc_callback(self, data: Int8):
        RADIO_BUTTONS = [
            self._widget.button_select_manual,
            self._widget.button_select_semi_auto,
            self._widget.button_select_auto
        ]
        self._widget.rc_label.setText(self.RC_MODES[data.data])
        RADIO_BUTTONS[data.data].setChecked(True)

    def restart_lane_control(self):
        rospy.logdebug("Pressed restart lane_control button")
        service_list = rosservice.get_service_list()
        if self.RES_LANE_CONTROL_SERVICE_NAME not in service_list:
            rospy.logwarn(self.RES_LANE_CONTROL_SERVICE_NAME +
                          " service server is not active")
        else:
            self.srv_res_lane()

    def restart_odometry(self):
        rospy.logdebug("Pressed restart odometry button")
        service_list = rosservice.get_service_list()
        if self.RES_ODOM_SERVICE_NAME not in service_list:
            rospy.logwarn(self.RES_ODOM_SERVICE_NAME +
                          " service server is not active")
        else:
            self.srv_res_odometry()

    def restart_vision(self):
        rospy.logdebug("Pressed restart vision button")
        service_list = rosservice.get_service_list()
        if self.RES_VISION_SERVICE_NAME not in service_list:
            rospy.logwarn(self.RES_VISION_SERVICE_NAME +
                          " service server is not active")
        else:
            self.srv_res_vision()

    def restart_simulation(self):
        rospy.logdebug("Pressed restart vision button")
        service_list = rosservice.get_service_list()
        if self.RES_GAZEBO_WORLD not in service_list:
            rospy.logwarn(self.RES_GAZEBO_WORLD +
                          " service server is not active")
        else:
            self.srv_res_world()

    def lane_pilot_state_callback(self, data: Int8):
        self._widget.lane_control_label.setText(self.LANE_MODES[data.data])

    def task_state_callback(self, data: Int8):
        self._widget.task_label.setText(self.TASKS[data.data])

    def shutdown_plugin(self):
        self.pub_button.unregister()

    def drive_command_callback(self, data: DriveCommand):
        if self.redraw_wheels_time > datetime.now():
            return

        print(math.degrees(-data.steering_angle_front))
        self.car_scene.rotate_wheels(
            front_angle=math.degrees(-data.steering_angle_front),
            back_angle=math.degrees(-data.steering_angle_rear))

        print("draw")
        self.redraw_wheels_time = datetime.now() + self.redraw_timeout

    def switch_view_callback(self, state):
        if state:
            self._widget.advanced_elements.show()
        else:
            self._widget.advanced_elements.hide()

    def check_if_running_simulation(self):
        topic_list = rospy.get_published_topics()
        if ['/gazebo/link_states', 'gazebo_msgs/LinkStates'] in topic_list:
            self._widget.button_restart_simulation.setEnabled(True)
        else:
            self._widget.button_restart_simulation.setEnabled(False)

    def change_mode(self, checked):
        RADIO_BUTTON_MODES = {
            self._widget.button_select_manual: 0,
            self._widget.button_select_semi_auto: 1,
            self._widget.button_select_auto: 2
        }
        self.pub_switch_state.publish(
            Int8(RADIO_BUTTON_MODES[self._widget.sender()]))
