from turtle import speed
import rospy
import rospkg
import time
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image
from custom_msgs.msg import Motion
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import datetime


rospack = rospkg.RosPack()
bridge = CvBridge()

COLORS = [(0, 255, 0), (0, 0, 255), (255, 0, 0), 
          (255, 255, 0), (255, 0, 255), (0, 255, 255)]

motion_msg = None
Conf_threshold = 0.45
NMS_threshold = 0.2
frame_counter = 0

last_time_seen_overtaking_ban = None
last_time_seen_speed_limit = None
overtaking_ban_start_distance = None
speed_limit_start_distance = None

SPEED_LIMIT = 0
OVERTAKING_BAN = 1

starting_time = time.time()
pkg_path = rospack.get_path('sign_detection')+'/data/'

visualize_detection = rospy.get_param('visualize_sign_detection', True)
speed_limit_distance = rospy.get_param('speed_limit_distance', 2.0)
overtaking_ban_distance = rospy.get_param('overtaking_ban_distance', 2.0)

class_name = []
with open(pkg_path +'classes.txt', 'r') as f:
    class_name = [cname.strip() for cname in f.readlines()]

net = cv.dnn.readNet(pkg_path +'znaki.weights', pkg_path +'znaki.cfg')
model = cv.dnn_DetectionModel(net)
model.setInputParams(size=(1600, 416), scale=1/255, swapRB=True)

can_overtake = True
speed_limit = False


def detect_signs(frame):
    global frame_counter
    frame_counter += 1
    frame = frame[:,:,1]
    frame = np.expand_dims(frame, 2)
    classes, scores, boxes = model.detect(frame, Conf_threshold, NMS_threshold)
    return classes, scores, boxes

def visualize(frame, classes, scores, boxes):
    for (classid, score, box) in zip(classes, scores, boxes):
        color = COLORS[int(classid) % len(COLORS)]
        label = "%s : %f" % (class_name[classid[0]], score)
        cv.rectangle(frame, box, color, 2)
        cv.putText(frame, label, (box[0], box[1]-10), cv.FONT_HERSHEY_COMPLEX, 0.3, color, 1)
    endingTime = time.time() - starting_time
    fps = frame_counter/endingTime
    cv.putText(frame, f'FPS: {fps}', (20, 50),
               cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
    cv.putText(frame, f"Can overtake: {can_overtake}", (20,80),
               cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
    cv.putText(frame, f"Speed limit: {speed_limit}", (20,110),
               cv.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
    cv.imshow('frame', frame)
    key = cv.waitKey(1)
    if key == ord('q'):
        return

def is_sign(classes):
    global last_time_seen_overtaking_ban
    global last_time_seen_speed_limit
    global speed_limit_start_distance
    global overtaking_ban_start_distance
    global can_overtake
    global speed_limit

    if OVERTAKING_BAN in classes:
        last_time_seen_overtaking_ban = datetime.datetime.now()
        can_overtake = True

    if SPEED_LIMIT in classes:
        last_time_seen_speed_limit = datetime.datetime.now()
        speed_limit = False

    if last_time_seen_overtaking_ban is None:
        can_overtake = True
    elif can_overtake and datetime.datetime.now() - last_time_seen_overtaking_ban > datetime.timedelta(seconds=1):
        can_overtake = False
        overtaking_ban_start_distance = motion_msg.distance

    if motion_msg.distance > overtaking_ban_start_distance + overtaking_ban_distance:
        can_overtake = True
        overtaking_ban_start_distance = None

    if last_time_seen_speed_limit:
        print(datetime.datetime.now() - last_time_seen_speed_limit)
    if last_time_seen_speed_limit is None:
        speed_limit = False
    elif not speed_limit and datetime.datetime.now() - last_time_seen_speed_limit > datetime.timedelta(seconds=1):
        speed_limit = True
        speed_limit_start_distance = motion_msg.distance

    if motion_msg.distance > speed_limit_start_distance + speed_limit_distance:
        speed_limit = False
        speed_limit_start_distance = None

def image_callback(msg):
    try:
        img = bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = img[184:600,:,:]
    except CvBridgeError as e:
        print(e)
    else:
        classes, scores, boxes = detect_signs(img)

    if visualize_detection and img is not None:
        visualize(img, classes, scores, boxes)

    is_sign(classes)
    pub_overtake.publish(can_overtake)
    pub_speed.publish(speed_limit)

def motion_callback(msg):
    global motion_msg
    motion_msg = msg



rospy.init_node('sign_detector')
image_topic = "/camera_basler/image_rect"
motion_topic = "/selfie_out/motion"
rospy.Subscriber(image_topic, Image, image_callback)
rospy.Subscriber(motion_topic, Motion, motion_callback)
pub_overtake = rospy.Publisher('can_overtake', Bool, queue_size=10)
pub_speed = rospy.Publisher('speed_limit', Bool, queue_size=10)
rospy.spin()

