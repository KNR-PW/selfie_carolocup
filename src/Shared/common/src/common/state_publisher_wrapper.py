from io import BytesIO

import rospy
from std_msgs.msg import Int8

from common.StatePublisherWrapper import StatePublisherWrapper


class StatePublisher:

    def __init__(self, topic_name, pub_freq=None):

        if (isinstance(pub_freq, float)):
            self.state_publisher = StatePublisherWrapper(topic_name, pub_freq)
            return

        self.state_publisher = StatePublisherWrapper(topic_name)

    def _to_cpp(self, msg):
        buf = BytesIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        msg = cls()
        return msg.deserialize(str.encode(str_msg))

    def updateState(self, state):

        if isinstance(state, Int8):
            state_str = self._to_cpp(state)
            self.state_publisher.updateState(state_str)
            return

        if isinstance(state, int):
            if (state >= -128 and state < 128):
                self.state_publisher.updateState(state)
                return

        rospy.ROSException(
            'Argument \'state\' is not a std_msgs/Int8 or int8_t')
