import sys
import rospy
from clients.basic_client import BasicClient
#jesli freedrive bedzie w miare okej, to pozostale clienty zostaną zrobione podobnie
from std_msgs.msg import Int8
class StartingProcedureClient(BasicClient):
    try:
        result = rospy.ServiceProxy('serwer_name', Int8)
    except Exception as e:
        pass