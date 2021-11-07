from clients.basic_client import BasicClient
import actionlib


#jesli freedrive bedzie w miare okej, to pozostale clienty zostaną zrobione podobnie

class ParkClient(BasicClient):
    def __init__(self, service_name) -> None:
        super().__init__(service_name)


    