from clients.basic_client import BasicClient

# from Shared.custom_msgs.action import driving, intersection, starting

#jesli freedrive bedzie w miare okej, to pozostale clienty zostaną zrobione podobnie
class IntersectionClient(BasicClient):
    def __init__(self, service_name, enum_task) -> None:
        super().__init__(service_name, enum_task)