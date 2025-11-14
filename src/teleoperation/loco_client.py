import time
import sys
from typing import Optional

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from dataclasses import dataclass
from hanger_boot_sequence import hanger_boot_sequence
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

def _rpc_get_int(client: LocoClient, api_id: int) -> Optional[int]:
    try:
        code, data = client._Call(api_id, "{}")  # type: ignore[attr-defined]
        if code == 0 and data:
            import json

            return json.loads(data).get("data")
    except Exception:
        pass
    return None


def _fsm_id(client: LocoClient) -> Optional[int]:
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_ID)


def _fsm_mode(client: LocoClient) -> Optional[int]:
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_MODE)

@dataclass
class TestOption:
    name: str
    id: int

option_list = [ 
    TestOption(name=" ", id=0), #damp
    TestOption(name="w", id=1), #forwards
    TestOption(name="s", id=2), #backwards
    TestOption(name="a", id=3), #left side step
    TestOption(name="d", id=4), #right side step
    TestOption(name="q", id=5), #left rotate
    TestOption(name="e", id=6), #right rotate
    TestOption(name="b", id=7), #go into balance
    TestOption(name="squat", id=8),
]

class UserInterface:
    def __init__(self):
        self.test_option_ = None

    def convert_to_int(self, input_str):
        try:
            return int(input_str)
        except ValueError:
            return None

    def terminal_handle(self):
        input_str = input("Enter id or name: \n")

        if input_str == "list":
            self.test_option_.name = None
            self.test_option_.id = None
            for option in option_list:
                print(f"{option.name}, id: {option.id}")
            return

        for option in option_list:
            if input_str == option.name or self.convert_to_int(input_str) == option.id:
                self.test_option_.name = option.name
                self.test_option_.id = option.id
                print(f"Test: {self.test_option_.name}, test_id: {self.test_option_.id}")
                return

        print("No matching test option found.")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        sys.exit(-1)

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    ChannelFactoryInitialize(0, sys.argv[1])

    test_option = TestOption(name=None, id=None) 
    user_interface = UserInterface()
    user_interface.test_option_ = test_option

    sport_client = LocoClient()  
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    print("Input \"list\" to list all test option ...")
    while True:
        print("fsm ID: ", _fsm_id(sport_client))
        user_interface.terminal_handle()

        print(f"Updated Test Option: Name = {test_option.name}, ID = {test_option.id}")

        if test_option.id == 0:
            sport_client.Damp()
        elif test_option.id == 1:
            sport_client.Move(1,0,0)
        elif test_option.id == 2:
            sport_client.Move(-1,0,0)
        elif test_option.id == 3:
            sport_client.Move(0,0.3,0)
        elif test_option.id == 4:
            sport_client.Move(0,-0.3,0)
        elif test_option.id == 5:
            sport_client.Move(0,0,0.5)
        elif test_option.id == 6:
            sport_client.Move(0,0,-0.5)
        elif test_option.id == 7:
            hanger_boot_sequence(iface = sys.argv[0])
        elif test_option.id == 8:
            sport_client.Squat2StandUp()

        time.sleep(1)