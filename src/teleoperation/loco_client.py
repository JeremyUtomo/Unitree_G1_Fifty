import time
import sys
from typing import Optional
from dataclasses import dataclass

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco.g1_loco_api import (
    ROBOT_API_ID_LOCO_GET_FSM_ID,
    ROBOT_API_ID_LOCO_GET_FSM_MODE,
)

try:
    from hanger_boot_sequence import hanger_boot_sequence
    HANGER_BOOT_AVAILABLE = True
except ImportError:
    HANGER_BOOT_AVAILABLE = False


def _rpc_get_int(client: LocoClient, api_id: int) -> Optional[int]:
    """Helper to get integer value from RPC call."""
    try:
        code, data = client._Call(api_id, "{}")  # type: ignore[attr-defined]
        if code == 0 and data:
            import json
            return json.loads(data).get("data")
    except Exception:
        pass
    return None


def get_fsm_id(client: LocoClient) -> Optional[int]:
    """Get current FSM ID from robot."""
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_ID)


def get_fsm_mode(client: LocoClient) -> Optional[int]:
    """Get current FSM mode from robot."""
    return _rpc_get_int(client, ROBOT_API_ID_LOCO_GET_FSM_MODE)

@dataclass
class Command:
    """Robot movement command."""
    name: str
    id: int
    description: str = ""


# Available movement commands
COMMANDS = [
    Command(name="stop", id=0, description="Stop/Damp"),
    Command(name="w", id=1, description="Move forward"),
    Command(name="s", id=2, description="Move backward"),
    Command(name="a", id=3, description="Side step left"),
    Command(name="d", id=4, description="Side step right"),
    Command(name="q", id=5, description="Rotate left"),
    Command(name="e", id=6, description="Rotate right"),
    Command(name="b", id=7, description="Balance mode"),
]


class LocoInterface:
    """Interactive terminal interface for locomotion control."""
    
    def __init__(self, client: LocoClient):
        self.client = client
        self.current_command: Optional[Command] = None

    def show_commands(self):
        """Display all available commands."""
        print("\n" + "="*60)
        print("Available Commands:")
        print("="*60)
        for cmd in COMMANDS:
            print(f"  {cmd.name:8s} (id: {cmd.id}) - {cmd.description}")
        print("="*60 + "\n")

    def get_command(self) -> Optional[Command]:
        """Get command from user input."""
        input_str = input("Enter command (or 'list' for options): ").strip().lower()

        if input_str == "list":
            self.show_commands()
            return None

        if input_str == "exit" or input_str == "quit":
            return Command(name="exit", id=-1, description="Exit program")

        # Try to match by name or ID
        try:
            input_id = int(input_str)
            for cmd in COMMANDS:
                if cmd.id == input_id:
                    return cmd
        except ValueError:
            for cmd in COMMANDS:
                if cmd.name == input_str:
                    return cmd

        print(f"⚠️  Unknown command: '{input_str}'")
        return None

    def execute_command(self, cmd: Command):
        """Execute a locomotion command."""
        if cmd.id == 0:
            self.client.Damp()
            print("🛑 Stop/Damp")
        elif cmd.id == 1:
            self.client.Move(1, 0, 0)
            print("⬆️  Moving forward")
        elif cmd.id == 2:
            self.client.Move(-1, 0, 0)
            print("⬇️  Moving backward")
        elif cmd.id == 3:
            self.client.Move(0, 0.3, 0)
            print("⬅️  Side step left")
        elif cmd.id == 4:
            self.client.Move(0, -0.3, 0)
            print("➡️  Side step right")
        elif cmd.id == 5:
            self.client.Move(0, 0, 0.5)
            print("↺  Rotating left")
        elif cmd.id == 6:
            self.client.Move(0, 0, -0.5)
            print("↻  Rotating right")
        elif cmd.id == 7:
            if HANGER_BOOT_AVAILABLE:
                print("🤸 Starting balance mode sequence...")
                hanger_boot_sequence(iface=sys.argv[1])
            else:
                print("⚠️  Hanger boot sequence not available")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} <network_interface>")
        print(f"Example: python3 {sys.argv[0]} eth0")
        sys.exit(-1)

    print("="*80)
    print("🤖 Unitree G1 - Locomotion Control")
    print("="*80)
    print("⚠️  WARNING: Ensure no obstacles around the robot!")
    print("="*80)
    input("\nPress Enter to continue...")

    ChannelFactoryInitialize(0, sys.argv[1])

    # Initialize loco client
    client = LocoClient()
    client.SetTimeout(10.0)
    client.Init()
    
    print("✅ Loco client initialized")
    
    # Create interface
    interface = LocoInterface(client)
    interface.show_commands()

    # Main control loop
    try:
        while True:
            # Show current FSM state
            fsm_id = get_fsm_id(client)
            if fsm_id is not None:
                print(f"📊 FSM ID: {fsm_id}")
            
            # Get command from user
            cmd = interface.get_command()
            
            if cmd is None:
                continue
            
            if cmd.id == -1:  # Exit command
                print("\n👋 Goodbye!")
                break
            
            # Execute command
            interface.execute_command(cmd)
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping...")
        print("👋 Goodbye!")