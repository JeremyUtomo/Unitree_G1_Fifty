import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


class VoiceController:
    
    def __init__(self):
        self.audio_client = None
        
    def Init(self):
        try:
            self.audio_client = AudioClient()
            self.audio_client.SetTimeout(10.0)
            self.audio_client.Init()
            self.audio_client.SetVolume(100)
            print("Audio client initialized")
        except Exception as e:
            print(f"Audio init failed: {e}")
            self.audio_client = None
    
    def say(self, text: str):
        if self.audio_client:
            self.audio_client.TtsMaker(text, 1)
        else:
            print(f"Cannot speak (audio unavailable): {text}")
    
    def set_volume(self, volume: int):
        if self.audio_client:
            self.audio_client.SetVolume(volume)
    
    def get_volume(self) -> int:
        if self.audio_client:
            return self.audio_client.GetVolume()
        return 0
    
    def led_control(self, r: int, g: int, b: int):
        if self.audio_client:
            self.audio_client.LedControl(r, g, b)
    
    def introduce(self):
        self.say("6 7")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        print(f"Example: python3 {sys.argv[0]} eth0")
        sys.exit(-1)

    ChannelFactoryInitialize(0, sys.argv[1])
    
    print("Voice Input - Interactive TTS Mode")
    print("For wireless controller TTS, use run_wireless_controller.py instead")

    voice = VoiceController()
    voice.Init()
    
    print("\nRunning LED demo...")
    print("   LED: Red")
    voice.led_control(255, 0, 0)
    time.sleep(1)
    print("   LED: Green")
    voice.led_control(0, 255, 0)
    time.sleep(1)
    print("   LED: Blue")
    voice.led_control(0, 0, 255)
    time.sleep(1)
    
    print("\nReady for interactive TTS")

    try:
        while True:
            try:
                user_text = input("\nEnter text for TTS (Ctrl+C to exit): ")
            except EOFError:
                print("\nInput closed, exiting.")
                break

            if not user_text.strip():
                continue

            voice.say(user_text)
    except KeyboardInterrupt:
        sys.exit(0)

