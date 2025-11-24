"""
Voice input module for TTS control.

Can be used as:
1. Standalone script with interactive TTS input loop
2. Imported module providing VoiceController class for other scripts
"""

import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient


class VoiceController:
    """Voice and audio control wrapper."""
    
    def __init__(self):
        self.audio_client = None
        
    def Init(self):
        """Initialize audio client."""
        try:
            self.audio_client = AudioClient()
            self.audio_client.SetTimeout(10.0)
            self.audio_client.Init()
            self.audio_client.SetVolume(85)
            print("✅ Audio client initialized")
        except Exception as e:
            print(f"⚠️  Audio init failed: {e}")
            self.audio_client = None
    
    def say(self, text: str):
        """Text-to-speech."""
        if self.audio_client:
            self.audio_client.TtsMaker(text, 1)
        else:
            print(f"⚠️  Cannot speak (audio unavailable): {text}")
    
    def set_volume(self, volume: int):
        """Set speaker volume (0-100)."""
        if self.audio_client:
            self.audio_client.SetVolume(volume)
    
    def get_volume(self) -> int:
        """Get current volume."""
        if self.audio_client:
            return self.audio_client.GetVolume()
        return 0
    
    def led_control(self, r: int, g: int, b: int):
        """Control LED colors (0-255)."""
        if self.audio_client:
            self.audio_client.LedControl(r, g, b)
    
    def introduce(self):
        """Say introduction message."""
        self.say("6 7")


if __name__ == "__main__":
    # Standalone mode: interactive TTS input loop
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} networkInterface")
        print(f"Example: python3 {sys.argv[0]} eth0")
        sys.exit(-1)

    ChannelFactoryInitialize(0, sys.argv[1])
    
    print("="*80)
    print("🎤 Voice Input - Interactive TTS Mode")
    print("="*80)
    print("💡 For wireless controller TTS, use run_wireless_controller.py instead")
    print("="*80)

    voice = VoiceController()
    voice.Init()
    
    # LED demo sequence
    print("\n🎬 Running LED demo...")
    print("   LED: Red")
    voice.led_control(255, 0, 0)
    time.sleep(1)
    print("   LED: Green")
    voice.led_control(0, 255, 0)
    time.sleep(1)
    print("   LED: Blue")
    voice.led_control(0, 0, 255)
    time.sleep(1)
    
    print("\n✅ Ready for interactive TTS")
    print("="*80)

    # Interactive loop: ask user for TTS input repeatedly
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
        print("\n\n👋 Goodbye!")
        sys.exit(0)

