import struct
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_


class WirelessController:
    """Subscribe to lowstate and expose simple callbacks for wireless events.

    Callbacks:
      - on_start_cb(): called when F1 + A is pressed (rising edge)
      - on_next_cb(): called when F1 + A pressed the second time when already at stage 1
      - on_stop_cb(): called when F1 + X (rising edge)
      - on_say_hello_cb(): called when F1 + Y (rising edge)

    Usage:
      wc = WirelessController(lowstate_topic="rt/lf/lowstate")
      wc.on_start = my_callback
      wc.Init()

    The controller uses rising-edge detection so each press triggers once.
    """

    def __init__(self, lowstate_topic: str = "rt/lf/lowstate"):
        self.lowstate_topic = lowstate_topic
        self._subscriber = None

        # callbacks
        self.on_start = None
        self.on_next = None
        self.on_stop = None
        self.on_say_hello = None

        # rising-edge guards
        self._f1_a_pressed = False
        self._f1_x_pressed = False
        self._f1_y_pressed = False

    class _Remote:
        def __init__(self):
            self.Lx = 0.0
            self.Rx = 0.0
            self.Ry = 0.0
            self.Ly = 0.0
            self.L1 = 0
            self.L2 = 0
            self.R1 = 0
            self.R2 = 0
            self.A = 0
            self.B = 0
            self.X = 0
            self.Y = 0
            self.Up = 0
            self.Down = 0
            self.Left = 0
            self.Right = 0
            self.Select = 0
            self.F1 = 0
            self.F3 = 0
            self.Start = 0

        def parse_botton(self, data1, data2):
            self.R1 = (data1 >> 0) & 1
            self.L1 = (data1 >> 1) & 1
            self.Start = (data1 >> 2) & 1
            self.Select = (data1 >> 3) & 1
            self.R2 = (data1 >> 4) & 1
            self.L2 = (data1 >> 5) & 1
            self.F1 = (data1 >> 6) & 1
            self.F3 = (data1 >> 7) & 1
            self.A = (data2 >> 0) & 1
            self.B = (data2 >> 1) & 1
            self.X = (data2 >> 2) & 1
            self.Y = (data2 >> 3) & 1
            self.Up = (data2 >> 4) & 1
            self.Right = (data2 >> 5) & 1
            self.Down = (data2 >> 6) & 1
            self.Left = (data2 >> 7) & 1

        def parse_key(self, data):
            lx_offset = 4
            self.Lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
            rx_offset = 8
            self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
            ry_offset = 12
            self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
            ly_offset = 20
            self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]

        def parse(self, remoteData):
            self.parse_key(remoteData)
            self.parse_botton(remoteData[2], remoteData[3])

    def Init(self, callback_queue_depth: int = 10):
        self._remote = WirelessController._Remote()
        self._subscriber = ChannelSubscriber(self.lowstate_topic, LowState_)
        self._subscriber.Init(self._lowstate_handler, callback_queue_depth)

    def _lowstate_handler(self, msg: LowState_):
        try:
            data = msg.wireless_remote
            self._remote.parse(data)

            # F1 + A behavior
            if self._remote.F1 == 1 and self._remote.A == 1:
                if not self._f1_a_pressed:
                    self._f1_a_pressed = True
                    if self.on_start:
                        self.on_start()
                else:
                    # if already pressed and a second press maps to next
                    if self.on_next:
                        self.on_next()
            else:
                self._f1_a_pressed = False

            # F1 + X -> stop
            if self._remote.F1 == 1 and self._remote.X == 1:
                if not self._f1_x_pressed:
                    self._f1_x_pressed = True
                    if self.on_stop:
                        self.on_stop()
            else:
                self._f1_x_pressed = False

            # F1 + Y -> say hello
            if self._remote.F1 == 1 and self._remote.Y == 1:
                if not self._f1_y_pressed:
                    self._f1_y_pressed = True
                    if self.on_say_hello:
                        self.on_say_hello()
            else:
                self._f1_y_pressed = False

        except Exception:
            # keep subscriber alive on parse errors
            print("Warning: wireless lowstate handler exception")
