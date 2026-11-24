"""EVP-12 -> smart-traffic-lab bridge.

Reads JSON lines from the intersection controller (RS-485 via a USB adapter) and hands
preemption requests to your signal controller. Sends commands back (inhibit, test).

    pip install pyserial
    python evp_bridge.py --port /dev/ttyUSB0
"""
import argparse
import json
import time

import serial  # pyserial


class EvpBridge:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.active = None          # dict of the current request, or None
        self.last_hb = 0.0

    def send(self, obj: dict) -> None:
        self.ser.write((json.dumps(obj) + "\n").encode())

    def inhibit(self, on: bool) -> None:
        """Call while the controller is in flash / manual mode."""
        self.send({"cmd": "inhibit", "on": on})

    def test_relay(self, approach: str) -> None:
        self.send({"cmd": "test", "app": approach})

    # ---- hook these into the smart-traffic-lab controller --------------------------
    def on_request(self, msg: dict) -> None:
        print(f"PREEMPT approach {msg['app']} vid={msg['vid']} via {msg['via']} eta={msg['eta_s']}s")

    def on_release(self, msg: dict) -> None:
        print(f"RELEASE approach {msg['app']} ({msg.get('why')})")

    def on_pre(self, msg: dict) -> None:
        print(f"PRE-ALERT approach {msg['app']} eta={msg['eta_s']}s dist={msg['dist_m']}m")

    def on_fault(self, msg: dict) -> None:
        print("FAULT", msg)

    # ---------------------------------------------------------------------------------
    def poll(self) -> None:
        line = self.ser.readline().decode(errors="ignore").strip()
        if not line.startswith("{"):
            return
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return
        t = msg.get("type")
        if t == "REQ_ON":
            self.active = msg
            self.on_request(msg)
        elif t == "REQ_OFF":
            self.active = None
            self.on_release(msg)
        elif t == "PRE":
            self.on_pre(msg)
        elif t in ("HEAD_FAULT", "DIR_CONFLICT", "BAD_MAC", "REPLAY"):
            self.on_fault(msg)
        elif t == "HB":
            self.last_hb = time.time()

    def healthy(self) -> bool:
        return time.time() - self.last_hb < 3.0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    b = EvpBridge(args.port, args.baud)
    while True:
        b.poll()
        if not b.healthy() and b.last_hb:
            print("EVP controller silent > 3 s: treat as no preemption")
            b.last_hb = 0
