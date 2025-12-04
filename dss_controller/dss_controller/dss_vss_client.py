import socket
import threading
import time
import json
from typing import Callable, Optional, Dict, List
import nats
from nats.aio.msg import Msg
import asyncio
from dss_controller import dss_pb2

class DSSVssClient:
    _instance = None

    @staticmethod
    def singleton():
        if DSSVssClient._instance is None:
            DSSVssClient._instance = DSSVssClient()
        return DSSVssClient._instance

    def __init__(self):
        # asyncio loop를 background thread에서 실행
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self._run_loop, daemon=True).start()

        self.nc: Optional[nats.NATS] = None
        self.udp_socket = None
        self.drive_addr = None
        self.subscriptions: List[int] = []

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    # -------------------------------------------------------
    # START  (동기 함수)
    # -------------------------------------------------------
    def start(self, ip="127.0.0.1", drive_port=8886, vss_port=4222):

        # NATS async connect → background loop에서 실행
        async def _async_connect():
            url = f"nats://{ip}:{vss_port}"
            self.nc = await nats.connect(url)
            print(f"[DSSVssClient] Connected to NATS {url}")

        if self.nc is None:
            asyncio.run_coroutine_threadsafe(_async_connect(), self.loop)

        # UDP sync init
        if self.udp_socket is None:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.drive_addr = (ip, drive_port)
            print(f"[DSSVssClient] UDP Control Ready: {ip}:{drive_port}")

    # -------------------------------------------------------
    # STOP
    # -------------------------------------------------------
    def stop(self):
        if self.nc:
            f = self.nc.close()
            asyncio.run_coroutine_threadsafe(f, self.loop)

        if self.udp_socket:
            self.udp_socket.close()
            self.udp_socket = None

        print("[DSSVssClient] stopped")

    # -------------------------------------------------------
    # Drive Control (UDP)
    # -------------------------------------------------------
    def set_drive_control(self, throttle: float, steer: float, brake: float):
        if not self.udp_socket:
            print("[DSSVssClient] UDP not initialized")
            return

        ctrl = dss_pb2.DssSetControl()
        ctrl.identifier = "DSS.ROSBridge.VSSClient"
        ctrl.timestamp = int(time.time() * 1000)
        ctrl.throttle = throttle
        ctrl.brake = brake
        ctrl.steer = steer

        self.udp_socket.sendto(ctrl.SerializeToString(), self.drive_addr)
        
    def get(self, vss_name: str):
        return asyncio.run_coroutine_threadsafe(
            self.request_async("VSS.Get", {"VSSName": vss_name}),
            self.loop
        )

    def set(self, vss_name: str, value: str):
        print(f"[DSSVssClient] set called: {vss_name} -> {value}")
        return asyncio.run_coroutine_threadsafe(
            self.request_async("VSS.Set", {"VSSName": vss_name, "Value": value}),
            self.loop
        )

        

    # -------------------------------------------------------
    # Async Request wrappers
    # -------------------------------------------------------
    async def request_async(self, subject: str, payload: dict, timeout_ms=1000):
        if self.nc is None:
            return None, "NO_CONNECTION"

        data = json.dumps(payload).encode()
        try:
            msg: Msg = await self.nc.request(subject, data, timeout=timeout_ms/1000.0)
            return msg.data.decode(), "OK"
        except Exception as e:
            return None, str(e)

    # -------------------------------------------------------
    # SUBSCRIBE (byte or str)
    # -------------------------------------------------------
    async def _subscribe_async(self, subject, callback):
        async def handler(msg: Msg):
            callback(msg.subject, msg.data.decode())

        sid = await self.nc.subscribe(subject, cb=handler)
        self.subscriptions.append(sid)

    def subscribe(self, subject: str, callback):
        asyncio.run_coroutine_threadsafe(
            self._subscribe_async(subject, callback),
            self.loop
        )
