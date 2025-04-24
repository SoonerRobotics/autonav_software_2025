#!/usr/bin/env -S python3
#TODO DO NOT COMMIT THIS FILE!! IT SHOULD BE REVERTED ONCE YOU GET WS WORKING
import asyncio
import json
import threading
import time

import cv2
import cv_bridge
import rclpy
import aiohttp.web as web
from autonav_shared.node import Node
from autonav_shared.types import DeviceState as AutonavDeviceState, LogLevel, SystemState
from autonav_msgs.msg import *

from std_msgs.msg import *
from sensor_msgs.msg import *

from enum import Enum


class Topics(Enum):
    # Topic List
    BROADCAST = "autonav/shared/broadcast"  # todo does this work?
    SYSTEM_STATE = "autonav/shared/system"
    DEVICE_STATE = "autonav/shared/device"

    # IMU Data
    IMU = "/autonav/imu"
    AUTONAV_GPS = "/autonav/gps"
    MOTOR_INPUT = "/autonav/MotorInput"
    POSITION = "/autonav/position"
    MOTOR_CONTROLLER_DEBUG = '/autonav/motor_feedback'  # todo implement
    CONTROLLER_INP = '/autonav/controller_input'        # todo implement

    MOTOR_FEEDBACK = "/autonav/MotorFeedback"
    NUC_STATISTICS = "/autonav/statistic"
    ULTRASONICS = "/autonav/ultrasonics"
    CONBUS = "/autonav/conbus"
    SAFETY_LIGHTS = "/autonav/safety_lights"
    PERFORMANCE = "autonav/performance"

    # Raw camera
    RAW_LEFT = "autonav/camera/left"
    RAW_RIGHT = "autonav/camera/right"
    RAW_FRONT = "autonav/camera/front"
    RAW_BACK = "autonav/camera/back"

    # Other Camera Nodes
    COMBINED_IMAGE = "/autonav/vision/combined/filtered"
    FEELERS = "/autonav/feelers/debug"  # todo does this transmit an image?

    # Others
    CONFIGURATION = "/scr/configuration"          # TODO is this still a topic?
    PLAYBACK = "autonav/autonav_playback"         # TODO see how to feed this data in
    AUDIBLE_FEEDBACK = '/autonav/audible_feedback' # todo implement!


async_loop = asyncio.new_event_loop()
bridge = cv_bridge.CvBridge()


class Limiter:
    def __init__(self) -> None:
        self.limits = {}
        self.nextAllowance = {}

    # Sets a limit for how many times per second a key can be used
    def setLimit(self, key, limit):
        self.limits[key] = limit
        self.nextAllowance[key] = 0

    # If it can be used, returns true and decrements the remaining uses
    def use(self, key):
        if key not in self.limits:
            return True
        nextUsageAfter = self.nextAllowance[key]
        if nextUsageAfter == 0 or time.time() >= nextUsageAfter:
            self.nextAllowance[key] = time.time() + (1.0 / self.limits[key])
            return True
        return False


class BroadcastNode(Node):
    def __init__(self):
        super().__init__("autonav_display_broadcast")

        self.port = 8080
        self.host = "0.0.0.0"
        self.send_map = {}
        self.client_map = {}
        self.QOS = 10

        # Rate limits
        self.limiter = Limiter()
        self.limiter.setLimit(Topics.MOTOR_INPUT.value, 2)
        self.limiter.setLimit(Topics.MOTOR_FEEDBACK.value, 5)
        self.limiter.setLimit(Topics.MOTOR_CONTROLLER_DEBUG.value, 1)
        self.limiter.setLimit(Topics.IMU.value, 1)
        self.limiter.setLimit(Topics.AUTONAV_GPS.value, 3)
        self.limiter.setLimit(Topics.POSITION.value, 3)
        self.limiter.setLimit(Topics.RAW_LEFT.value, 0.5)
        self.limiter.setLimit(Topics.RAW_RIGHT.value, 0.5)
        self.limiter.setLimit(Topics.RAW_FRONT.value, 0.5)
        self.limiter.setLimit(Topics.RAW_BACK.value, 0.5)
        self.limiter.setLimit(Topics.COMBINED_IMAGE.value, 0.5)

        # Clients TODO a lot of these dont exist anymore
#        self.system_state_c = self.create_subscription(
#            SystemState, Topics.SYSTEM_STATE.value, self.systemStateCallback, 20
#        )
#        self.system_state_c = self.create_client(
#            SetSystemState, Topics.SYSTEM_STATE.value
#        )
        #self.config_c = self.create_client(UpdateConfig, Topics.CONFIGURATION.value)

        #self.get_presets_c = self.create_client(GetPresets, Topics.GET_PRESETS.value)# TODO THESE Don't EXIST ANYMORE
        #self.set_active_preset_c = self.create_client(
        #   SetActivePreset, Topics.SET_ACTIVE_PRESET.value
        #)
        #self.save_active_preset_c = self.create_client(
        #   SaveActivePreset, Topics.SAVE_ACTIVE_PRESET.value
        #)
        #self.delete_preset_c = self.create_client(DeletePreset, Topics.DELETE_PRESET.value)

        #Publishers
        self.broadcast_p = self.create_publisher(Empty, Topics.BROADCAST.value, 20)

        # Subscriptions
        self.device_state_s = self.create_subscription(
            DeviceState,
            Topics.DEVICE_STATE.value,
            self.deviceStateCallback,
            20,
        )
        self.system_state_s = self.create_subscription(
            SystemState,
            Topics.SYSTEM_STATE.value,
            self.systemStateCallback,
            20,
        )
        self.position_s = self.create_subscription(
            Position,
            Topics.POSITION.value,
            self.positionCallback,
            20,
        )
        self.motor_feedback_s = self.create_subscription(
            MotorFeedback,
            Topics.MOTOR_FEEDBACK.value,
            self.motorFeedbackCallback,
            20,
        )
        self.motor_input_s = self.create_subscription(
            MotorInput,
            Topics.MOTOR_INPUT.value,
            self.motorInputCallback,
            20,
        )
        self.gps_s = self.create_subscription(
            GPSFeedback,
            Topics.AUTONAV_GPS.value,
            self.gpsFeedbackCallback,
            20,
        )
        self.imu_s = self.create_subscription(
            IMUData,
            Topics.IMU.value,
            self.imuDataCallback,
            20,
        )

        # Camera subscriptions
        self.camera_left_s = self.create_subscription(
            CompressedImage,
            Topics.RAW_LEFT.value,
            lambda msg: self.cameraCallback(msg, 'left'),
            self.QOS,
        )
        self.camera_right_s = self.create_subscription(
            CompressedImage,
            Topics.RAW_RIGHT.value,
            lambda msg: self.cameraCallback(msg, 'right'),
            self.QOS,
        )
        self.camera_front_s = self.create_subscription(
            CompressedImage,
            Topics.RAW_FRONT.value,
            lambda msg: self.cameraCallback(msg, 'front'),
            self.QOS,
        )
        self.camera_back_s = self.create_subscription(
            CompressedImage,
            Topics.RAW_BACK.value,
            lambda msg: self.cameraCallback(msg, 'back'),
            self.QOS,
        )
        self.combined = self.create_subscription(
            CompressedImage,
            Topics.COMBINED_IMAGE.value,
            lambda msg: self.cameraCallback(msg, 'combined'),
            self.QOS,
        )
        self.feelers = self.create_subscription(
            CompressedImage,
            Topics.FEELERS.value,
            lambda msg: self.cameraCallback(msg, 'feelers'),
            self.QOS,
        )

        self.loop_thread = threading.Thread(target=self.loopthread)
        self.loop_thread.start()

    def loopthread(self):
        asyncio.set_event_loop(async_loop)
        async_loop.run_until_complete(self.startHttpApp())

    async def startHttpApp(self):
        app = web.Application()
        app.router.add_get(
            "/", self.handler  # WebSocket upgrade on root
        )
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.port)
        await site.start()
        self.get_logger().info(
            "Started webserver on ws://%s:%d" % (self.host, self.port)
        )
        await asyncio.Future()

    def push_image(self, topic, msg):
        if not self.limiter.use(topic):
            return
        cvimg = bridge.compressed_imgmsg_to_cv2(msg)
        _, img = cv2.imencode('.jpg', cvimg)
        self.push_old(json.dumps({
            "op": "data",
            "topic": topic,
            "format": msg.format,
            "data": list(img.tobytes())
        }))

    def push(self, topic, data, unique_id=None):
        if not self.limiter.use(topic):
            return
        packet = {"op": "data", "topic": topic}
        for key in data.get_fields_and_field_types().keys():
            packet[key] = getattr(data, key)
        if not self.send_map:
            return
        message_json = json.dumps(packet)
        if unique_id is None:
            for uid in self.send_map:
                self.send_map[uid].append(message_json)
        else:
            self.send_map[unique_id].append(message_json)

    def push_old(self, message, unique_id=None):
        if not self.send_map:
            return
        if unique_id is None:
            for uid in self.send_map:
                self.send_map[uid].append(message)
        else:
            self.send_map[unique_id].append(message)

    async def handler(self, request):
        # Upgrade HTTP request to WebSocket
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        uid = request.query.get("id")
        if uid is None or uid in self.client_map:
            await ws.close()
            return ws

        self.client_map[uid] = ws
        self.send_map[uid] = []

        async def producer():
            while not ws.closed:
                if self.send_map[uid]:
                    await ws.send_str(self.send_map[uid].pop(0))
                else:
                    await asyncio.sleep(0.01)

        async def consumer():
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    obj = json.loads(msg.data)
                    await self.handle_ws_message(obj, uid)
                elif msg.type == web.WSMsgType.ERROR:
                    break

        p = asyncio.create_task(producer())
        c = asyncio.create_task(consumer())
        done, pending = await asyncio.wait([p, c], return_when=asyncio.FIRST_COMPLETED)
        for task in pending:
            task.cancel()
        del self.client_map[uid]
        del self.send_map[uid]
        await ws.close()
        return ws

    async def handle_ws_message(self, obj, uid):
        # Mirror first-file consumer logic
        if obj.get("op") == "broadcast":
            self.broadcast_p.publish(Empty())
        elif obj.get("op") == "get_nodes":
            nodes = self.get_node_names()
            for i in range(len(nodes)):
                nodes[i] = nodes[i].replace("/", "")
            node_states = {nid: self.device_states.get(nid, 0) for nid in nodes}
            self.push_old(json.dumps({
                "op": "get_nodes_callback",
                "nodes": nodes,
                "states": node_states,
                "system": {
                    "state": self.system_state,
                    #"mode": self.system_mode,
                    "mobility": self.mobility
                }
            }), uid)
        elif obj.get("op") == "set_system_state":
            self.set_system_total_state(int(obj["state"]), int(obj["mode"]), bool(obj["mobility"]))
        # TODO: add configuration, conbus, preset ops if needed

    # ROS callbacks
    def systemStateCallback(self, msg: SystemState):
        self.push(Topics.SYSTEM_STATE.value, msg)

    def deviceStateCallback(self, msg: DeviceState):
        self.push(Topics.DEVICE_STATE.value, msg)

    def positionCallback(self, msg: Position):
        self.push(Topics.POSITION.value, msg)

    def motorInputCallback(self, msg: MotorInput):
        self.push(Topics.MOTOR_INPUT.value, msg)

    def motorFeedbackCallback(self, msg: MotorFeedback):
        self.push(Topics.MOTOR_FEEDBACK.value, msg)

    def imuDataCallback(self, msg: IMUData):
        self.push(Topics.IMU.value, msg)

    def gpsFeedbackCallback(self, msg: GPSFeedback):
        self.push(Topics.AUTONAV_GPS.value, msg)

    # Cameras
    def cameraCallback(self, msg: CompressedImage, key: str):
        topic = getattr(Topics, f"RAW_{key.upper()}" if key in ['left', 'right', 'front', 'back'] else f"FEELERS").value
        self.push_image(topic, msg)

    def init(self):
        self.set_device_state(AutonavDeviceState.OPERATING)


def main():
    rclpy.init()
    node = BroadcastNode()
    Node.run_node(node)

if __name__ == "__main__":
    main()
