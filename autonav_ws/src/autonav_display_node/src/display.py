#!/usr/bin/env -S python3
import asyncio
import json
import threading
import time
import os
import glob

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

class Topics(Enum):# todo refactor to json file being read in..
    # Topic List
    BROADCAST = "/autonav/shared/autonav_display_broadcast"
    SYSTEM_STATE = "/autonav/shared/system"
    DEVICE_STATE = "/autonav/shared/device"
    LOG = "/autonav/shared/log"

    # IMU Data
    IMU = "/autonav/imu"
    AUTONAV_GPS = "/autonav/gps"
    MOTOR_INPUT = "/autonav/motor_input"
    POSITION = "/autonav/position"
    CONTROLLER_INPUT = '/autonav/controller_input'

    # Motor and System Feedback
    MOTOR_FEEDBACK = "/autonav/motor_feedback"
    NUC_STATISTICS = "/autonav/statistics"
    ULTRASONICS = "/autonav/ultrasonic"
    CONBUS_DATA = "/autonav/conbus/data"
    CONBUS_INSTRUCTION = "/autonav/conbus/instruction"

    # Aliases for backward compatibility
    CONBUS = "/autonav/conbus/data"
    SAFETY_LIGHTS = "/autonav/safety_lights"
    PERFORMANCE = "/autonav/performance"

    # PID and Motor Statistics
    LINEAR_PID_STATISTICS = "/autonav/linear_pid_statistics"
    ANGULAR_PID_STATISTICS = "/autonav/angular_pid_statistics"
    MOTOR_STATISTICS_FRONT = "/autonav/motor_statistics_front_motors"
    MOTOR_STATISTICS_BACK = "/autonav/motor_statistics_back_motors"
    CAN_STATS = "/autonav/can_stats"
    ZERO_ENCODERS = "/autonav/zero_encoders"

    # Raw camera
    RAW_LEFT = "/autonav/camera/left"
    RAW_RIGHT = "/autonav/camera/right"
    RAW_FRONT = "/autonav/camera/front"
    RAW_BACK = "/autonav/camera/back"

    # Other Camera Nodes
    COMBINED_IMAGE = "/autonav/vision/combined/filtered"
    FEELERS = "/autonav/feelers/debug"  

    # Configuration2
    CONFIGURATION_BROADCAST = "/autonav/shared/config/requests"
    CONFIGURATION_UPDATE = "/autonav/shared/config/updates" 
    CONFIG_PRESTS_LOAD = "/autonav/presets/load"
    CONFIG_PRESTS_SAVE = "/autonav/presets/save"

    # Others
    PLAYBACK = "/autonav/autonav_playback"  # 
    AUDIBLE_FEEDBACK = '/autonav/audible_feedback'  


async_loop = asyncio.new_event_loop()
bridge = cv_bridge.CvBridge()


class Limiter:
    def __init__(self) -> None:
        self.limits = {}
        self.nextAllowance = {}

    # Lim 4 hw mny times /s  key used
    def setLimit(self, key, limit):
        self.limits[key] = limit
        self.nextAllowance[key] = 0

    # used? rtrn T & remaining_use--
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
        self.limiter.setLimit(Topics.IMU.value, 1)
        self.limiter.setLimit(Topics.AUTONAV_GPS.value, 3)
        self.limiter.setLimit(Topics.POSITION.value, 3)

        self.limiter.setLimit(Topics.RAW_LEFT.value, 0.5)
        self.limiter.setLimit(Topics.RAW_RIGHT.value, 0.5)
        self.limiter.setLimit(Topics.RAW_FRONT.value, 0.5)
        self.limiter.setLimit(Topics.RAW_BACK.value, 0.5)
        self.limiter.setLimit(Topics.COMBINED_IMAGE.value, 0.5)

        self.limiter.setLimit(Topics.FEELERS.value, 0.5)
        self.limiter.setLimit(Topics.DEVICE_STATE.value, 1)
        self.limiter.setLimit(Topics.SYSTEM_STATE.value, 1)
        self.limiter.setLimit(Topics.NUC_STATISTICS.value, 1)
        self.limiter.setLimit(Topics.ULTRASONICS.value, 1)
        self.limiter.setLimit(Topics.CONBUS_DATA.value, 1)
        self.limiter.setLimit(Topics.SAFETY_LIGHTS.value, 1)
        self.limiter.setLimit(Topics.PLAYBACK.value, 1)
        self.limiter.setLimit(Topics.AUDIBLE_FEEDBACK.value, 1)
        self.limiter.setLimit(Topics.PERFORMANCE.value, 1)

        # New topics with limiters
        self.limiter.setLimit(Topics.LOG.value, 1)
        self.limiter.setLimit(Topics.CONTROLLER_INPUT.value, 1)
        self.limiter.setLimit(Topics.CONBUS_INSTRUCTION.value, 1)
        self.limiter.setLimit(Topics.LINEAR_PID_STATISTICS.value, 1)
        self.limiter.setLimit(Topics.ANGULAR_PID_STATISTICS.value, 1)
        self.limiter.setLimit(Topics.MOTOR_STATISTICS_FRONT.value, 1)
        self.limiter.setLimit(Topics.MOTOR_STATISTICS_BACK.value, 1)
        self.limiter.setLimit(Topics.CAN_STATS.value, 1)
        self.limiter.setLimit(Topics.ZERO_ENCODERS.value, 1)
        self.limiter.setLimit(Topics.CONFIGURATION_BROADCAST.value, 1)

        # Clients todo none of the clients work rn...
        # self.system_state_c = self.create_subscription(SystemState, Topics.SYSTEM_STATE, self.systemStateCallback, 20)
        # self.config_c = self.create_client(ConfigurationUpdate, Topics.CONFIGURATION_UPDATE)
        # self.get_presets_c = self.create_client(GetPresets, Topics.CONFIG_PRESTS_LOAD)
        # self.save_active_preset_c = self.create_client(SaveActivePreset, Topics.CONFIG_PRESTS_SAVE)
        # self.delete_preset_c = self.create_client(DeletePreset, "/scr/delete_preset")
        # self.set_active_preset_c = self.create_client(SetActivePreset, "/scr/set_active_preset")

       #  Publishers
        self.broadcast_p = self.create_publisher(Empty, Topics.BROADCAST.value, 20)
       #
       #  Subscriptions
       #
       #  Topic List

        # self.broadcast = self.create_subscription( todo delete pretty sure this doesn't exist as it wasn't in the original build
        #     SystemState,
        #     Topics.BROADCAST.value,
        #     self.systemStateCallback,
        #     20,
        # )

        # self.system_state_s = self.create_subscription( todo delete, same reason as method above
        #     SystemState,
        #     Topics.SYSTEM_STATE.value,
        #     self.systemStateCallback,
        #     20,
        # )

        self.device_state_s = self.create_subscription(
            DeviceState,
            Topics.DEVICE_STATE.value,
            self.deviceStateCallback,
            20,
        )
        # IMU DATA
        self.imu_s = self.create_subscription(
            IMUFeedback,
            Topics.IMU.value,
            self.imuDataCallback,
            20,
        )
        self.gps_s = self.create_subscription(
            GPSFeedback,
            Topics.AUTONAV_GPS.value,
            self.gpsFeedbackCallback,
            20,
        )
        self.motor_input_s = self.create_subscription(
            MotorInput,
            Topics.MOTOR_INPUT.value,
            self.motorInputCallback,
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
        self.NUC_statistic = self.create_subscription(
            NUCStatistics,
            Topics.NUC_STATISTICS.value,
            self.statisticCallback,
            20
        )

        self.ultrasonic = self.create_subscription(
            Ultrasonic,
            Topics.ULTRASONICS.value,
            self.ultrasonicsCallback,
            20
        )
        self.conbus = self.create_subscription(
            Conbus,
            Topics.CONBUS_DATA.value,
            self.conbusCallback,
            20
        )
        self.SafetyLights = self.create_subscription(
            SafetyLights,
            Topics.SAFETY_LIGHTS.value,
            self.safteyLightsCallback,
            20
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

        # New topic subscriptions
        self.log_s = self.create_subscription(
            Log,
            Topics.LOG.value,
            self.logCallback,
            20
        )

        self.controller_input_s = self.create_subscription(
            ControllerInput,
            Topics.CONTROLLER_INPUT.value,
            self.controllerInputCallback,
            20
        )

        self.conbus_instruction_s = self.create_subscription(
            Conbus,
            Topics.CONBUS_INSTRUCTION.value,
            self.conbusInstructionCallback,
            20
        )

        self.linear_pid_statistics_s = self.create_subscription(
            LinearPIDStatistics,
            Topics.LINEAR_PID_STATISTICS.value,
            self.linearPIDStatisticsCallback,
            20
        )

        self.angular_pid_statistics_s = self.create_subscription(
            AngularPIDStatistics,
            Topics.ANGULAR_PID_STATISTICS.value,
            self.angularPIDStatisticsCallback,
            20
        )

        self.motor_statistics_front_s = self.create_subscription(
            MotorStatistics,
            Topics.MOTOR_STATISTICS_FRONT.value,
            self.motorStatisticsFrontCallback,
            20
        )

        self.motor_statistics_back_s = self.create_subscription(
            MotorStatistics,
            Topics.MOTOR_STATISTICS_BACK.value,
            self.motorStatisticsBackCallback,
            20
        )

        self.can_stats_s = self.create_subscription(
            CanStats,
            Topics.CAN_STATS.value,
            self.canStatsCallback,
            20
        )

        self.zero_encoders_s = self.create_subscription(
            ZeroEncoders,
            Topics.ZERO_ENCODERS.value,
            self.zeroEncodersCallback,
            20
        )

        self.configuration_broadcast_s = self.create_subscription(
            ConfigurationBroadcast,
            Topics.CONFIGURATION_BROADCAST.value,
            self.configurationBroadcastCallback,
            20
        )

        self.loop_thread = threading.Thread(target=self.loopthread)
        self.loop_thread.start()

    def loopthread(self):
        asyncio.set_event_loop(async_loop)
        async_loop.run_until_complete(self.startHttpApp())

    async def startHttpApp(self):
        app = web.Application()
        app.router.add_get(
            "/", self.handler
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
        """
        Sends msgs to client
        Unique_id used to specify which client
        No rate limiter used in contrast to normal push
        Accepts any msg format, normal accepts only ROS2 msgs
        """
        if not self.send_map:
            return
        if unique_id is None:
            for uid in self.send_map:
                self.send_map[uid].append(message)
        else:
            self.send_map[unique_id].append(message)

    async def handler(self, request):

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
        # Todo What are all expected messages? Or is
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
                    # "mode": self.system_mode,
                    "mobility": self.mobility
                }
            }), uid)
        elif obj.get("op") == "get_log_files":

            log_files = self.get_log_files()
            self.push_old(json.dumps({
                "op": "get_log_files_callback",
                "log_files": log_files
            }), uid)
        elif obj.get("op") == "get_log_file_content":

            log_file_path = obj.get("log_file_path")
            if log_file_path:
                content = self.get_log_file_content(log_file_path)
                self.push_old(json.dumps({
                    "op": "get_log_file_content_callback",
                    "log_file_path": log_file_path,
                    "content": content
                }), uid)
        elif obj.get("op") == "get_configuration":
            device = obj.get("device")
            if device:
                # -m ConfigurationBroadcast msg 2 req config
                msg = ConfigurationBroadcast()
                msg.device = device
                msg.opcode = 4  # Get configuration

                #publisher
                self.configuration_broadcast_p = self.create_publisher(
                    ConfigurationBroadcast,
                    Topics.CONFIGURATION_BROADCAST.value,
                    20
                )
                self.configuration_broadcast_p.publish(msg)

                # response send
                self.push_old(json.dumps({
                    "op": "get_configuration_response",
                    "device": device,
                    "status": "requested"
                }), uid)
        elif obj.get("op") == "configuration":
            device = obj.get("device")
            json_data = obj.get("json")
            if device and json_data:
                # pusher to robot
                msg = ConfigurationUpdate()
                msg.device = device
                msg.json = json_data if isinstance(json_data, str) else json.dumps(json_data)


                self.configuration_update_p = self.create_publisher(
                    ConfigurationUpdate,
                    Topics.CONFIGURATION_UPDATE.value,
                    20
                )
                self.configuration_update_p.publish(msg)
        # elif obj.get("op") == "set_system_state": todo fix setting sys state?
        #     self.set_system_total_state(int(obj["state"]), int(obj["mode"]), bool(obj["mobility"]))
        # TODO: add conbus, preset ops to ws msg


#todo add new callbacks
    def systemStateCallback(self, msg: SystemState):
        self.push(Topics.SYSTEM_STATE.value, msg)

    def deviceStateCallback(self, msg: DeviceState):
        self.push(Topics.DEVICE_STATE.value, msg)

#IMU Data
    def gpsFeedbackCallback(self, msg: GPSFeedback):
        self.push(Topics.AUTONAV_GPS.value, msg)

    def imuDataCallback(self, msg: IMUFeedback):
        self.push(Topics.IMU.value, msg)

    def motorInputCallback(self, msg: MotorInput):
        self.push(Topics.MOTOR_INPUT.value, msg)

    def positionCallback(self, msg: Position):
        self.push(Topics.POSITION.value, msg)

    def controllerInputCallback(self,msg):
        self.push(Topics.CONTROLLER_INPUT.value, msg)

    def motorFeedbackCallback(self, msg: MotorFeedback):
        self.push(Topics.MOTOR_FEEDBACK.value, msg)

    def statisticCallback(self,msg):
        self.push(Topics.NUC_STATISTICS.value, msg)

    def ultrasonicsCallback(self,msg):
        self.push(Topics.ULTRASONICS.value, msg)

    def conbusCallback(self,msg):
        self.push(Topics.CONBUS_DATA.value, msg)

    def safteyLightsCallback(self,msg):
        self.push(Topics.SAFETY_LIGHTS.value, msg)

    # New topic callbacks
    def logCallback(self, msg: Log):
        self.push(Topics.LOG.value, msg)

    def conbusInstructionCallback(self, msg: Conbus):
        self.push(Topics.CONBUS_INSTRUCTION.value, msg)

    def linearPIDStatisticsCallback(self, msg: LinearPIDStatistics):
        self.push(Topics.LINEAR_PID_STATISTICS.value, msg)

    def angularPIDStatisticsCallback(self, msg: AngularPIDStatistics):
        self.push(Topics.ANGULAR_PID_STATISTICS.value, msg)

    def motorStatisticsFrontCallback(self, msg: MotorStatistics):
        self.push(Topics.MOTOR_STATISTICS_FRONT.value, msg)

    def motorStatisticsBackCallback(self, msg: MotorStatistics):
        self.push(Topics.MOTOR_STATISTICS_BACK.value, msg)

    def canStatsCallback(self, msg: CanStats):
        self.push(Topics.CAN_STATS.value, msg)

    def zeroEncodersCallback(self, msg: ZeroEncoders):
        self.push(Topics.ZERO_ENCODERS.value, msg)

    def configurationBroadcastCallback(self, msg: ConfigurationBroadcast):
        self.push(Topics.CONFIGURATION_BROADCAST.value, msg)

        # If this is a response to a get_configuration request (opcode 4),
        # send the configuration back to all clients
        if msg.opcode == 4 and hasattr(msg, 'json') and msg.json:
            try:
                config_data = json.loads(msg.json)
                self.push_old(json.dumps({
                    "op": "get_configuration_response",
                    "device": msg.device,
                    "config": config_data
                }))
            except json.JSONDecodeError:
                self.get_logger().error(f"Failed to parse configuration JSON: {msg.json}")
            except Exception as e:
                self.get_logger().error(f"Error processing configuration broadcast: {str(e)}")

    # Cameras
    def cameraCallback(self, msg: CompressedImage, key: str):
        topic = getattr(Topics, f"RAW_{key.upper()}" if key in ['left', 'right', 'front', 'back'] else f"FEELERS").value
        self.push_image(topic, msg)

    def get_log_files(self):
        log_files = []
        home_dir = os.path.expanduser("~")
        log_base_dir = os.path.join(home_dir, ".autonav", "logs")


        if not os.path.exists(log_base_dir):
            return log_files


        for mode in ["manual", "autonomous"]:
            mode_dir = os.path.join(log_base_dir, mode)
            if os.path.exists(mode_dir):

                timestamp_dirs = glob.glob(os.path.join(mode_dir, "*"))
                for timestamp_dir in timestamp_dirs:
                    if os.path.isdir(timestamp_dir):

                        timestamp = os.path.basename(timestamp_dir)


                        log_file = os.path.join(timestamp_dir, "output.suslog")
                        if os.path.exists(log_file):
                            log_files.append({
                                "mode": mode,
                                "timestamp": timestamp,
                                "path": log_file,
                                "size": os.path.getsize(log_file)
                            })


        zip_files = glob.glob(os.path.join(log_base_dir, "*.zip"))
        for zip_file in zip_files:
            filename = os.path.basename(zip_file)

            parts = filename.split("_", 1)
            if len(parts) == 2:
                mode = parts[0]
                timestamp = parts[1].replace(".zip", "")
                log_files.append({
                    "mode": mode,
                    "timestamp": timestamp,
                    "path": zip_file,
                    "size": os.path.getsize(zip_file),
                    "is_zip": True
                })


        log_files.sort(key=lambda x: x["timestamp"], reverse=True)

        return log_files

    def get_log_file_content(self, log_file_path):
        """
        Get the content of a log file
        Returns the content as a string or an error message
        """
        try:
            if not os.path.exists(log_file_path):
                return {"error": "File not found"}


            if log_file_path.endswith(".zip"):
                return {"error": "Zip files cannot be viewed directly"}


            with open(log_file_path, "r") as f:
                content = f.read()


            try:
                parsed_content = json.loads(content)
                return {"content": parsed_content}
            except json.JSONDecodeError:

                return {"content": content, "is_plain_text": True}

        except Exception as e:
            return {"error": str(e)}

    def init(self):
        self.set_device_state(AutonavDeviceState.OPERATING)


def main():
    rclpy.init()
    node = BroadcastNode()
    Node.run_node(node)


if __name__ == "__main__":
    main()
