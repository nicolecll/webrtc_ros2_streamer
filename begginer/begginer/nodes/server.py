#!/usr/bin/env python3

# Importing necessary libraries
import argparse
import asyncio
import json
import logging
import os
import ssl
import uuid
import time
import cv2
import threading
import rclpy
import cv_bridge 
from rclpy.node import Node
from sensor_msgs.msg import Image
from aiohttp import web
from av import VideoFrame
from fractions import Fraction
from aiortc import MediaStreamTrack, RTCPeerConnection, RTCSessionDescription

# Setting up logging and global variables
ROOT = os.path.dirname(__file__)
logger = logging.getLogger("pc")
pcs = set()

class Intermediate(Node):
    """
    ROS Node for handling image data and adjusting the quality based on network conditions.
    """
    def __init__(self, mode="auto"):
        super().__init__('intermediate_node')
        self.bridge = cv_bridge.CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/image/rosbag', self.image_callback, 10)
        self.last_time = time.time()
        self.fps = 30
        self.lock = threading.Lock()
        self.latest_image = None
        self.placeholder_image = cv2.imread("placeholder.jpg")
        self.new_image = None
        self.rtt = None
        self.manual_resolution = (1881, 1051)
        self.mode = mode
        logger.info("Intermediate Node initialized in %s mode", self.mode)

    def image_callback(self, msg):
        """
        Callback function to process the incoming image messages.
        """
        current_time = time.time()
        if current_time - self.last_time >= 1.0 / self.fps:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.mode == "manual":
                resized_image = cv2.resize(cv_image, self.manual_resolution)
                self.new_image = resized_image
            else:
                resized_image = self.resize_image(cv_image)
                self.new_image = resized_image
            self.last_time = current_time

    def update_bandwidth(self, msg):
        """
        Updates the available bandwidth based on message data.
        """
        self.bandwidth = msg.data
        self.adjust_fps_and_resolution()

    def adjust_fps_and_resolution(self):
        """
        Adjusts FPS and resolution based on the current round-trip time (RTT).
        """
        rtt_settings = {
            (0, 3): {'resolution': (1920, 1080), 'fps': 60},
            (4, 7): {'resolution': (820, 720), 'fps': 30},
            (8, 11): {'resolution': (640, 480), 'fps': 15},
            (12, float('inf')): {'resolution': (320, 240), 'fps': 5},
        }
        for (lower_bound, upper_bound), settings in rtt_settings.items():
            if lower_bound <= self.rtt < upper_bound:
                self.fps = settings['fps']
                self.resolution = settings['resolution']
                logger.info("Adjusted FPS to %s and resolution to %s", self.fps, self.resolution)
                break

    def resize_image(self, image):
        """
        Resize the image based on the current bandwidth.
        """
        if self.rtt is not None:
            self.adjust_fps_and_resolution()
            return cv2.resize(image, self.resolution)
        return image

    def get_latest_image(self):
        """
        Returns the latest processed image or a placeholder if none available.
        """
        return self.new_image if self.new_image is not None else self.placeholder_image

class ImageVideoTrack(MediaStreamTrack):
    """
    MediaStreamTrack for video, streaming images from the ROS node.
    """
    kind = "video"

    def __init__(self, intermediate_node):
        super().__init__()
        self.start_time = time.time()
        self.frames = 0
        self.framerate = 30
        self.intermediate_node = intermediate_node

    async def next_timestamp(self):
        """
        Calculates the timestamp for the next frame.
        """
        self.frames += 1
        next_time = self.start_time + (self.frames / self.framerate)
        await asyncio.sleep(max(0, next_time - time.time()))
        return int((next_time - self.start_time) * 1000)

    async def recv(self):
        """
        Receives the next video frame to be sent to the peer.
        """
        frame = await self.get_frame()
        image_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        image_frame.pts = await self.next_timestamp()
        image_frame.time_base = Fraction(1, 1000)
        return image_frame

    async def get_frame(self):
        """
        Retrieves the latest image frame from the intermediate node.
        """
        latest_frame = self.intermediate_node.get_latest_image()
        await asyncio.sleep(1.0 / self.intermediate_node.fps)
        return latest_frame

# Web handler functions
async def index(request):
    """
    Serves the index.html page.
    """
    content = open(os.path.join(ROOT, "index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def javascript(request):
    """
    Serves the client.js JavaScript file.
    """
    content = open(os.path.join(ROOT, "client.js"), "r").read()
    return web.Response(content_type="application/javascript", text=content)

async def offer(request):
    """
    Handles the WebRTC offer from the client.
    """
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    client_resolutions = params["video_resolution"]

    if intermediate_node.mode == "manual":
        shape = client_resolutions.split('x')
        new_resolution = (int(shape[0]), int(shape[1]))
        intermediate_node.manual_resolution = new_resolution

    pc = RTCPeerConnection()
    pc_id = "PeerConnection(%s)" % uuid.uuid4()
    pcs.add(pc)

    def log_info(msg, *args):
        """
        Utility function for logging information.
        """
        logger.info(pc_id + " " + msg, *args)

    log_info("Received WebRTC offer")

    image_track = ImageVideoTrack(intermediate_node)
    pc.addTrack(image_track)

    @pc.on("datachannel")
    def on_datachannel(channel):
        @channel.on("message")
        def on_message(message):
            """
            Processes incoming messages on the data channel.
            """
            if isinstance(message, str) and message.startswith("ping"):
                log_info("Received ping message", message)
                channel.send("pong" + message[4:])
            if isinstance(message, str) and message.startswith("latency"):
                intermediate_node.rtt = int(message[7:])
                log_info("Updated RTT to %d", intermediate_node.rtt)

    @pc.on("iceconnectionstatechange")
    async def on_iceconnectionstatechange():
        """
        Monitors the ICE connection state changes.
        """
        log_info("ICE connection state is %s", pc.iceConnectionState)
        if pc.iceConnectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
        ),
    )

async def on_shutdown(app):
    """
    Handles the shutdown of the web application, closing all peer connections.
    """
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


def main(args=None):

    parser = argparse.ArgumentParser(
        description="WebRTC audio / video / data-channels demo"
    )
    parser.add_argument("--cert-file", help="SSL certificate file (for HTTPS)")
    parser.add_argument("--key-file", help="SSL key file (for HTTPS)")
    parser.add_argument(
        "--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=8081, help="Port for HTTP server (default: 8080)"
    )
    parser.add_argument("--verbose", "-v", action="count")
    parser.add_argument("--write-audio", help="Write received audio to a file")
    parser.add_argument("--mode", choices=["manual", "auto"], default="auto", help="Set mode to 'manual' or 'auto' (default: 'auto')")

    # args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    if args.cert_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)
    else:
        ssl_context = None

    # App initialization and ROS node creation
    app = web.Application()
    app.on_shutdown.append(on_shutdown)
    app.router.add_get("/", index)
    app.router.add_get("/client.js", javascript)
    app.router.add_post("/offer", offer)

    rclpy.init()
    global intermediate_node
    intermediate_node = Intermediate(mode=args.mode)

    # Start the ROS node in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(intermediate_node), daemon=True)
    ros_thread.start()

    # Start the web application
    web.run_app(
        app, access_log=None, host=args.host, port=args.port, ssl_context=ssl_context
    )

    ros_thread.join()

if __name__ == "__main__":
    main()