# webrtc_ros2_streamer
This repository hosts an advanced ROS2 package designed to seamlessly integrate WebRTC into robotic applications. Its primary purpose is to facilitate real-time transmission of images and visual data captured from specific ROS2 topics.

# Webrtc stream for ros2 topics:robot:

This project aims to transmit images from a ros2 topic called **/cam_front/image_raw** through a peer to peer architecture provided by WebRTC, WebRTC is a free and open source project that provides web browsers and mobile applications with real-time communication through application programming interfaces. 

## Prerequisites:white_check_mark:

List of tools, libraries, or prior knowledge necessary to use the project. For example:

- Python 3.x
- ROS2 (Robot Operating System 2)
- OpenCV
- aiortc (pip install aiortc)
- aiohttp (pip install aiohttp)
- av (pip install av)
- Basic knowledge of WebRTC and video streaming
- An appropriate development environment for working with ROS2 and Python.
- **Have the rosbag2_2023_10_25-09_13_26 folder at the same height as the nodes folder in this repository.**


## Installation and Usage:computer:

1. Clone begginer ros2 package into your ros2 workspace
```bash
git clone https://github.com/nicolecll/webrtc_ros2_streamer.git
```
2. Rebuild and load the packages into your ros2 workspace
```bash
colcon build 
```
3. Move to nodes directory 
```bash
cd begginer/nodes
```
4. Change permission files 
```bash
chmod a+x reader_bag.py
chmod a+x server.py
```
5. Execute the node reader_bag with the command
```bash
./reader_bag.py
```
6. Open another console and move to nodes directory again
```bash
cd begginer/nodes
```
7. Execute the server node with the command
```bash
./server.py
```
The server node has two modes, manual and automatic, by default it runs in automatic mode, this allows that depending on the identified latency the images coming from the topic /cam_front/image_raw, change their resolution to allow more fluidity in the transmission. If you want to run in manual mode, you must run the node as follows:
```bash
./server.py --mode manual
```
This allows the user to choose the resolution of the images through the user interface.

## Warnings:warning:
- The site can be viewed correctly from devices such as laptops, android phones and tablets. Its use on IOS devices is not implemented.
- If the application cannot be displayed correctly, check the rules that allow incoming and outgoing traffic on port 8081 in the device's firewall, it may be necessary to create them if they do not exist.
- Check that the  rosbag2_2023_10_25-09_13_26 folder is at the same height as the nodes folder and that it contains the metada.yaml and rosbag2_2023_10_25-09_13_26_0.db3 file.
  
## Visit the web application:key:

The web application is hosted at address 0.0.0.0.0 (localhost) on port 8081, if you want to visit it on the same device that runs the ros nodes, you must open the browser and in the browser visit

```bash
localhost:8081
```
If you want to visit the application on devices that are on the same local network (LAN) you need to type the following in the browser

```bash
IP_HOST:8081
```

Where IP_HOST is the IP of the device that mounts the web application, this must be known by the other devices and can be obtained by running the ifconfig command on the host device. 

The application cannot be visited from devices that are not on the same local network.

## Files documentation:file_folder:

### 1. reader_bag.py

This Python script creates a ROS (Robot Operating System) node, designed to read and process image data from a ROS bag file. Here's an overview of what the node does:

- **Initialization**: The node, named `'reader_bag'`, initializes itself, sets up a `CvBridge` for image data handling, and creates an image publisher on the `/image/rosbag` topic.

- **Reading ROS Bag File**: The node opens a specific ROS bag file (`'../rosbag2_2023_10_25-09_13_26'`) and begins to read through its contents.

- **Filtering and Processing Image Messages**: It specifically looks for messages from the `/cam_front/image_raw` topic within the bag file. As it encounters these messages, it deserializes them to convert them into a format that can be used within the ROS ecosystem.

- **Publishing Image Data**: After converting the raw data into ROS Image messages, the node publishes these images onto the `/image/rosbag` topic. This makes the image data available for other nodes in the ROS network to subscribe to and use.

- **Continuous Operation**: The node remains operational, continuously reading from the bag file and publishing image messages, until it's manually shut down.

### 2. server.py

This Python script is a sophisticated integration of a ROS (Robot Operating System) node and a WebRTC-based web application. Here's an overview of its functionality:

### ROS Node - `Intermediate`
- **Purpose**: Handles image data and adjusts the quality based on network conditions.
- **Initialization**: Sets up as a ROS node named `intermediate_node`, creates a subscription to the `/image/rosbag` topic, and initializes various parameters related to image processing.
- **Image Processing**:
  - **Callback Function**: Processes incoming image messages. It adjusts the image size based on the operational mode (`manual` or `auto`), which affects how images are resized.
  - **Network Adaptation**: Includes functions to update bandwidth and adjust frames per second (FPS) and resolution based on network round-trip time (RTT). These adjustments help optimize image quality for varying network conditions.

### MediaStreamTrack - `ImageVideoTrack`
- **Functionality**: Facilitates video streaming of images from the ROS node. It handles the timing and delivery of video frames in a WebRTC session.
- **Frame Handling**: Includes methods for timestamp generation and retrieval of the latest image frame from the `Intermediate` node for streaming.

### Web Application Handlers
- **WebRTC Integration**: Handles WebRTC offer from a client, establishing a peer-to-peer connection.
- **Dynamic Resolution**: Adapts the image resolution based on client requirements when in manual mode.
- **Data Channel**: Processes incoming messages on the WebRTC data channel, particularly for updating RTT values.
- **Shutdown Handling**: Closes all peer connections upon the shutdown of the web application.

### Main Function
- **Argument Parsing**: Allows configuring the application through command-line arguments (like SSL certificates, host, port, and operational mode).
- **Web Server Initialization**: Sets up an `aiohttp` web application, routes, and handles shutdown operations.
- **ROS and WebRTC Integration**: Initializes the ROS node and starts it in a separate thread. It also launches the web application, enabling real-time video streaming of ROS-handled image data over WebRTC.

### 3. client.js

This JavaScript file is designed to establish and manage a WebRTC peer connection for audio and video communication, along with a data channel for sending and receiving messages. Here is a breakdown of its key functionalities:

- **Variable Initialization**: Sets up variables to log various connection states (`iceConnectionLog`, `iceGatheringLog`, `signalingLog`, `dataChannelLog`) and initializes the peer connection (`pc`) and data channel (`dc`) variables.

- **Creating a Peer Connection (`createPeerConnection`)**: Initializes a new RTCPeerConnection with appropriate configuration. It also sets up event listeners to log changes in ICE gathering state, ICE connection state, and signaling state. Additionally, it handles incoming media tracks by attaching them to video or audio elements in the webpage.

- **Negotiating the Peer Connection (`negotiate`)**: Initiates the negotiation process for the peer connection. This involves creating an offer, setting the local description, handling ICE candidate gathering, sending the offer to a server, and then setting the remote description once an answer is received.

- **Starting the Connection (`start`)**: Initiates the peer connection and data channel. It creates a data channel with specified parameters, sets up event handlers for data channel events (open, close, message), and begins the negotiation process. It also sends periodic 'ping' messages over the data channel to measure round-trip time (RTT).

- **Stopping the Connection (`stop`)**: Closes the data channel and stops all transceivers and tracks associated with the peer connection, then closes the peer connection itself.

- **SDP Filtering (`sdpFilterCodec`)**: Provides a function to filter the Session Description Protocol (SDP) for a specific codec, useful for adapting the connection based on codec support and preferences.

- **Utility Function (`escapeRegExp`)**: A helper function to escape special characters in a string for use in regular expressions.

### 4. index.html

File that allows the client to view the web application.

## Difficulties faced:brain:

One of the main difficulties I faced in the development of this technical test was the implementation of the webrtc architecture, in particular the communication and the offer for the client to receive media from the server, in particular configuring the connection and allowing more than one connection. 

Another difficulty faced was the use of launch files, as when trying to launch the nodes in a unified way through a launch file, several problems occurred related to the reading of the rosbag and its metadata file. This is why it was decided to leave the execution of each node separately on two different terminals.

Finally one of the most important difficulties was the improvement of the latency of the transmission, although I tried to improve the efficiency of the code and the transmission, in the first images of the rosbag you can notice high latency. 

## To be improved:next_track_button:
1. Launching nodes through launch files
2. Streamlining existing code to improve the smoothness of transmission

## Bonus: To be developed and explained in the interview:bulb:
