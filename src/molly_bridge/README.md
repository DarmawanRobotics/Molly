# molly_bridge

How the handheld talks to the robot without ROS: the UDP gateway with the guide
commands, and its WebRTC video. The control mux that decides who drives is
[dharma_control](https://github.com/DarmawanRobotics/dharma_navigation); Molly starts it with
the `control_mux_node` section of `config/bridge_params.yaml`.

## Nodes

### teleop_udp_node

UDP gateway for the [Molly Controller](https://github.com/DarmawanRobotics/MollyController) handheld: joystick, commands (mode, navigation, route recording, stop
points, routes, localization, robot commands), 10 Hz telemetry, JPEG camera, downsampled lidar and
the loaded route. The wire format is in [PROTOCOL.md](PROTOCOL.md). With `video_port` set (the
`molly` launch does it when MediaMTX runs) it offers the handheld the WebRTC video instead of
the JPEG frames.

Set the same `token` here (site file) and in the handheld; a wrong one is answered with `denied`.
`robot_commands` are the driver's `std_srvs/Trigger` services shown in the handheld's action
menu, in this order. The camera, the cloud and TF are only subscribed while a handheld is connected
and has that stream on, so an idle gateway costs next to nothing.

### video_stream_node

H.264 of the camera for the handheld's WebRTC video. MediaMTX starts it when the handheld opens
the video and stops it 10 s after the last viewer, so it costs nothing while nobody watches. It
encodes `image_topic` (raw `rgb8`, `bgr8`, `mono8`, `yuv422`, ...) at `fps`, no wider than
`max_width`, with the first encoder that works: the Jetson hardware encoder (`nvv4l2h264enc`, not
on the Orin Nano), x264, OpenH264. Then it publishes over RTSP to MediaMTX on the loopback, and
MediaMTX serves WebRTC (WHEP) to the handheld. MediaMTX, its configuration and the ports come from
`molly` `robot.launch.py`; GStreamer and MediaMTX from `script/setup.sh install webrtc`.

## Parameters

All parameters: [config/bridge_params.yaml](config/bridge_params.yaml). For a lighter camera
stream publish a compressed image (`camera_compressed: true`); raw images are JPEG-encoded with
OpenCV.

## Launch

```bash
ros2 launch molly_bridge bridge.launch.py robot_ns:=robot
ros2 launch dharma_navigation navigation.launch.py cmd_vel_topic:=/bridge/nav_cmd_vel
```

## Test

```bash
colcon test --packages-select molly_bridge && colcon test-result --verbose
```
