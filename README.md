# jetson_v4l2_camera

**English** | [한국어](README.ko.md)

A ROS 2 (Humble) driver that captures a V4L2 camera (GMSL2 etc.) on NVIDIA Jetson, encodes it with the **hardware JPEG encoder**, and publishes it with the **capture time** in `header.stamp`.

- One node per camera (`jetson_v4l2_camera_node`)
- Output: `sensor_msgs/CompressedImage` (JPEG) + `sensor_msgs/CameraInfo`
- Stamp: not the time the message was received, but the V4L2 buffer time (when the kernel received the frame) converted to UTC
- Frames are never dropped; missing frames and stamp reversals are counted from stamp intervals and logged

It was built for data collection, where images have to be aligned offline by time with other sensors (LiDAR, GNSS, etc.).

## How it works

```
/dev/videoN ─▶ v4l2src ─▶ nvvidconv ─▶ nvjpegenc ─▶ appsink ─▶ capture thread ─▶ image/compressed
 (V4L2 driver)  (raw UYVY)  (NVMM I420)  (HW JPEG)                 │  stamp           camera_info (1 Hz)
                                                                  └─ stats log (fps, missing, reversed)
```

| Component | What it does |
|---|---|
| GStreamer pipeline | `v4l2src` captures the native mode as is (no resize or crop), and `nvvidconv` → `nvjpegenc` produce JPEG on the Jetson hardware encoder. `appsink` uses `drop=false`, so frames are never dropped silently. The whole pipeline can be replaced with the `pipeline` parameter. |
| Capture thread | Pulls frames from `appsink` and publishes them as `CompressedImage`. It also checks the GStreamer bus for errors and EOS. |
| Timestamps | See [Timestamps](#timestamps) below. |
| CameraInfo | Reads the calibration file with `camera_info_manager` and publishes it every `camera_info_rate` with the same stamp as the image. Without calibration, only the image size is filled in. |
| Stats and failure handling | Every `stats_period`, logs fps, total frames, missing frames and the number of reversed stamps. On a pipeline error (device unplugged etc.) the node exits with code 1, so launch `respawn` restarts it. |

### Timestamps

1. With the pipeline clock fixed to the monotonic system clock, `v4l2src` keeps the V4L2 buffer time, so `PTS + base_time` = kernel capture time (`CLOCK_MONOTONIC`). The PTS is preserved through `nvvidconv` and `nvjpegenc`.
2. The current `CLOCK_REALTIME − CLOCK_MONOTONIC` is added to convert it to UTC. The system clock must be synchronized with PTP or NTP (`phc2sys`, `chrony`, etc.) to compare stamps with other hosts.
3. The per-camera `latency_offset_ns` (delay between exposure start and buffer time) is subtracted. The default is 0; measure it for each camera, e.g. with an LED + PPS test.

With `timestamp_source: receive`, the node uses the time it received the JPEG (`now()`). This is for debugging only.

## Requirements

| Item | Details |
|---|---|
| Platform | NVIDIA Jetson + L4T (tested on L4T R35.3.1 / JetPack 5.1.1, Orin) |
| ROS | ROS 2 Humble |
| Camera | A V4L2 device visible as `/dev/videoN`. The camera kernel driver must be on the host (board vendor BSP) |
| Dependencies | `rclcpp`, `sensor_msgs`, `camera_info_manager`, GStreamer 1.0 development packages (`gstreamer-1.0`, `gstreamer-app-1.0`), L4T GStreamer plugins (`nvvidconv`, `nvjpegenc`) |

When running in Docker, use a base image with the same L4T version as the host (e.g. `dustynv/ros:humble-ros-base-l4t-r35.3.1`), and pass `runtime: nvidia` and the `/dev/video*` devices. The hardware encoder only works when its version matches the host L4T.

## Build

```bash
cd ~/ros2_ws/src
git clone <this repository> jetson_v4l2_camera
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y   # where ROS packages are installed with apt
colcon build --symlink-install --packages-select jetson_v4l2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Topics / Services / Actions

Topic and service names are relative to the node namespace. The tables below assume the default launch (`namespace:=camera`).

### Published

| Topic | Type | Rate | Contents |
|---|---|---|---|
| `/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | `framerate` | JPEG frame. `format: jpeg`, `header.stamp` = capture time (UTC), `header.frame_id` = `frame_id` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_info_rate` (default 1 Hz) | Calibration. The stamp equals that of the image published at the same moment |

Both topics use reliable QoS with depth 10.

### Subscribed

None.

### Services

| Service | Type | Contents |
|---|---|---|
| `/camera/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | Provided by `camera_info_manager`. Used by the `camera_calibration` tool to save results to `camera_info_url` |

The parameter services common to every ROS 2 node (`~/get_parameters` etc.) are omitted.

### Actions

None.

## Parameters

All parameters are read once at startup. Changing them at runtime has no effect.

| Name | Type | Default | Description |
|---|---|---|---|
| `camera_name` | string | `camera` | Camera name. Used as the log prefix, the `camera_info_manager` name, and to build the default `frame_id` |
| `frame_id` | string | `<camera_name>_optical_frame` | `header.frame_id` of the messages |
| `device` | string | `/dev/video0` | V4L2 device path |
| `width` / `height` | int | `1920` / `1080` | Capture resolution. Must be a native mode supported by the camera |
| `framerate` | int | `30` | Frame rate. Also used to count missing frames |
| `pixel_format` | string | `UYVY` | `v4l2src` output format |
| `jpeg_quality` | int | `95` | `nvjpegenc` quality (0–100) |
| `pipeline` | string | `""` | If not empty, the capture settings above are ignored and this pipeline is used as is. Must end with `appsink name=sink` |
| `timestamp_source` | string | `v4l2` | `v4l2`: kernel capture time, `receive`: time the node received the frame (debug) |
| `latency_offset_ns` | int | `0` | Per-camera latency subtracted from the capture time (ns) |
| `camera_info_url` | string | `""` | Calibration file URL (e.g. `file:///path/camera.yaml`). If empty, `~/.ros/camera_info/<camera_name>.yaml` is tried; if that does not exist, a warning is logged and only the image size is published |
| `camera_info_rate` | double | `1.0` | CameraInfo publish rate (Hz). `0` disables it |
| `stats_period` | double | `10.0` | Stats log period (s) |

An example parameter file is [config/jetson_v4l2_camera.yaml](config/jetson_v4l2_camera.yaml).

## Launch

[launch/jetson_v4l2_camera.launch.py](launch/jetson_v4l2_camera.launch.py) starts one camera. Common settings come from the parameter file, and per-camera values are overridden with launch arguments.

| Argument | Default | Description |
|---|---|---|
| `params_file` | `share/jetson_v4l2_camera/config/jetson_v4l2_camera.yaml` | Node parameter file |
| `camera_name` | `camera` | `camera_name` parameter |
| `device` | `/dev/video0` | `device` parameter |
| `namespace` | `camera` | Topic namespace |
| `pipeline` | `""` | `pipeline` parameter (replaces the whole pipeline) |
| `respawn` | `true` | Restart the node 1 s after it exits |

### Examples

One camera:

```bash
ros2 launch jetson_v4l2_camera jetson_v4l2_camera.launch.py \
  camera_name:=front device:=/dev/video0 namespace:=cam/front
# → /cam/front/image/compressed, /cam/front/camera_info, frame_id: front_optical_frame
```

With your own parameter file (resolution, calibration, latency offset, etc.):

```bash
ros2 launch jetson_v4l2_camera jetson_v4l2_camera.launch.py \
  params_file:=/path/to/my_camera.yaml camera_name:=front device:=/dev/video0 namespace:=cam/front
```

For multiple cameras, run the launch file once per camera with a different `camera_name`, `device` and `namespace`, or create one node per camera from a higher-level launch file.

Without launch:

```bash
ros2 run jetson_v4l2_camera jetson_v4l2_camera_node --ros-args \
  -r __ns:=/cam/front -p camera_name:=front -p device:=/dev/video0
```

### Testing on a non-Jetson PC

On a PC without `nvvidconv` and `nvjpegenc`, replace `pipeline` with a software pipeline to check the node and its topics (requires `gstreamer1.0-plugins-base` and `-good`).

```bash
ros2 launch jetson_v4l2_camera jetson_v4l2_camera.launch.py respawn:=false pipeline:="videotestsrc is-live=true ! video/x-raw,format=UYVY,width=1920,height=1080,framerate=30/1 ! videoconvert ! jpegenc quality=95 ! appsink name=sink sync=false max-buffers=8 drop=false"
```

## Checking operation

```bash
# Modes supported by the camera (match width/height/pixel_format/framerate to these)
v4l2-ctl -d /dev/video0 --list-formats-ext

# Rate and stamps
ros2 topic hz /camera/image/compressed
ros2 topic echo --once /camera/image/compressed --field header
```

Every `stats_period`, the node logs a line like the one below. If `missing` or `non-monotonic stamps` grows, frames were lost or time went backwards.

```
[front] 30.07 fps, frames 301, missing 0, non-monotonic stamps 0
```

| Symptom | What to check |
|---|---|
| `pipeline parse error` / `no element "nvjpegenc"` | Whether the L4T GStreamer plugins are present; in a container, `runtime: nvidia` and a matching L4T version |
| `not-negotiated` error | Whether `width`, `height`, `pixel_format` and `framerate` match the `--list-formats-ext` output |
| `Camera calibration file ... not found` warning | Expected before calibration. It goes away once `camera_info_url` is set |
| `missing` keeps increasing | Encoder and CPU load when running several cameras, recording disk speed |

## License

[Apache License 2.0](LICENSE)

Copyright 2026 Chungbuk National University, Autonomous Vehicle Laboratory
