# jetson_v4l2_camera

[English](README.md) | **한국어**

NVIDIA Jetson에서 V4L2 카메라(GMSL2 등)를 받아 **하드웨어 JPEG**으로 인코딩하고, **촬영 시각**을 `header.stamp`에 담아 발행하는 ROS 2 (Humble) 드라이버입니다.

- 카메라 1대당 노드 1개 (`jetson_v4l2_camera_node`)
- 출력: `sensor_msgs/CompressedImage` (JPEG) + `sensor_msgs/CameraInfo`
- stamp: 메시지를 받은 시각이 아니라, 커널이 프레임을 받은 V4L2 버퍼 시각을 UTC로 바꾼 값
- 프레임을 버리지 않고, 누락·역전은 stamp 간격으로 세어 로그에 남김

데이터 취득처럼 영상과 다른 센서(LiDAR, GNSS 등)를 오프라인에서 시각으로 맞춰야 하는 용도를 위해 만들었습니다.

## 동작 구조

```
/dev/videoN ─▶ v4l2src ─▶ nvvidconv ─▶ nvjpegenc ─▶ appsink ─▶ capture thread ─▶ image/compressed
 (V4L2 드라이버)  (원본 UYVY)  (NVMM I420)  (HW JPEG)                 │  stamp 계산      camera_info (1 Hz)
                                                                  └─ 통계 로그 (fps, 누락, 역전)
```

| 구성 요소 | 하는 일 |
|---|---|
| GStreamer 파이프라인 | `v4l2src`로 원본 모드를 그대로 받고(리사이즈·크롭 없음) `nvvidconv` → `nvjpegenc`로 Jetson 하드웨어 인코더에서 JPEG을 만듭니다. `appsink`는 `drop=false`라서 프레임을 조용히 버리지 않습니다. `pipeline` 파라미터로 전체를 바꿀 수 있습니다. |
| 캡처 스레드 | `appsink`에서 프레임을 꺼내 `CompressedImage`로 발행합니다. GStreamer 버스의 오류·EOS도 이 스레드가 확인합니다. |
| 타임스탬프 | 아래 [타임스탬프](#타임스탬프) 참고. |
| CameraInfo | `camera_info_manager`로 캘리브레이션 파일을 읽고, 이미지와 같은 stamp로 `camera_info_rate` 주기마다 발행합니다. 캘리브레이션이 없으면 이미지 크기만 채웁니다. |
| 통계·장애 처리 | `stats_period`마다 fps, 누적 프레임, 누락 프레임, 역전된 stamp 수를 로그로 남깁니다. 파이프라인 오류(장치 분리 등)가 나면 노드가 종료 코드 1로 끝나므로, launch의 `respawn`으로 다시 띄웁니다. |

### 타임스탬프

1. 파이프라인 시계를 monotonic 시스템 시계로 고정하면 `v4l2src`가 V4L2 버퍼 시각을 유지하므로, `PTS + base_time` = 커널 캡처 시각(`CLOCK_MONOTONIC`)이 됩니다. PTS는 `nvvidconv`·`nvjpegenc`를 거쳐도 유지됩니다.
2. 여기에 현재의 `CLOCK_REALTIME − CLOCK_MONOTONIC`을 더해 UTC로 바꿉니다. 시스템 시계를 PTP·NTP(`phc2sys`, `chrony` 등)로 맞춰 두어야 다른 호스트와 비교할 수 있습니다.
3. 카메라별 `latency_offset_ns`(노출 시작 ~ 버퍼 시각 사이의 지연)를 뺍니다. 기본값 0이며, LED·PPS 시험 등으로 카메라마다 측정해 넣습니다.

`timestamp_source: receive`로 바꾸면 노드가 JPEG을 받은 시각(`now()`)을 씁니다. 디버깅 전용입니다.

## 요구 사항

| 항목 | 내용 |
|---|---|
| 플랫폼 | NVIDIA Jetson + L4T (시험 환경: L4T R35.3.1 / JetPack 5.1.1, Orin) |
| ROS | ROS 2 Humble |
| 카메라 | `/dev/videoN`으로 보이는 V4L2 장치. 카메라 커널 드라이버는 호스트(보드 제조사 BSP)에 있어야 합니다 |
| 의존성 | `rclcpp`, `sensor_msgs`, `camera_info_manager`, GStreamer 1.0 개발 패키지(`gstreamer-1.0`, `gstreamer-app-1.0`), L4T GStreamer 플러그인(`nvvidconv`, `nvjpegenc`) |

Docker에서 실행할 때는 L4T 버전이 호스트와 같은 베이스 이미지(예: `dustynv/ros:humble-ros-base-l4t-r35.3.1`)를 쓰고, `runtime: nvidia`와 `/dev/video*` 장치를 넘겨 주세요. 하드웨어 인코더는 호스트 L4T와 버전이 맞아야 동작합니다.

## 빌드

```bash
cd ~/ros2_ws/src
git clone <this repository> jetson_v4l2_camera
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y   # apt로 ROS 패키지를 설치한 환경에서
colcon build --symlink-install --packages-select jetson_v4l2_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Topic / Service / Action

토픽과 서비스 이름은 노드의 namespace 아래에 붙습니다. 아래 표는 기본 launch(`namespace:=camera`) 기준입니다.

### 발행 (Publish)

| Topic | 타입 | 주기 | 내용 |
|---|---|---|---|
| `/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | `framerate` | JPEG 프레임. `format: jpeg`, `header.stamp` = 촬영 시각(UTC), `header.frame_id` = `frame_id` |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_info_rate` (기본 1 Hz) | 캘리브레이션. stamp는 같은 시점에 발행한 이미지와 같습니다 |

두 토픽 모두 QoS는 reliable, depth 10입니다.

### 구독 (Subscribe)

없음.

### Service

| Service | 타입 | 내용 |
|---|---|---|
| `/camera/set_camera_info` | `sensor_msgs/srv/SetCameraInfo` | `camera_info_manager`가 제공합니다. `camera_calibration` 도구가 캘리브레이션 결과를 `camera_info_url` 위치에 저장할 때 씁니다 |

ROS 2 노드라면 공통으로 생기는 파라미터 서비스(`~/get_parameters` 등)는 생략했습니다.

### Action

없음.

## Parameter

모든 파라미터는 시작할 때 한 번만 읽습니다. 실행 중에 바꿔도 반영되지 않습니다.

| 이름 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `camera_name` | string | `camera` | 카메라 이름. 로그 접두사와 `camera_info_manager` 이름으로 쓰고, `frame_id` 기본값을 만들 때도 씁니다 |
| `frame_id` | string | `<camera_name>_optical_frame` | 메시지의 `header.frame_id` |
| `device` | string | `/dev/video0` | V4L2 장치 경로 |
| `width` / `height` | int | `1920` / `1080` | 캡처 해상도. 카메라가 지원하는 원본 모드여야 합니다 |
| `framerate` | int | `30` | 프레임레이트. 누락 프레임 계산에도 씁니다 |
| `pixel_format` | string | `UYVY` | `v4l2src` 출력 포맷 |
| `jpeg_quality` | int | `95` | `nvjpegenc` 품질 (0–100) |
| `pipeline` | string | `""` | 비어 있지 않으면 위 캡처 설정을 무시하고 이 파이프라인을 그대로 씁니다. 반드시 `appsink name=sink`로 끝나야 합니다 |
| `timestamp_source` | string | `v4l2` | `v4l2`: 커널 캡처 시각, `receive`: 노드가 받은 시각(디버그용) |
| `latency_offset_ns` | int | `0` | 캡처 시각에서 뺄 카메라별 지연 (ns) |
| `camera_info_url` | string | `""` | 캘리브레이션 파일 URL (예: `file:///path/camera.yaml`). 비어 있으면 `~/.ros/camera_info/<camera_name>.yaml`을 찾고, 파일이 없으면 경고 후 이미지 크기만 발행합니다 |
| `camera_info_rate` | double | `1.0` | CameraInfo 발행 주기 (Hz). `0`이면 발행하지 않습니다 |
| `stats_period` | double | `10.0` | 통계 로그 주기 (초) |

예시 파라미터 파일은 [config/jetson_v4l2_camera.yaml](config/jetson_v4l2_camera.yaml)입니다.

## Launch

[launch/jetson_v4l2_camera.launch.py](launch/jetson_v4l2_camera.launch.py)는 카메라 1대를 띄웁니다. 공통 설정은 파라미터 파일에서 읽고, 카메라마다 달라지는 값은 launch 인자로 덮어씁니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `params_file` | `share/jetson_v4l2_camera/config/jetson_v4l2_camera.yaml` | 노드 파라미터 파일 |
| `camera_name` | `camera` | `camera_name` 파라미터 |
| `device` | `/dev/video0` | `device` 파라미터 |
| `namespace` | `camera` | 토픽 namespace |
| `pipeline` | `""` | `pipeline` 파라미터 (전체 파이프라인 교체) |
| `respawn` | `true` | 노드가 종료되면 1초 뒤 다시 시작 |

### 사용 예

카메라 1대:

```bash
ros2 launch jetson_v4l2_camera jetson_v4l2_camera.launch.py \
  camera_name:=front device:=/dev/video0 namespace:=cam/front
# → /cam/front/image/compressed, /cam/front/camera_info, frame_id: front_optical_frame
```

자기 파라미터 파일을 쓰는 경우 (해상도, 캘리브레이션, 지연 오프셋 등):

```bash
ros2 launch jetson_v4l2_camera jetson_v4l2_camera.launch.py \
  params_file:=/path/to/my_camera.yaml camera_name:=front device:=/dev/video0 namespace:=cam/front
```

여러 대를 띄울 때는 카메라마다 `camera_name`, `device`, `namespace`를 다르게 줘서 launch를 여러 번 실행하거나, 상위 launch 파일에서 이 노드를 카메라 수만큼 만듭니다.

launch 없이 실행:

```bash
ros2 run jetson_v4l2_camera jetson_v4l2_camera_node --ros-args \
  -r __ns:=/cam/front -p camera_name:=front -p device:=/dev/video0
```

### Jetson이 아닌 PC에서 시험

`nvvidconv`·`nvjpegenc`가 없는 PC에서는 `pipeline`을 소프트웨어 파이프라인으로 바꿔서 노드와 토픽 동작을 확인할 수 있습니다 (`gstreamer1.0-plugins-base`, `-good` 필요).

```bash
ros2 launch jetson_v4l2_camera jetson_v4l2_camera.launch.py respawn:=false pipeline:="videotestsrc is-live=true ! video/x-raw,format=UYVY,width=1920,height=1080,framerate=30/1 ! videoconvert ! jpegenc quality=95 ! appsink name=sink sync=false max-buffers=8 drop=false"
```

## 동작 확인

```bash
# 카메라가 지원하는 모드 (width/height/pixel_format/framerate를 여기에 맞춤)
v4l2-ctl -d /dev/video0 --list-formats-ext

# 주기와 stamp
ros2 topic hz /camera/image/compressed
ros2 topic echo --once /camera/image/compressed --field header
```

노드 로그에 `stats_period`마다 다음과 같은 줄이 나옵니다. `missing`이나 `non-monotonic stamps`가 늘어나면 프레임이 빠졌거나 시각이 뒤로 간 것입니다.

```
[front] 30.07 fps, frames 301, missing 0, non-monotonic stamps 0
```

| 증상 | 확인할 것 |
|---|---|
| `pipeline parse error` / `no element "nvjpegenc"` | L4T GStreamer 플러그인이 있는지, 컨테이너라면 `runtime: nvidia`와 L4T 버전이 맞는지 |
| `not-negotiated` 오류 | `width`·`height`·`pixel_format`·`framerate`가 `--list-formats-ext` 결과와 맞는지 |
| `Camera calibration file ... not found` 경고 | 캘리브레이션 전이면 정상입니다. `camera_info_url`을 지정하면 사라집니다 |
| `missing`이 계속 늘어남 | 여러 대를 동시에 띄웠을 때 인코더·CPU 부하, 기록 디스크 속도 |

## License

[Apache License 2.0](LICENSE)

Copyright 2026 Chungbuk National University, Autonomous Vehicle Laboratory
