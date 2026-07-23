# Aruco Pose Estimation with ROS2, using RGB and Depth camera images from Realsense D435

Code developed by: __Simone Giampà__

Project and experimentation conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis project at Politecnico di Milano, Italy._

ROS2 wrapper for Aruco marker detection and pose estimation, using OpenCV library. The marker detection and pose estimation is
done using RGB and optionally Depth images. This package works for ROS2 Humble and Iron.

This package allows to use cameras to detect Aruco markers and estimate their poses. It allows to use any camera with ROS2 drivers.
The code is a ROS2 publisher-subscriber working with RGB camera images for marker detection and RGB or depth images for pose estimation. 
It also allows using multiple aruco markers at the same time, and each of them will be published as a separate pose. 
The code supports many different Aruco dictionaries and sizes of markers.

This package was tested for the Realsense D435 camera, compatible with ROS2 `realsense-ros` driver,
available at [ros2_intel_realsense](https://github.com/IntelRealSense/realsense-ros). The code should work equally well on other and
different cameras, provided a proper calibration of the camera parameters.

## Installation

This package depends on a recent version of OpenCV python library and transforms libraries:

```bash
$ pip3 install opencv-python opencv-contrib-python transforms3d

```

Build the package from source with `colcon build --symlink-install` in the workspace root.

## Aruco Pose Detection and Estimation ROS2 nodes description

This node subscribes to the RGB and optionally Depth images from the camera, and the camera inof topic for
intrinsic and distortion parameters. It detects Aruco markers in the RGB image, and estimates their poses using the
camera intrinsic parameters or the depth image. The poses are published as PoseArray message, and the detected markers
are published as ArucoMarkers messages. The output image contains the detected markers and aruco bounding boxes drawn on it.

__Subscribed topics__ (topic names can be changed in the `config/aruco_parameters.yaml` file):

* `/camera/image_raw`: RGB image input (`sensor_msgs.msg.Image`)
* `/camera/depth/image_rect_raw`: Depth image input (`sensor_msgs.msg.Image`)
* `/camera/camera_info`: Camera intrinsic, projection, distortion parameters (`sensor_msgs.msg.CameraInfo`)

__Published topics__ (topic names can be changed in the `config/aruco_parameters.yaml` file):

* `/aruco/poses`: Poses of all detected markers, suitable for rviz visualization - (`geometry_msgs.msg.PoseArray`) - 
* `/output/aruco/markers`: Provides an array of all poses along with the corresponding marker ids - (`aruco_interfaces.msg.ArucoMarkers`)
* `/aruco/image`: Output image with detected markers drawn on it, for visualization purposes - (`sensor_msgs.msg.Image`)

__Parameters__ for the node can be set in the `config/aruco_parameters.yaml` file, and include the following options:

* `marker_size` - size of the markers in meters
* `aruco_dictionary_id` - dictionary type that was used to generate markers (example `DICT_5X5_250`)
* `use_depth_input` - use depth image for pose estimation (default `false`)
* `image_topic` - RGB image topic to subscribe to, provided by the camera ROS2 driver
* `depth_image_topic` - Depth image topic to subscribe to, provided by the camera ROS2 driver
* `camera_calibration_mode` - calibration source mode: `auto`, `topic`, or `file`
* `camera_info_topic` - Camera info topic to subscribe to, providing intrinsic and distortion parameters
* `camera_info_timeout_sec` - topic wait time before local-file fallback in `auto` mode
* `camera_calibration_file` - path to the local VITA calibration YAML
* `camera_calibration_label` - camera label selected from the local calibration YAML
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)
* `detecter_markers_topic` - Topic to publish the detected markers as ArucoMarkers message
* `markers_visualization_topic` - Topic to publish the detected markers as PoseArray message
* `output_image_topic` - Topic to publish the output image with detected markers drawn on it, for visualization purposes

## Running Marker Detection for Pose Estimation

Launch the aruco pose estimation node with this command. The parameters will be loaded from _aruco\_parameters.yaml_,
but can also be changed directly in the launch file with command line arguments.

```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

Change the parameters directly in the launch file:

```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py marker_size:=0.1 aruco_dictionary_id:=DICT_5X5_250 camera_frame:=camera_link
```

## dog3 NV12 input

The checked-in configuration is set up for dog3:

* input image: `/image_left_raw/nv12_half` (`nv12`, 1280x720)
* camera calibration mode: `auto`
* camera info topic: `/stereo_left/camera_info`
* local calibration fallback: `/app_param/vita_calib.yaml`, label `stereo_left`
* marker size: `0.2` meters

For this input, the node extracts only the NV12 Y plane, resizes it to
1920x1080, detects ArUco corners on the resized luminance image, and then runs
`solvePnP`. Because the processing image matches the CameraInfo calibration
size, the received camera matrix is used without pixel scaling. If the
configured processing size changes, the node scales the camera matrix to match.

## 统一输入配置与跨机器狗移植

摄像头输入接口统一配置在
`aruco_pose_estimation/config/aruco_parameters.yaml`。正常移植时不需要修改
Python 节点或 Launch 文件，主要修改以下参数：

```yaml
image_topic: /image_left_raw/nv12_half
camera_calibration_mode: auto
camera_info_topic: /stereo_left/camera_info
camera_info_timeout_sec: 5.0
camera_calibration_file: /app_param/vita_calib.yaml
camera_calibration_label: stereo_left
camera_frame: stereo_left
resize_width: 1920
resize_height: 1080
```

内参读取模式：

* `auto`：优先订阅 `camera_info_topic`，超时后读取本地文件。若文件读取失败，继续等待话题。
* `topic`：只订阅内参话题，不读取本地文件。
* `file`：启动时直接读取本地文件，不订阅内参话题。

dog3 默认 `use_depth_input: false`，因此运行输入是图像以及一种内参来源。
只有设置为 `true` 时，才会额外订阅 `depth_image_topic`。

## 输出接口

工程不是只输出一个 ROS 话题，而是保留三个输出：

* `/output/aruco/markers` (`aruco_interfaces/msg/ArucoMarkers`)：主业务输出，包含二维码 ID 和对应 Pose。
* `/aruco/poses` (`geometry_msgs/msg/PoseArray`)：供 RViz 等工具显示 Pose。
* `/aruco/image` (`sensor_msgs/msg/Image`)：带二维码边框和坐标轴的 RGB8 图像。

`/output/aruco/markers` 和 `/aruco/poses` 只在识别到二维码时发布；`/aruco/image`
在每个完成处理的图像帧上发布。话题中的 Pose 保持 OpenCV 光学坐标系
（X 向右、Y 向下、Z 向前），表示“二维码相对摄像头”的位姿。终端日志另外转换成
前、左、上右手坐标系，便于直接阅读。

Build and run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

The launch file does not start a local RealSense driver or RViz by default.
They can be enabled with `launch_camera_driver:=true` and `launch_rviz:=true`.

On dog3, after building the workspace, use the environment-safe wrapper:

```bash
cd /userdata/6_cal_back_charging_location
./build_dog3.sh
./1_cal_back_charge_location.sh
```
