# Aruco_detection
determination of depth in aruco marker real life using zed2i camera
# ZED 2i ArUco Marker Detection using ROS 2

A ROS 2 package for detecting ArUco markers using the RGB camera of the **Stereolabs ZED 2i stereo camera**. The system uses OpenCV for marker detection and pose estimation and ROS 2 topics to publish detection results.

## Features

- ArUco marker detection using OpenCV.
- Configurable ArUco dictionary and marker size.
- Pose estimation using camera calibration parameters.
- RGB image and depth information support.
- Optional OpenCV window for visualizing detections.
- ROS 2 topics for publishing marker IDs, poses, and measurements.
- Launch file and YAML configuration for convenient setup.

## Technology Stack

- **Operating System:** Ubuntu
- **Middleware:** ROS 2 Humble
- **Camera:** Stereolabs ZED 2i
- **Computer Vision:** OpenCV
- **Programming Language:** Python

## Project Structure

```text
ros2_ws/
├── src/
│   └── zed_aruco_detector/
│       ├── config/
│       │   └── aruco_params.yaml
│       ├── launch/
│       │   └── aruco.launch.py
│       ├── zed_aruco_detector/
│       │   ├── __init__.py
│       │   └── aruco.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── README.md
```

*Note: The exact files and folders may vary depending on the current version of the package.*

## Prerequisites

Ensure the following are installed and configured:

- Ubuntu with ROS 2 Humble.
- Python 3.
- OpenCV with ArUco module support.
- ZED 2i camera and its compatible ZED SDK.
- ZED ROS 2 wrapper.
- `colcon` build tools.

Install common ROS 2 build dependencies:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions \
                 python3-rosdep \
                 ros-humble-cv-bridge \
                 ros-humble-image-transport
```

Install the Python dependencies if required by your environment:

```bash
python3 -m pip install numpy opencv-contrib-python
```

**Important:** If you use the ZED SDK's bundled OpenCV or an existing ROS-compatible OpenCV installation, avoid installing a conflicting OpenCV version. The ArUco module must be available in the OpenCV environment used by the ROS 2 node.

## Installation

### 1. Clone the repository

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/karimsafiya43-gif/Aruco_detection.git
```

If the repository contains the complete workspace rather than just the package, inspect its folder structure before cloning it into another workspace.

### 2. Build the package

Navigate to the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select zed_aruco_detector
```

Source the workspace:

```bash
source install/setup.bash
```

To automatically source it in new terminals:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Camera Setup

Connect the ZED 2i camera and ensure the ZED SDK and ROS 2 wrapper are installed.

Start the ZED camera node in a separate terminal using the launch command appropriate for your installed wrapper:

```bash
source /opt/ros/humble/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

If the wrapper is installed in a separate workspace, source that workspace as well.

Verify that the camera topics are available:

```bash
ros2 topic list
```

Check whether RGB images are being published:

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

The detector requires an actively publishing RGB image topic.

## Running the Detector

Open another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch zed_aruco_detector aruco.launch.py
```

The launch file uses the package's configured parameters, including the ArUco dictionary, marker size, camera topics, and display settings.

## Configuration

The detector's parameters are configured in:

```text
config/aruco_params.yaml
```

Example settings used during development:

| Parameter | Example value | Description |
|---|---|---|
| ArUco dictionary | `DICT_4X4_50` | Dictionary used to identify markers |
| Marker size | `0.20` m | Physical marker side length |
| Show window | `true` | Enables the OpenCV display, if supported by the node |
| RGB topic | `/zed/zed_node/rgb/color/rect/image` | Rectified RGB images |
| Camera info topic | `/zed/zed_node/rgb/color/rect/camera_info` | Camera calibration information |
| Depth topic | `/zed/zed_node/depth/depth_registered` | Registered depth images |

These are example values; your YAML file is the source of truth for the parameters actually used by your installation.

**Marker size matters:** Measure the printed marker's actual side length and configure the same value in meters. Incorrect dimensions can produce inaccurate pose estimates.

## ROS 2 Output Topics

The detector is designed to publish its results through ROS 2 topics.

| Topic | Intended output |
|---|---|
| `/aruco/ids` | IDs of detected markers |
| `/aruco/poses` | Estimated marker poses |
| `/aruco/measurements` | Marker measurement data |

Inspect available topics:

```bash
ros2 topic list
```

Check detected marker IDs:

```bash
ros2 topic echo /aruco/ids
```

Check pose estimates:

```bash
ros2 topic echo /aruco/poses
```

Check measurements:

```bash
ros2 topic echo /aruco/measurements
```

These commands will show data only when the detector is running and publishing messages.

## Troubleshooting

### ZED camera topics exist but no images are published

Check the camera node and its logs. Confirm the ZED 2i is connected, the SDK recognizes it, and the wrapper is running.

```bash
ros2 node list
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

### `Package 'zed_wrapper' not found`

Source the workspace containing the ZED ROS 2 wrapper, or install/build a compatible wrapper version for ROS 2 Humble.

### OpenCV window does not appear

- Ensure the RGB topic is publishing.
- Check that `show_window` is enabled.
- Run the node in a graphical desktop session.
- Check that the code calls the relevant OpenCV display functions and processes incoming frames.

### No markers are detected

- Use a marker from the configured dictionary.
- Ensure the marker is clearly visible and sufficiently large in the image.
- Check lighting, focus, and image quality.
- Verify the configured marker size and camera calibration.

## Development

After modifying the source code, rebuild and source the workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select zed_aruco_detector
source install/setup.bash
```

## Future Improvements

- Improve detection robustness under varying lighting conditions.
- Add filtering to reduce noisy pose estimates.
- Improve depth-based validation.
- Integrate marker pose information with robot localization.
- Add automated tests and performance evaluation.

## License

Add the license applicable to this project before distributing or reusing the code.

## Author

Developed as a ROS 2 computer-vision project for ArUco marker detection using the ZED 2i camera.
