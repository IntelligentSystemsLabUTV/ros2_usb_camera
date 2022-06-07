# ros2_usb_camera

Simple ROS 2 driver node for USB cameras compatible with the `Video4Linux` APIs. Based on `image_transport`, `camera_calibration` and `OpenCV`.

## Features

- `CameraInfo` topic.
- `CompressedImage` topic.
- `Image` topic.
- Supports namespace and node name remappings, in order to run different cameras with multiple instances of the node.
- ROS 2 component compilation and installation.
- Optimized memory handling.
- High-resolution, thread-based camera sampling.
- Offers both reliable and best-effort transmissions, configurable via node parameters.

## Usage

The code compiles to both a standalone application executable and a ROS 2 component, and both can be run easily. There is also a launch file for the standalone application.

You can use `RViz` to display the frames being streamed:

- topic `Reliability Policy` must be set to what the corresponding node parameter has been set to;
- `Fixed Frame` in `Global Options` must be set to what the corresponding node parameter has been set to.

### Node Parameters

Configuration files for node parameters can be found in `config`, with some standard default settings. They can be customized, or overridden from command line or launch files.

- `base_topic_name`: base transmission topic name for `image_transport` publishers.
- `best_effort_qos`: toggles unreliable but faster transmissions.
- `camera_calibration_file`: camera calibration YAML file URL.
- `camera_id`: ID of the video capture device to open.
- `fps`: camera capture rate, defaults to `20`.
- `frame_id`: transform frame_id of the camera, defaults to `map`.
- `image_height`: image height, defaults to `480`.
- `image_width`: image width, defaults to `640`.
- `is_flipped`: toggles vertical image flipping.

### Camera Calibration

The necessary parameters and camera intrinsics can be acquired from a standard calibration procedure. You can write your own routine for this, e.g. with `OpenCV`, or use the `camera_calibration cameracalibrator` tool as documented [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html).

## Copyright

Copyright © 2022 Intelligent Systems Lab
