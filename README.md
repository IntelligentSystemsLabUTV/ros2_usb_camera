# ros2_usb_camera

Simple ROS 2 driver node for USB cameras compatible with the `Video4Linux` APIs. Based on `image_transport`, `camera_calibration` and `OpenCV`.

## Features

- `CameraInfo` topic.
- `CompressedImage` topic.
- `Image` topic.
- Camera selection, i.e. running multiple instances of the node, each one to drive a different camera.

## Usage

The code compiles to both a standalone application executable and a ROS 2 component, and both can be run easily. There are also launch files for both configurations.

You can use `RViz` to display the frames being streamed:

- topic `Reliability Policy` must be set to `Best Effort`;
- `Fixed Frame` in `Global Options` must be set to `map`.

### Node Parameters

Configuration files for node parameters can be found in `config`, with some standard default settings. They can be customized, or overridden from command line or launch files.

- `frame_id`: transform frame_id of the camera, defaults to `map`.
- `image_width`: image width, defaults to `640`.
- `image_height`: image height, defaults to `480`.
- `fps`: camera capture rate, defaults to `20`.
- `camera_id`: id of camera to be captured, defaults to `0`.

### Camera Calibration Files

TODO Check this.

To use the camera info functionality you need to load a file from the [camera_calibration](https://github.com/ros-perception/image_pipeline/tree/ros2/camera_calibration) library and put it in/name it `file:///Users/<youruser>/.ros/camera_info/camera.yaml`

### Compressed images

TODO Check this.

To get compressed images (works seamlessly with web streaming) republish the topic using image_transport which is available for ROS2.

```bash
ros2 run image_transport republish raw in:=image_raw compressed out:=image_raw_compressed
```

Make sure to link/install the [plugin](https://github.com/ros-perception/image_transport_plugins/tree/ros2) before to enable compressed image republishing using image_transport since its not included in the base package. More information [here](http://wiki.ros.org/image_transport), [here](http://wiki.ros.org/compressed_image_transport) and [here](https://answers.ros.org/question/35183/compressed-image-to-image/).
